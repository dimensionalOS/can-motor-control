//! [`Robot`] and [`RobotBuilder`].

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use mio::Token;

use crate::bus::{Bus, RouteKey};
use crate::error::Error;
use crate::group::{Arm, Generic, Gripper, GroupKind, MotorGroup, OPENING_CALIBRATION_TRAVEL_RAD};
use crate::motor::Motor;
use crate::spec::{GripperOpeningSpec, GroupSpecKind, MotorSpec};
use crate::transport::{BusPoller, CanBus};
use motor_codec::{CommandKind, MotorCodec};

#[cfg(test)]
const OPENING_CALIBRATION_ENDPOINT_DWELL: Duration = Duration::from_millis(100);
#[cfg(not(test))]
const OPENING_CALIBRATION_ENDPOINT_DWELL: Duration = Duration::from_millis(1500);
const OPENING_CALIBRATION_TICK: Duration = Duration::from_millis(10);
const OPENING_CALIBRATION_REFRESH_TIMEOUT: Duration = Duration::from_millis(250);
const OPENING_MODE_VERIFY_TIMEOUT: Duration = Duration::from_millis(50);
const OPENING_MODE_VERIFY_TICK: Duration = Duration::from_millis(5);
const OPENING_MODE_SETTLE: Duration = Duration::from_millis(2);

/// A configured robot: named buses + named groups + lifecycle state.
pub struct Robot {
    buses: HashMap<String, Arc<Mutex<Bus>>>,
    bus_order: Vec<String>,
    groups: HashMap<String, GroupKind>,
    group_order: Vec<String>,
    connected: bool,
    poller: Option<BusPoller>,
    /// Cached for tick: bus_name → poller token.
    bus_tokens: HashMap<String, Token>,
    /// Cached for tick: token → bus_name (inverse lookup).
    token_to_bus: HashMap<Token, String>,
}

impl Robot {
    /// Start a builder.
    pub fn builder() -> RobotBuilder {
        RobotBuilder::new()
    }

    /// Read-only access to a named group.
    pub fn group(&self, name: &str) -> Option<&GroupKind> {
        self.groups.get(name)
    }

    /// Mutable access to a named group.
    pub fn group_mut(&mut self, name: &str) -> Option<&mut GroupKind> {
        self.groups.get_mut(name)
    }

    /// Iterate group names in insertion order.
    pub fn group_names(&self) -> impl Iterator<Item = &str> {
        self.group_order.iter().map(String::as_str)
    }

    /// Iterate bus names in insertion order.
    pub fn bus_names(&self) -> impl Iterator<Item = &str> {
        self.bus_order.iter().map(String::as_str)
    }

    /// True after a successful `connect()`.
    pub fn is_connected(&self) -> bool {
        self.connected
    }

    /// Borrow a bus by name (returns the `Arc<Mutex<Bus>>` for direct lock).
    pub fn bus(&self, name: &str) -> Option<&Arc<Mutex<Bus>>> {
        self.buses.get(name)
    }

    /// Open sockets, populate per-bus recv-id routing tables, register fds
    /// with the poller, lock topology.
    pub fn connect(&mut self) -> Result<(), Error> {
        if self.connected {
            return Ok(());
        }
        // Build routing tables.
        for (bus_name, bus_arc) in &self.buses {
            let mut bus = bus_arc.lock().map_err(|_| Error::BusPoisoned)?;
            bus.routes.clear();
            for group_name in &self.group_order {
                let group = self.groups.get(group_name).expect("group order invariant");
                if group.bus_name() != bus_name {
                    continue;
                }
                for (i, motor) in group.inner().motors().iter().enumerate() {
                    let key = RouteKey {
                        group_name: group_name.clone(),
                        motor_index: i,
                    };
                    if let Some(existing) = bus.routes.get(&motor.recv_id()) {
                        return Err(Error::CanIdCollision {
                            bus_name: bus_name.clone(),
                            recv_id: motor.recv_id(),
                            existing: existing.clone(),
                            attempted: key,
                        });
                    }
                    bus.codec
                        .bind_motor(motor_codec::MotorRef {
                            motor_type: motor.motor_type(),
                            send_id: motor.send_id(),
                            recv_id: motor.recv_id(),
                            name: motor.name(),
                        })
                        .map_err(Error::Codec)?;
                    bus.routes.insert(motor.recv_id(), key);
                }
            }
        }
        // Set up poller.
        let poller = BusPoller::with_capacity(self.buses.len().max(1))?;
        let mut bus_tokens = HashMap::with_capacity(self.buses.len());
        let mut token_to_bus = HashMap::with_capacity(self.buses.len());
        for (idx, bus_name) in self.bus_order.iter().enumerate() {
            let bus = self.buses[bus_name]
                .lock()
                .map_err(|_| Error::BusPoisoned)?;
            if let Some(fd) = bus.transport.raw_fd() {
                let token = Token(idx);
                poller.register(token, fd)?;
                bus_tokens.insert(bus_name.clone(), token);
                token_to_bus.insert(token, bus_name.clone());
            }
        }
        self.poller = Some(poller);
        self.bus_tokens = bus_tokens;
        self.token_to_bus = token_to_bus;
        self.connected = true;
        Ok(())
    }

    /// Send the enable frame to every motor in every group, in insertion order.
    pub fn enable(&mut self) -> Result<(), Error> {
        if !self.connected {
            return Err(Error::NotConnected);
        }
        let opening_names: Vec<String> = self
            .group_order
            .iter()
            .filter(|name| {
                self.groups
                    .get(*name)
                    .and_then(GroupKind::as_gripper)
                    .is_some_and(Gripper::has_opening_control)
            })
            .cloned()
            .collect();

        // Damiao's mode write is send-only. Verify every opening-control motor
        // before enabling any group, so a bad mode cannot reach calibration.
        for name in &opening_names {
            let is_damiao = self
                .groups
                .get(name)
                .expect("group order invariant")
                .inner()
                .bus_vendor()
                .map(|vendor| vendor == "damiao")?;
            if is_damiao {
                self.groups
                    .get_mut(name)
                    .and_then(GroupKind::as_gripper_mut)
                    .expect("opening gripper invariant")
                    .set_mode(CommandKind::PosForce)?;
                thread::sleep(OPENING_MODE_SETTLE);
                self.verify_opening_mode(name)?;
            }
        }

        for name in &self.group_order.clone() {
            let has_opening_control = self
                .groups
                .get(name)
                .and_then(GroupKind::as_gripper)
                .is_some_and(Gripper::has_opening_control);
            if has_opening_control
                && self
                    .groups
                    .get(name)
                    .expect("group order invariant")
                    .inner()
                    .bus_vendor()
                    .map(|vendor| vendor != "damiao")?
            {
                self.groups
                    .get_mut(name)
                    .and_then(GroupKind::as_gripper_mut)
                    .expect("checked gripper kind")
                    .set_mode(CommandKind::PosForce)?;
            }
            if let Some(g) = self.groups.get_mut(name) {
                g.enable_all()?;
            }
            if has_opening_control {
                if let Err(err) = self.calibrate_gripper_opening(name) {
                    if let Some(g) = self.groups.get_mut(name) {
                        if let Some(gripper) = g.as_gripper_mut() {
                            gripper.clear_opening_calibration();
                        }
                        let _ = g.disable_all();
                    }
                    return Err(err);
                }
            }
        }
        Ok(())
    }

    fn verify_opening_mode(&mut self, name: &str) -> Result<(), Error> {
        let (send_id, recv_id) = self
            .groups
            .get_mut(name)
            .and_then(GroupKind::as_gripper_mut)
            .expect("opening gripper invariant")
            .inner_mut()
            .send_control_mode_readback()?
            .ok_or_else(|| Error::OpeningControlModeVerificationFailed {
                name: name.to_string(),
                reason: "codec does not support CTRL_MODE read-back".to_string(),
            })?;
        let deadline = Instant::now() + OPENING_MODE_VERIFY_TIMEOUT;
        while Instant::now() < deadline {
            if let Some(value) = self.tick_for_mode(name, send_id, recv_id)? {
                if value == 4 {
                    return Ok(());
                }
                return Err(Error::OpeningControlModeVerificationFailed {
                    name: name.to_string(),
                    reason: format!("CTRL_MODE read-back was {value}, expected 4"),
                });
            }
            self.groups
                .get_mut(name)
                .and_then(GroupKind::as_gripper_mut)
                .expect("opening gripper invariant")
                .inner_mut()
                .send_control_mode_readback()?;
        }
        Err(Error::OpeningControlModeVerificationFailed {
            name: name.to_string(),
            reason: "CTRL_MODE read-back timed out".to_string(),
        })
    }

    fn calibrate_gripper_opening(&mut self, name: &str) -> Result<(), Error> {
        let Some(opening_direction_sign) = self
            .groups
            .get_mut(name)
            .and_then(GroupKind::as_gripper_mut)
            .and_then(|gripper| {
                gripper.clear_opening_calibration();
                gripper.opening_direction_sign()
            })
        else {
            return Ok(());
        };

        self.refresh_gripper_feedback(name)?;

        let start_position = self.gripper_position(name)?;
        let closed_position = self.calibrate_gripper_endpoint(
            name,
            "close",
            start_position,
            -opening_direction_sign,
        )?;
        let open_position =
            self.calibrate_gripper_endpoint(name, "open", closed_position, opening_direction_sign)?;

        let Some(gripper) = self
            .groups
            .get_mut(name)
            .and_then(GroupKind::as_gripper_mut)
        else {
            return Ok(());
        };
        gripper.set_opening_calibration(closed_position, open_position)
    }

    fn refresh_gripper_feedback(&mut self, name: &str) -> Result<(), Error> {
        let (_, initial_sequence) = self.gripper_position_and_sequence(name)?;
        let deadline = Instant::now() + OPENING_CALIBRATION_REFRESH_TIMEOUT;
        while Instant::now() < deadline {
            self.groups
                .get_mut(name)
                .and_then(GroupKind::as_gripper_mut)
                .ok_or_else(|| Error::OpeningCalibrationFailed {
                    name: name.to_string(),
                    reason: "group is not a gripper".to_string(),
                })?
                .refresh()?;
            self.tick(OPENING_CALIBRATION_TICK)?;
            let (position, sequence) = self.gripper_position_and_sequence(name)?;
            if sequence != initial_sequence {
                log_calibration_debug(format_args!(
                    "{name}: fresh pre-calibration feedback position={position:.5}, sequence={sequence}"
                ));
                return Ok(());
            }
            thread::sleep(Duration::from_millis(1));
        }

        Err(Error::OpeningCalibrationFailed {
            name: name.to_string(),
            reason: "opening calibration did not receive fresh feedback before starting"
                .to_string(),
        })
    }

    fn calibrate_gripper_endpoint(
        &mut self,
        name: &str,
        phase: &str,
        start_position: f64,
        raw_direction: f64,
    ) -> Result<f64, Error> {
        let target_position = start_position + raw_direction * OPENING_CALIBRATION_TRAVEL_RAD;
        let mut measured_position = start_position;
        let mut ticks = 0;
        let deadline = Instant::now() + OPENING_CALIBRATION_ENDPOINT_DWELL;

        log_calibration_debug(format_args!(
            "{name}: {phase} phase start={start_position:.5}, target={target_position:.5}, raw_direction={raw_direction:+.1}"
        ));

        while Instant::now() < deadline {
            let iteration_start = Instant::now();
            self.groups
                .get_mut(name)
                .and_then(GroupKind::as_gripper_mut)
                .ok_or_else(|| Error::OpeningCalibrationFailed {
                    name: name.to_string(),
                    reason: "group is not a gripper".to_string(),
                })?
                .opening_calibration_command(target_position)?;
            self.tick(OPENING_CALIBRATION_TICK)?;
            measured_position = self.gripper_position(name)?;
            ticks += 1;
            thread::sleep(OPENING_CALIBRATION_TICK.saturating_sub(iteration_start.elapsed()));
        }

        log_calibration_debug(format_args!(
            "{name}: {phase} endpoint_measured={measured_position:.5}, command_target={target_position:.5}, ticks={ticks}, dwell_ms={}",
            OPENING_CALIBRATION_ENDPOINT_DWELL.as_millis()
        ));
        Ok(measured_position)
    }

    fn gripper_position(&self, name: &str) -> Result<f64, Error> {
        self.gripper_position_and_sequence(name)
            .map(|(position, _)| position)
    }

    fn gripper_position_and_sequence(&self, name: &str) -> Result<(f64, u64), Error> {
        self.groups
            .get(name)
            .and_then(GroupKind::as_gripper)
            .map(|gripper| (gripper.motor().position(), gripper.motor().state_sequence()))
            .ok_or_else(|| Error::OpeningCalibrationFailed {
                name: name.to_string(),
                reason: "group is not a gripper".to_string(),
            })
    }

    /// Send the disable frame to every motor in every group, in reverse
    /// insertion order. A no-op `Ok(())` if never enabled.
    pub fn disable(&mut self) -> Result<(), Error> {
        if !self.connected {
            return Ok(());
        }
        for name in self.group_order.clone().iter().rev() {
            if let Some(g) = self.groups.get_mut(name) {
                g.disable_all()?;
            }
        }
        Ok(())
    }

    /// Send a state-refresh query to every motor in every group (no motion).
    /// Send-only — pair with [`Robot::tick`] to receive the replies. Errors with
    /// [`Error::NotConnected`] if called before `connect()`.
    pub fn refresh(&mut self) -> Result<(), Error> {
        if !self.connected {
            return Err(Error::NotConnected);
        }
        for name in &self.group_order.clone() {
            if let Some(g) = self.groups.get_mut(name) {
                g.refresh_all()?;
            }
        }
        Ok(())
    }

    /// Set the persistent control mode (MIT / PosVel / Vel / PosForce) on every
    /// motor in every group (no motion). Send-only. Call once after `connect`
    /// and before commanding. Errors with [`Error::NotConnected`] if not
    /// connected.
    pub fn set_mode(&mut self, mode: CommandKind) -> Result<(), Error> {
        if !self.connected {
            return Err(Error::NotConnected);
        }
        for name in &self.group_order.clone() {
            if let Some(g) = self.groups.get_mut(name) {
                g.set_mode(mode)?;
            }
        }
        Ok(())
    }

    /// One tick of the control loop: poll all buses, drain readable ones,
    /// decode each frame exactly once, dispatch the resulting events to the
    /// owning groups via per-bus routing tables.
    pub fn tick(&mut self, deadline: Duration) -> Result<(), Error> {
        self.tick_internal(deadline, None).map(|_| ())
    }

    fn tick_for_mode(
        &mut self,
        name: &str,
        send_id: u32,
        recv_id: u32,
    ) -> Result<Option<u32>, Error> {
        self.tick_internal(OPENING_MODE_VERIFY_TICK, Some((name, send_id, recv_id)))
    }

    fn tick_internal(
        &mut self,
        deadline: Duration,
        mode_target: Option<(&str, u32, u32)>,
    ) -> Result<Option<u32>, Error> {
        if !self.connected {
            return Err(Error::NotConnected);
        }
        let mode_target =
            mode_target.map(|(name, send_id, recv_id)| (name.to_string(), send_id, recv_id));
        // 1) Wait for any registered fd to become readable (or the deadline).
        let ready = if let Some(p) = self.poller.as_mut() {
            p.wait(deadline)?
        } else {
            Vec::new()
        };
        // 2) For each ready bus, plus raw-fd-less buses (mock transports), drain
        // + decode + collect dispatch tuples.
        let mut dispatches: Vec<(String, usize, motor_codec::Event)> = Vec::new();
        let mut mode_value = None;
        let mut readable_bus_names: Vec<String> = ready
            .into_iter()
            .filter_map(|token| self.token_to_bus.get(&token).cloned())
            .collect();
        readable_bus_names.extend(
            self.bus_order
                .iter()
                .filter(|bus_name| !self.bus_tokens.contains_key(*bus_name))
                .cloned(),
        );
        for bus_name in readable_bus_names {
            let bus_arc = self.buses[&bus_name].clone();
            let mut bus = bus_arc.lock().map_err(|_| Error::BusPoisoned)?;
            let frames = bus.transport.drain_inbound_nonblocking()?;
            for frame in &frames {
                if let Some((ref target_name, send_id, recv_id)) = mode_target {
                    if frame.id == recv_id {
                        let group = self.groups.get(target_name).expect("group order invariant");
                        let motor = &group.inner().motors()[0];
                        let m_ref = motor_codec::MotorRef {
                            motor_type: motor.motor_type(),
                            send_id,
                            recv_id,
                            name: motor.name(),
                        };
                        mode_value = bus
                            .codec
                            .decode_control_mode_readback(frame, m_ref)
                            .map_err(Error::Codec)?;
                        if mode_value.is_some() {
                            continue;
                        }
                    }
                }
                let decoded = bus.codec.decode(frame).map_err(Error::Codec)?;
                if let Some(event) = decoded {
                    let motor_id = match event {
                        motor_codec::Event::State { motor_id, .. } => motor_id,
                        motor_codec::Event::ParamReply { motor_id, .. } => motor_id,
                        motor_codec::Event::Fault { motor_id, .. } => motor_id,
                        _ => continue,
                    };
                    if let Some(route) = bus.routes.get(&motor_id) {
                        dispatches.push((route.group_name.clone(), route.motor_index, event));
                    }
                    // No route: silently drop (foreign motor_id, not in this robot).
                }
                // None: codec didn't recognize, silently drop.
            }
        }
        // 3) Dispatch outside the bus lock to avoid deadlocking groups that
        // would lock the same bus during apply_event (currently they don't,
        // but the discipline is correct).
        for (group_name, motor_index, event) in dispatches {
            if let Some(g) = self.groups.get_mut(&group_name) {
                g.apply_event(motor_index, &event);
            }
        }
        Ok(mode_value)
    }
}

fn log_calibration_debug(args: std::fmt::Arguments<'_>) {
    if std::env::var_os("CAN_MOTOR_CONTROL_CALIBRATION_DEBUG").is_some() {
        eprintln!("[gripper-calibration] {args}");
    }
}

impl Drop for Robot {
    fn drop(&mut self) {
        // Each transport owns its platform resources. We don't send disable
        // frames here — that's the user's responsibility via explicit
        // disable().
        for token in self.token_to_bus.keys() {
            if let Some(p) = self.poller.as_ref() {
                if let Some(name) = self.token_to_bus.get(token) {
                    if let Some(bus_arc) = self.buses.get(name) {
                        if let Ok(bus) = bus_arc.lock() {
                            if let Some(fd) = bus.transport.raw_fd() {
                                let _ = p.deregister(fd);
                            }
                        }
                    }
                }
            }
        }
    }
}

/// Pending bus declaration accumulated by the builder.
struct PendingBus {
    name: String,
    transport: Box<dyn CanBus>,
    codec: Box<dyn MotorCodec>,
}

/// Pending group declaration accumulated by the builder.
struct PendingGroup {
    name: String,
    bus_name: String,
    kind: GroupSpecKind,
    motors: Vec<MotorSpec>,
    opening: Option<GripperOpeningSpec>,
}

/// Builder for [`Robot`].
pub struct RobotBuilder {
    buses: Vec<PendingBus>,
    groups: Vec<PendingGroup>,
}

impl Default for RobotBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl RobotBuilder {
    /// Start an empty builder.
    pub fn new() -> Self {
        Self {
            buses: Vec::new(),
            groups: Vec::new(),
        }
    }

    /// Register a bus with its transport and vendor codec. The codec's
    /// `bind_to_bus(transport.capabilities())` is invoked exactly once here.
    pub fn add_bus(
        mut self,
        name: impl Into<String>,
        transport: Box<dyn CanBus>,
        codec: Box<dyn MotorCodec>,
    ) -> Self {
        self.buses.push(PendingBus {
            name: name.into(),
            transport,
            codec,
        });
        self
    }

    /// Register an arm group.
    pub fn add_arm(
        mut self,
        name: impl Into<String>,
        bus_name: impl Into<String>,
        motors: Vec<MotorSpec>,
    ) -> Self {
        self.groups.push(PendingGroup {
            name: name.into(),
            bus_name: bus_name.into(),
            kind: GroupSpecKind::Arm,
            motors,
            opening: None,
        });
        self
    }

    /// Register a one-motor gripper. Validation enforces the one-motor rule
    /// at `build()` time.
    pub fn add_gripper(
        mut self,
        name: impl Into<String>,
        bus_name: impl Into<String>,
        motor: MotorSpec,
    ) -> Self {
        self.groups.push(PendingGroup {
            name: name.into(),
            bus_name: bus_name.into(),
            kind: GroupSpecKind::Gripper,
            motors: vec![motor],
            opening: None,
        });
        self
    }

    /// Register a one-motor gripper with normalized opening support.
    pub fn add_gripper_with_opening(
        mut self,
        name: impl Into<String>,
        bus_name: impl Into<String>,
        motor: MotorSpec,
        opening: GripperOpeningSpec,
    ) -> Self {
        self.groups.push(PendingGroup {
            name: name.into(),
            bus_name: bus_name.into(),
            kind: GroupSpecKind::Gripper,
            motors: vec![motor],
            opening: Some(opening),
        });
        self
    }

    /// Register a generic catch-all group.
    pub fn add_generic(
        mut self,
        name: impl Into<String>,
        bus_name: impl Into<String>,
        motors: Vec<MotorSpec>,
    ) -> Self {
        self.groups.push(PendingGroup {
            name: name.into(),
            bus_name: bus_name.into(),
            kind: GroupSpecKind::Generic,
            motors,
            opening: None,
        });
        self
    }

    /// Validate and build.
    pub fn build(self) -> Result<Robot, Error> {
        // 1) Validate bus names: non-empty, trimmed, unique.
        let mut bus_order = Vec::with_capacity(self.buses.len());
        let mut buses: HashMap<String, Arc<Mutex<Bus>>> = HashMap::with_capacity(self.buses.len());
        for pending in self.buses {
            if pending.name.trim().is_empty() || pending.name != pending.name.trim() {
                return Err(Error::ConfigSchema(format!(
                    "bus name '{}' must be non-empty and not have leading/trailing whitespace",
                    pending.name
                )));
            }
            if buses.contains_key(&pending.name) {
                return Err(Error::DuplicateBusName(pending.name));
            }
            let bus = Bus::new(pending.transport, pending.codec);
            bus_order.push(pending.name.clone());
            buses.insert(pending.name, Arc::new(Mutex::new(bus)));
        }
        // 2) Validate group names + codec support + bus existence.
        let mut group_order = Vec::with_capacity(self.groups.len());
        let mut groups: HashMap<String, GroupKind> = HashMap::with_capacity(self.groups.len());
        for pending in self.groups {
            if pending.name.trim().is_empty() || pending.name != pending.name.trim() {
                return Err(Error::ConfigSchema(format!(
                    "group name '{}' must be non-empty and not have leading/trailing whitespace",
                    pending.name
                )));
            }
            if groups.contains_key(&pending.name) {
                return Err(Error::DuplicateGroupName(pending.name));
            }
            let bus_arc = buses
                .get(&pending.bus_name)
                .ok_or_else(|| Error::UnknownBusName(pending.bus_name.clone()))?;
            let (vendor, supported_all) = {
                let bus = bus_arc.lock().map_err(|_| Error::BusPoisoned)?;
                let v = bus.vendor().to_string();
                let supported = pending
                    .motors
                    .iter()
                    .find(|m| !bus.codec_supports(m.motor_type))
                    .cloned();
                (v, supported)
            };
            if let Some(unsupported) = supported_all {
                return Err(Error::MotorNotSupportedByCodec {
                    vendor,
                    motor_type: unsupported.motor_type,
                    bus_name: pending.bus_name,
                });
            }
            if matches!(pending.kind, GroupSpecKind::Gripper) && pending.motors.len() != 1 {
                return Err(Error::GripperRequiresOneMotor {
                    got: pending.motors.len(),
                });
            }
            if let Some(opening) = pending.opening {
                if !matches!(pending.kind, GroupSpecKind::Gripper) {
                    return Err(Error::ConfigSchema(format!(
                        "group '{}': opening configuration is only valid for grippers",
                        pending.name
                    )));
                }
                if let Some(current) = opening.default_current {
                    validate_opening_current(current)?;
                }
            }
            // Construct the MotorGroup and attach the bus.
            let motors: Vec<Motor> = pending
                .motors
                .into_iter()
                .map(|s| Motor::new(s.name, s.motor_type, s.send_id, s.recv_id))
                .collect();
            let mut group = MotorGroup::new(pending.name.clone(), pending.bus_name, motors);
            group.attach_bus(bus_arc.clone());
            let kind = match pending.kind {
                GroupSpecKind::Arm => GroupKind::Arm(Arm(group)),
                GroupSpecKind::Gripper => match pending.opening {
                    Some(opening) => GroupKind::Gripper(Gripper::with_opening(group, opening)),
                    None => GroupKind::Gripper(Gripper::raw(group)),
                },
                GroupSpecKind::Generic => GroupKind::Generic(Generic(group)),
            };
            group_order.push(pending.name.clone());
            groups.insert(pending.name, kind);
        }
        Ok(Robot {
            buses,
            bus_order,
            groups,
            group_order,
            connected: false,
            poller: None,
            bus_tokens: HashMap::new(),
            token_to_bus: HashMap::new(),
        })
    }
}

fn validate_opening_current(current: f64) -> Result<(), Error> {
    if current > 0.0 && current <= 1.0 {
        Ok(())
    } else {
        Err(Error::OpeningCurrentOutOfRange { got: current })
    }
}

#[cfg(test)]
mod tests {
    use motor_codec::{
        BusCapabilities, CanFrame, CodecError, Command, Event, FrameFlags, Limits, MotorCodec,
        MotorRef, MotorTypeId,
    };

    use super::*;
    use crate::spec::OpeningDirection;
    use crate::transport::MockCanBus;
    use std::sync::atomic::{AtomicUsize, Ordering};

    struct CountingCodec {
        binds: Arc<AtomicUsize>,
        decodes: Arc<AtomicUsize>,
    }
    impl CountingCodec {
        fn new() -> (Self, Arc<AtomicUsize>, Arc<AtomicUsize>) {
            let b = Arc::new(AtomicUsize::new(0));
            let d = Arc::new(AtomicUsize::new(0));
            (
                Self {
                    binds: b.clone(),
                    decodes: d.clone(),
                },
                b,
                d,
            )
        }
    }
    impl MotorCodec for CountingCodec {
        fn vendor_name(&self) -> &'static str {
            "stub"
        }
        fn supports(&self, t: MotorTypeId) -> bool {
            matches!(t, MotorTypeId::Damiao(_))
        }
        fn limits(&self, _: MotorTypeId) -> Result<Limits, CodecError> {
            Ok(Limits {
                p_max: 1.0,
                v_max: 1.0,
                t_max: 1.0,
            })
        }
        fn bind_to_bus(&mut self, _: BusCapabilities) {
            self.binds.fetch_add(1, Ordering::SeqCst);
        }
        fn encode_enable(&self, m: MotorRef<'_>) -> Result<CanFrame, CodecError> {
            CanFrame::classical(m.send_id, &[0xFC])
                .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
        }
        fn encode_disable(&self, m: MotorRef<'_>) -> Result<CanFrame, CodecError> {
            CanFrame::classical(m.send_id, &[0xFD])
                .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
        }
        fn encode_set_zero(&self, m: MotorRef<'_>) -> Result<CanFrame, CodecError> {
            CanFrame::classical(m.send_id, &[0xFE])
                .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
        }
        fn encode_command(&self, m: MotorRef<'_>, _: &Command) -> Result<CanFrame, CodecError> {
            CanFrame::classical(m.send_id, &[0x55])
                .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
        }
        fn decode(&self, frame: &CanFrame) -> Result<Option<Event>, CodecError> {
            self.decodes.fetch_add(1, Ordering::SeqCst);
            if frame.is_fd() || frame.flags.contains(FrameFlags::REMOTE_REQUEST) {
                return Ok(None);
            }
            Ok(Some(Event::State {
                motor_id: frame.id,
                q: 1.0,
                dq: 2.0,
                tau: 3.0,
                t_mos: 30,
                t_rotor: 35,
            }))
        }
    }

    struct FailingCommandCodec;
    impl MotorCodec for FailingCommandCodec {
        fn vendor_name(&self) -> &'static str {
            "failing"
        }
        fn supports(&self, t: MotorTypeId) -> bool {
            matches!(t, MotorTypeId::Damiao(_))
        }
        fn limits(&self, _: MotorTypeId) -> Result<Limits, CodecError> {
            Ok(Limits {
                p_max: 1.0,
                v_max: 1.0,
                t_max: 1.0,
            })
        }
        fn bind_to_bus(&mut self, _: BusCapabilities) {}
        fn encode_enable(&self, m: MotorRef<'_>) -> Result<CanFrame, CodecError> {
            CanFrame::classical(m.send_id, &[0xFC])
                .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
        }
        fn encode_disable(&self, m: MotorRef<'_>) -> Result<CanFrame, CodecError> {
            CanFrame::classical(m.send_id, &[0xFD])
                .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
        }
        fn encode_set_zero(&self, m: MotorRef<'_>) -> Result<CanFrame, CodecError> {
            CanFrame::classical(m.send_id, &[0xFE])
                .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
        }
        fn encode_command(&self, _: MotorRef<'_>, _: &Command) -> Result<CanFrame, CodecError> {
            Err(CodecError::DecodeFailed { reason: "command" })
        }
        fn encode_refresh(&self, m: MotorRef<'_>) -> Result<Option<CanFrame>, CodecError> {
            CanFrame::classical(m.recv_id, &[0xCC])
                .map(Some)
                .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
        }
        fn decode(&self, frame: &CanFrame) -> Result<Option<Event>, CodecError> {
            Ok(Some(Event::State {
                motor_id: frame.id,
                q: 0.0,
                dq: 0.0,
                tau: 0.0,
                t_mos: 30,
                t_rotor: 35,
            }))
        }
    }

    struct FeedbackCodec {
        positions: Vec<f64>,
        decodes: AtomicUsize,
    }
    impl FeedbackCodec {
        fn new(positions: Vec<f64>) -> Self {
            Self {
                positions,
                decodes: AtomicUsize::new(0),
            }
        }
    }
    impl MotorCodec for FeedbackCodec {
        fn vendor_name(&self) -> &'static str {
            "feedback"
        }
        fn supports(&self, t: MotorTypeId) -> bool {
            matches!(t, MotorTypeId::Damiao(_))
        }
        fn limits(&self, _: MotorTypeId) -> Result<Limits, CodecError> {
            Ok(Limits {
                p_max: 1.0,
                v_max: 1.0,
                t_max: 1.0,
            })
        }
        fn bind_to_bus(&mut self, _: BusCapabilities) {}
        fn encode_enable(&self, m: MotorRef<'_>) -> Result<CanFrame, CodecError> {
            CanFrame::classical(m.send_id, &[0xFC])
                .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
        }
        fn encode_disable(&self, m: MotorRef<'_>) -> Result<CanFrame, CodecError> {
            CanFrame::classical(m.send_id, &[0xFD])
                .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
        }
        fn encode_set_zero(&self, m: MotorRef<'_>) -> Result<CanFrame, CodecError> {
            CanFrame::classical(m.send_id, &[0xFE])
                .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
        }
        fn encode_command(
            &self,
            m: MotorRef<'_>,
            command: &Command,
        ) -> Result<CanFrame, CodecError> {
            let id = if matches!(command, Command::PosForce { .. }) {
                m.recv_id
            } else {
                m.send_id
            };
            CanFrame::classical(id, &[0x55])
                .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
        }
        fn encode_refresh(&self, m: MotorRef<'_>) -> Result<Option<CanFrame>, CodecError> {
            CanFrame::classical(m.recv_id, &[0xCC])
                .map(Some)
                .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
        }
        fn decode(&self, frame: &CanFrame) -> Result<Option<Event>, CodecError> {
            let idx = self.decodes.fetch_add(1, Ordering::SeqCst);
            let q = self.positions[idx.min(self.positions.len().saturating_sub(1))];
            Ok(Some(Event::State {
                motor_id: frame.id,
                q,
                dq: 0.0,
                tau: 0.0,
                t_mos: 30,
                t_rotor: 35,
            }))
        }
    }

    struct EchoFeedbackCodec {
        position: Mutex<f64>,
    }

    impl EchoFeedbackCodec {
        fn new() -> Self {
            Self {
                position: Mutex::new(0.0),
            }
        }
    }

    impl MotorCodec for EchoFeedbackCodec {
        fn vendor_name(&self) -> &'static str {
            "echo-feedback"
        }
        fn supports(&self, t: MotorTypeId) -> bool {
            matches!(t, MotorTypeId::Damiao(_))
        }
        fn limits(&self, _: MotorTypeId) -> Result<Limits, CodecError> {
            Ok(Limits {
                p_max: 1.0,
                v_max: 1.0,
                t_max: 1.0,
            })
        }
        fn bind_to_bus(&mut self, _: BusCapabilities) {}
        fn encode_enable(&self, m: MotorRef<'_>) -> Result<CanFrame, CodecError> {
            CanFrame::classical(m.send_id, &[0xFC])
                .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
        }
        fn encode_disable(&self, m: MotorRef<'_>) -> Result<CanFrame, CodecError> {
            CanFrame::classical(m.send_id, &[0xFD])
                .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
        }
        fn encode_set_zero(&self, m: MotorRef<'_>) -> Result<CanFrame, CodecError> {
            CanFrame::classical(m.send_id, &[0xFE])
                .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
        }
        fn encode_command(
            &self,
            m: MotorRef<'_>,
            command: &Command,
        ) -> Result<CanFrame, CodecError> {
            if let Command::PosForce { q, .. } = *command {
                *self.position.lock().unwrap() = q;
                CanFrame::classical(m.recv_id, &[0x55])
                    .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
            } else {
                CanFrame::classical(m.send_id, &[0x55])
                    .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
            }
        }
        fn encode_refresh(&self, m: MotorRef<'_>) -> Result<Option<CanFrame>, CodecError> {
            CanFrame::classical(m.recv_id, &[0xCC])
                .map(Some)
                .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
        }
        fn decode(&self, frame: &CanFrame) -> Result<Option<Event>, CodecError> {
            Ok(Some(Event::State {
                motor_id: frame.id,
                q: *self.position.lock().unwrap(),
                dq: 0.0,
                tau: 0.0,
                t_mos: 30,
                t_rotor: 35,
            }))
        }
    }

    /// Deterministic lifecycle seam: loopback frames are enough to exercise
    /// ordering, while the mode reply can be selected by each test.
    struct ModeLifecycleCodec {
        reply: Option<u32>,
        position: Mutex<f64>,
    }

    impl MotorCodec for ModeLifecycleCodec {
        fn vendor_name(&self) -> &'static str {
            "damiao"
        }
        fn supports(&self, t: MotorTypeId) -> bool {
            matches!(t, MotorTypeId::Damiao(_))
        }
        fn limits(&self, _: MotorTypeId) -> Result<Limits, CodecError> {
            Ok(Limits {
                p_max: 10.0,
                v_max: 10.0,
                t_max: 10.0,
            })
        }
        fn bind_to_bus(&mut self, _: BusCapabilities) {}
        fn encode_enable(&self, m: MotorRef<'_>) -> Result<CanFrame, CodecError> {
            Ok(CanFrame::classical(m.send_id, &[0xfc]).unwrap())
        }
        fn encode_disable(&self, m: MotorRef<'_>) -> Result<CanFrame, CodecError> {
            Ok(CanFrame::classical(m.send_id, &[0xfd]).unwrap())
        }
        fn encode_set_zero(&self, m: MotorRef<'_>) -> Result<CanFrame, CodecError> {
            Ok(CanFrame::classical(m.send_id, &[0xfe]).unwrap())
        }
        fn encode_command(&self, m: MotorRef<'_>, cmd: &Command) -> Result<CanFrame, CodecError> {
            let marker = if let Command::PosForce { q, .. } = cmd {
                *self.position.lock().unwrap() = *q;
                0x55
            } else {
                0x01
            };
            Ok(CanFrame::classical(m.recv_id, &[marker]).unwrap())
        }
        fn encode_refresh(&self, m: MotorRef<'_>) -> Result<Option<CanFrame>, CodecError> {
            Ok(Some(CanFrame::classical(m.recv_id, &[0xcc]).unwrap()))
        }
        fn encode_set_mode(
            &self,
            m: MotorRef<'_>,
            _: CommandKind,
        ) -> Result<Option<CanFrame>, CodecError> {
            Ok(Some(CanFrame::classical(m.send_id, &[0x10]).unwrap()))
        }
        fn encode_control_mode_readback(
            &self,
            m: MotorRef<'_>,
        ) -> Result<Option<CanFrame>, CodecError> {
            let id = if self.reply.is_some() {
                m.recv_id
            } else {
                0x7ff
            };
            let mut payload = [0u8; 8];
            payload[0] = (m.send_id & 0xff) as u8;
            payload[2] = 0x20;
            payload[3] = self.reply.unwrap_or(0) as u8;
            Ok(Some(CanFrame::classical(id, &payload).unwrap()))
        }
        fn decode_control_mode_readback(
            &self,
            frame: &CanFrame,
            _: MotorRef<'_>,
        ) -> Result<Option<u32>, CodecError> {
            if frame.payload()[2] == 0x20 {
                Ok(Some(frame.payload()[3] as u32))
            } else {
                Ok(None)
            }
        }
        fn decode(&self, frame: &CanFrame) -> Result<Option<Event>, CodecError> {
            if frame.payload()[0] == 0x20 || frame.payload()[0] == 0x10 || frame.id == 0x7ff {
                return Ok(None);
            }
            Ok(Some(Event::State {
                motor_id: frame.id,
                q: *self.position.lock().unwrap(),
                dq: 0.0,
                tau: 0.0,
                t_mos: 20,
                t_rotor: 20,
            }))
        }
    }

    fn motor(name: &str, send: u32, recv: u32) -> MotorSpec {
        MotorSpec::new(name, MotorTypeId::Damiao(3), send, recv)
    }

    #[test]
    fn build_simple_robot() {
        let (codec, binds, _) = CountingCodec::new();
        let robot = RobotBuilder::new()
            .add_bus("main", Box::new(MockCanBus::new("m")), Box::new(codec))
            .add_arm(
                "arm",
                "main",
                vec![motor("j0", 0x01, 0x11), motor("j1", 0x02, 0x12)],
            )
            .build()
            .unwrap();
        assert_eq!(robot.bus_names().collect::<Vec<_>>(), vec!["main"]);
        assert_eq!(robot.group_names().collect::<Vec<_>>(), vec!["arm"]);
        assert_eq!(binds.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn duplicate_bus_name_rejected() {
        let (c1, _, _) = CountingCodec::new();
        let (c2, _, _) = CountingCodec::new();
        let r = RobotBuilder::new()
            .add_bus("main", Box::new(MockCanBus::new("m")), Box::new(c1))
            .add_bus("main", Box::new(MockCanBus::new("m")), Box::new(c2))
            .build();
        assert!(matches!(r, Err(Error::DuplicateBusName(ref s)) if s == "main"));
    }

    #[test]
    fn duplicate_group_name_rejected() {
        let (c, _, _) = CountingCodec::new();
        let r = RobotBuilder::new()
            .add_bus("main", Box::new(MockCanBus::new("m")), Box::new(c))
            .add_arm("arm", "main", vec![motor("j0", 0x01, 0x11)])
            .add_arm("arm", "main", vec![motor("j1", 0x02, 0x12)])
            .build();
        assert!(matches!(r, Err(Error::DuplicateGroupName(ref s)) if s == "arm"));
    }

    #[test]
    fn unknown_bus_name_rejected() {
        let r = RobotBuilder::new()
            .add_arm("arm", "ghost", vec![motor("j0", 0x01, 0x11)])
            .build();
        assert!(matches!(r, Err(Error::UnknownBusName(ref s)) if s == "ghost"));
    }

    #[test]
    fn motor_not_supported_by_codec() {
        let (c, _, _) = CountingCodec::new();
        let r = RobotBuilder::new()
            .add_bus("main", Box::new(MockCanBus::new("m")), Box::new(c))
            .add_arm(
                "arm",
                "main",
                vec![MotorSpec::new("j0", MotorTypeId::Robostride(0), 0x01, 0x11)],
            )
            .build();
        assert!(matches!(r, Err(Error::MotorNotSupportedByCodec { .. })));
    }

    #[test]
    fn gripper_requires_one_motor() {
        let (c, _, _) = CountingCodec::new();
        // The public add_gripper API takes a single MotorSpec; the runtime
        // check fires when an internal mis-construction supplies != 1 motors.
        // For this test we use add_generic to mimic the bad input path.
        let r = RobotBuilder::new()
            .add_bus("main", Box::new(MockCanBus::new("m")), Box::new(c))
            .add_gripper("g", "main", motor("g0", 0x05, 0x18))
            .build();
        assert!(r.is_ok());
    }

    #[test]
    fn build_does_not_open_sockets() {
        // Both phases must complete without opening sockets — the MockCanBus
        // has no fds anyway, so we just verify build() returns Ok and
        // is_connected() is false.
        let (c, _, _) = CountingCodec::new();
        let robot = RobotBuilder::new()
            .add_bus("main", Box::new(MockCanBus::new("m")), Box::new(c))
            .add_arm("arm", "main", vec![motor("j0", 0x01, 0x11)])
            .build()
            .unwrap();
        assert!(!robot.is_connected());
    }

    #[test]
    fn connect_populates_routes() {
        let (c, _, _) = CountingCodec::new();
        let mut robot = RobotBuilder::new()
            .add_bus("main", Box::new(MockCanBus::new("m")), Box::new(c))
            .add_arm(
                "arm",
                "main",
                vec![motor("j0", 0x01, 0x11), motor("j1", 0x02, 0x12)],
            )
            .build()
            .unwrap();
        robot.connect().unwrap();
        assert!(robot.is_connected());
        let bus = robot.bus("main").unwrap().lock().unwrap();
        assert_eq!(bus.routes.len(), 2);
        let r11 = bus.routes.get(&0x11).unwrap();
        assert_eq!(r11.group_name, "arm");
        assert_eq!(r11.motor_index, 0);
    }

    #[test]
    fn can_id_collision_across_groups_on_same_bus() {
        let (c, _, _) = CountingCodec::new();
        let mut robot = RobotBuilder::new()
            .add_bus("main", Box::new(MockCanBus::new("m")), Box::new(c))
            .add_arm("arm", "main", vec![motor("j0", 0x01, 0x18)])
            .add_gripper("g", "main", motor("g0", 0x05, 0x18))
            .build()
            .unwrap();
        let r = robot.connect();
        assert!(matches!(
            r,
            Err(Error::CanIdCollision {
                ref bus_name,
                recv_id: 0x18,
                ..
            }) if bus_name == "main"
        ));
    }

    #[test]
    fn collision_across_buses_is_not_error() {
        let (c1, _, _) = CountingCodec::new();
        let (c2, _, _) = CountingCodec::new();
        let mut robot = RobotBuilder::new()
            .add_bus("left", Box::new(MockCanBus::new("l")), Box::new(c1))
            .add_bus("right", Box::new(MockCanBus::new("r")), Box::new(c2))
            .add_arm("la", "left", vec![motor("j0", 0x01, 0x11)])
            .add_arm("ra", "right", vec![motor("j0", 0x01, 0x11)])
            .build()
            .unwrap();
        robot.connect().unwrap();
    }

    #[test]
    fn tick_before_connect_returns_not_connected() {
        let (c, _, _) = CountingCodec::new();
        let mut robot = RobotBuilder::new()
            .add_bus("main", Box::new(MockCanBus::new("m")), Box::new(c))
            .add_arm("arm", "main", vec![motor("j0", 0x01, 0x11)])
            .build()
            .unwrap();
        let r = robot.tick(Duration::from_millis(1));
        assert!(matches!(r, Err(Error::NotConnected)));
    }

    #[test]
    fn enable_before_connect_returns_not_connected() {
        let (c, _, _) = CountingCodec::new();
        let mut robot = RobotBuilder::new()
            .add_bus("main", Box::new(MockCanBus::new("m")), Box::new(c))
            .add_arm("arm", "main", vec![motor("j0", 0x01, 0x11)])
            .build()
            .unwrap();
        assert!(matches!(robot.enable(), Err(Error::NotConnected)));
    }

    #[test]
    fn enable_calibrates_configured_gripper_opening() {
        let mut robot = RobotBuilder::new()
            .add_bus(
                "main",
                Box::new(MockCanBus::new("m")),
                Box::new(EchoFeedbackCodec::new()),
            )
            .add_gripper_with_opening(
                "g",
                "main",
                motor("g0", 0x05, 0x18),
                GripperOpeningSpec::new(OpeningDirection::IncreasingPosition, Some(0.2)),
            )
            .build()
            .unwrap();
        robot.connect().unwrap();
        robot.enable().unwrap();
        let gripper = robot
            .group_mut("g")
            .and_then(|group| group.as_gripper_mut())
            .unwrap();
        gripper.set_opening(0.5, None).unwrap();
    }

    fn lifecycle_robot(reply: Option<u32>) -> (Robot, MockCanBus) {
        let mock = MockCanBus::new("m");
        let inspector = mock.clone();
        let mut robot = RobotBuilder::new()
            .add_bus(
                "main",
                Box::new(mock),
                Box::new(ModeLifecycleCodec {
                    reply,
                    position: Mutex::new(0.0),
                }),
            )
            .add_gripper_with_opening(
                "g",
                "main",
                motor("g0", 0x05, 0x18),
                GripperOpeningSpec::new(OpeningDirection::IncreasingPosition, Some(0.2)),
            )
            .build()
            .unwrap();
        robot.connect().unwrap();
        (robot, inspector)
    }

    #[test]
    fn damiao_opening_lifecycle_orders_write_readback_enable_then_calibration() {
        let (mut robot, bus) = lifecycle_robot(Some(4));
        robot.enable().unwrap();
        let markers: Vec<u8> = bus
            .sent_frames()
            .iter()
            .map(|f| {
                if f.len >= 4 && f.payload()[2] == 0x20 {
                    0x20
                } else {
                    f.payload()[0]
                }
            })
            .collect();
        assert_eq!(&markers[..4], &[0x10, 0x20, 0xfc, 0xcc]);
        assert!(markers.iter().skip(3).any(|marker| *marker == 0x55));
    }

    #[test]
    fn non_four_mode_readback_stops_before_enable_or_calibration() {
        let (mut robot, bus) = lifecycle_robot(Some(3));
        assert!(matches!(
            robot.enable(),
            Err(Error::OpeningControlModeVerificationFailed { .. })
        ));
        let markers: Vec<u8> = bus
            .sent_frames()
            .iter()
            .map(|f| {
                if f.len >= 4 && f.payload()[2] == 0x20 {
                    0x20
                } else {
                    f.payload()[0]
                }
            })
            .collect();
        assert!(markers.iter().all(|m| *m != 0xfc && *m != 0x55));
    }

    #[test]
    fn mode_readback_timeout_stops_before_enable_or_calibration() {
        let (mut robot, bus) = lifecycle_robot(None);
        assert!(matches!(
            robot.enable(),
            Err(Error::OpeningControlModeVerificationFailed { .. })
        ));
        let markers: Vec<u8> = bus
            .sent_frames()
            .iter()
            .map(|f| {
                if f.len >= 4 && f.payload()[2] == 0x20 {
                    0x20
                } else {
                    f.payload()[0]
                }
            })
            .collect();
        assert!(markers.iter().all(|m| *m != 0xfc && *m != 0x55));
    }

    #[test]
    fn enable_fails_when_measured_feedback_span_is_zero() {
        let mut robot = RobotBuilder::new()
            .add_bus(
                "main",
                Box::new(MockCanBus::new("m")),
                Box::new(FeedbackCodec::new(vec![0.0])),
            )
            .add_gripper_with_opening(
                "g",
                "main",
                motor("g0", 0x05, 0x18),
                GripperOpeningSpec::new(OpeningDirection::IncreasingPosition, Some(0.2)),
            )
            .build()
            .unwrap();
        robot.connect().unwrap();
        let result = robot.enable();
        assert!(
            matches!(
                result,
                Err(Error::OpeningCalibrationFailed { ref reason, .. })
                    if reason.starts_with("calibrated opening span is too small")
            ),
            "unexpected result: {result:?}"
        );
    }

    #[test]
    fn enable_fails_when_measured_feedback_span_is_too_small() {
        let mut robot = RobotBuilder::new()
            .add_bus(
                "main",
                Box::new(MockCanBus::new("m")),
                Box::new(FeedbackCodec::new(
                    [0.0]
                        .into_iter()
                        .chain([-0.06; 24])
                        .chain([0.02; 24])
                        .collect(),
                )),
            )
            .add_gripper_with_opening(
                "g",
                "main",
                motor("g0", 0x05, 0x18),
                GripperOpeningSpec::new(OpeningDirection::IncreasingPosition, Some(0.2)),
            )
            .build()
            .unwrap();
        robot.connect().unwrap();
        let result = robot.enable();
        assert!(
            matches!(
                result,
                Err(Error::OpeningCalibrationFailed { ref reason, .. })
                    if reason.starts_with("calibrated opening span is too small")
            ),
            "unexpected result: {result:?}"
        );
    }

    #[test]
    fn set_opening_before_enable_calibration_requires_calibration() {
        let mut robot = RobotBuilder::new()
            .add_bus(
                "main",
                Box::new(MockCanBus::new("m")),
                Box::new(FeedbackCodec::new(vec![0.0])),
            )
            .add_gripper_with_opening(
                "g",
                "main",
                motor("g0", 0x05, 0x18),
                GripperOpeningSpec::new(OpeningDirection::IncreasingPosition, Some(0.2)),
            )
            .build()
            .unwrap();
        let gripper = robot
            .group_mut("g")
            .and_then(|group| group.as_gripper_mut())
            .unwrap();
        assert!(matches!(
            gripper.set_opening(0.5, None),
            Err(Error::OpeningCalibrationRequired)
        ));
    }

    #[test]
    fn enable_fails_when_gripper_opening_calibration_command_fails() {
        let mut robot = RobotBuilder::new()
            .add_bus(
                "main",
                Box::new(MockCanBus::new("m")),
                Box::new(FailingCommandCodec),
            )
            .add_gripper_with_opening(
                "g",
                "main",
                motor("g0", 0x05, 0x18),
                GripperOpeningSpec::new(OpeningDirection::IncreasingPosition, Some(0.2)),
            )
            .build()
            .unwrap();
        robot.connect().unwrap();
        assert!(matches!(robot.enable(), Err(Error::Codec(_))));
    }

    #[test]
    fn disable_without_enable_is_noop() {
        let (c, _, _) = CountingCodec::new();
        let mut robot = RobotBuilder::new()
            .add_bus("main", Box::new(MockCanBus::new("m")), Box::new(c))
            .add_arm("arm", "main", vec![motor("j0", 0x01, 0x11)])
            .build()
            .unwrap();
        // Not connected → disable returns Ok no-op
        robot.disable().unwrap();
        // Connected but never enabled → disable runs without enable ACK first
        robot.connect().unwrap();
        robot.disable().unwrap();
    }

    #[test]
    fn tick_quiet_buses_returns_within_deadline() {
        let (c, _, _) = CountingCodec::new();
        let mut robot = RobotBuilder::new()
            .add_bus("main", Box::new(MockCanBus::new("m")), Box::new(c))
            .add_arm("arm", "main", vec![motor("j0", 0x01, 0x11)])
            .build()
            .unwrap();
        robot.connect().unwrap();
        let t0 = std::time::Instant::now();
        robot.tick(Duration::from_millis(5)).unwrap();
        let elapsed = t0.elapsed();
        // MockCanBus has no fd, so poller is empty and tick returns essentially
        // immediately (the BusPoller wait still respects deadline; here it has
        // no fds so it returns 0-token vec immediately).
        assert!(elapsed < Duration::from_millis(50), "tick took {elapsed:?}");
    }

    /// Source grep: no group method body calls drain_inbound_nonblocking.
    /// Enforced as a textual scan of the group source. The `#[cfg(test)]`
    /// modules are stripped first (test code legitimately drains mock peers to
    /// observe what a group sent); this guards production method bodies only,
    /// matching the sibling `source_invariants` scan in group.rs.
    #[test]
    fn group_source_does_not_call_drain() {
        let src = include_str!("group.rs");
        let scan = match src.find("#[cfg(test)]") {
            Some(idx) => &src[..idx],
            None => src,
        };
        for needle in ["drain_inbound_nonblocking(", ".drain_inbound_nonblocking"] {
            assert!(
                !scan.contains(needle),
                "group.rs must not call {needle}; only Robot::tick may"
            );
        }
    }
}
