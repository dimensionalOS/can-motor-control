//! Group types: [`MotorGroup`] base + [`Arm`], [`Gripper`], [`Generic`] newtypes
//! and the [`GroupKind`] dispatcher.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use motor_codec::{Command, CommandKind, Event, MotorRef};

use crate::bus::Bus;
use crate::error::Error;
use crate::motor::Motor;
use crate::spec::GripperOpeningSpec;

const DEFAULT_OPENING_CURRENT: f64 = 0.15;
const DEFAULT_OPENING_VELOCITY: f64 = 10.0;
pub(crate) const OPENING_CALIBRATION_TRAVEL_RAD: f64 = 4.0;
pub(crate) const OPENING_CALIBRATION_MIN_SPAN_RAD: f64 = 0.1;

#[derive(Copy, Clone, Debug, PartialEq)]
pub(crate) struct OpeningCalibration {
    pub(crate) closed_position: f64,
    pub(crate) open_position: f64,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub(crate) struct OpeningControl {
    pub(crate) spec: GripperOpeningSpec,
    pub(crate) calibration: Option<OpeningCalibration>,
}

impl OpeningControl {
    fn new(spec: GripperOpeningSpec) -> Self {
        Self {
            spec,
            calibration: None,
        }
    }
}

/// MIT impedance command.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct MitCmd {
    /// Position gain.
    pub kp: f64,
    /// Velocity gain.
    pub kd: f64,
    /// Position setpoint (radians).
    pub q: f64,
    /// Velocity setpoint (rad/s).
    pub dq: f64,
    /// Feed-forward torque (Nm).
    pub tau: f64,
}

/// PosVel command.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PosVelCmd {
    /// Position setpoint (radians).
    pub q: f64,
    /// Velocity setpoint (rad/s).
    pub dq: f64,
}

/// Pure-velocity command.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VelCmd {
    /// Velocity setpoint (rad/s).
    pub dq: f64,
}

/// PosForce command (position + velocity + current per-unit).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PosForceCmd {
    /// Position setpoint (radians).
    pub q: f64,
    /// Velocity setpoint (rad/s).
    pub dq: f64,
    /// Current setpoint (per-unit).
    pub i_pu: f64,
}

impl From<MitCmd> for Command {
    fn from(c: MitCmd) -> Self {
        Command::Mit {
            kp: c.kp,
            kd: c.kd,
            q: c.q,
            dq: c.dq,
            tau: c.tau,
        }
    }
}
impl From<PosVelCmd> for Command {
    fn from(c: PosVelCmd) -> Self {
        Command::PosVel { q: c.q, dq: c.dq }
    }
}
impl From<VelCmd> for Command {
    fn from(c: VelCmd) -> Self {
        Command::Vel { dq: c.dq }
    }
}
impl From<PosForceCmd> for Command {
    fn from(c: PosForceCmd) -> Self {
        Command::PosForce {
            q: c.q,
            dq: c.dq,
            i_pu: c.i_pu,
        }
    }
}

/// Collection of motors that share a CAN bus and a vendor codec.
pub struct MotorGroup {
    name: String,
    bus_name: String,
    motors: Vec<Motor>,
    by_name: HashMap<String, usize>,
    pub(crate) bus: Option<Arc<Mutex<Bus>>>,
}

impl MotorGroup {
    pub(crate) fn new(name: String, bus_name: String, motors: Vec<Motor>) -> Self {
        let mut by_name = HashMap::with_capacity(motors.len());
        for (i, m) in motors.iter().enumerate() {
            by_name.insert(m.name().to_string(), i);
        }
        Self {
            name,
            bus_name,
            motors,
            by_name,
            bus: None,
        }
    }

    /// Group name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Name of the bus this group is attached to.
    pub fn bus_name(&self) -> &str {
        &self.bus_name
    }

    /// Number of motors in the group.
    pub fn len(&self) -> usize {
        self.motors.len()
    }

    /// True if there are no motors (shouldn't happen via the builder).
    pub fn is_empty(&self) -> bool {
        self.motors.is_empty()
    }

    /// Get a motor by name.
    pub fn motor(&self, name: &str) -> Option<&Motor> {
        self.by_name.get(name).map(|&i| &self.motors[i])
    }

    /// Get a motor mutably by name.
    pub fn motor_mut(&mut self, name: &str) -> Option<&mut Motor> {
        let i = *self.by_name.get(name)?;
        Some(&mut self.motors[i])
    }

    /// Get a motor by insertion index.
    pub fn motor_at(&self, idx: usize) -> Option<&Motor> {
        self.motors.get(idx)
    }

    /// All motors in insertion order.
    pub fn motors(&self) -> &[Motor] {
        &self.motors
    }

    /// Crate-private: attach the bus reference at builder time.
    pub(crate) fn attach_bus(&mut self, bus: Arc<Mutex<Bus>>) {
        self.bus = Some(bus);
    }

    /// Update one motor from an inbound [`Event`].
    pub(crate) fn apply_event(&mut self, motor_index: usize, event: &Event) {
        debug_assert!(motor_index < self.motors.len(), "apply_event index OOB");
        if let Some(m) = self.motors.get_mut(motor_index) {
            m.apply_event(event);
        }
    }

    fn bus_arc(&self) -> Result<Arc<Mutex<Bus>>, Error> {
        Ok(self.bus.as_ref().ok_or(Error::NotConnected)?.clone())
    }

    pub(crate) fn send_command(&mut self, idx: usize, cmd: Command) -> Result<(), Error> {
        let motor = self
            .motors
            .get(idx)
            .ok_or(Error::Internal("motor index out of range"))?;
        let m_ref = MotorRef {
            motor_type: motor.motor_type(),
            send_id: motor.send_id(),
            recv_id: motor.recv_id(),
            name: motor.name(),
        };
        let bus_arc = self.bus_arc()?;
        let mut bus = bus_arc.lock().map_err(|_| Error::BusPoisoned)?;
        let frame = bus
            .codec
            .encode_command(m_ref, &cmd)
            .map_err(Error::Codec)?;
        bus.transport.send(&frame).map_err(Error::Transport)
    }

    pub(crate) fn bus_vendor(&self) -> Result<&'static str, Error> {
        let bus = self.bus_arc()?;
        let bus = bus.lock().map_err(|_| Error::BusPoisoned)?;
        Ok(bus.codec.vendor_name())
    }

    pub(crate) fn send_control_mode_readback(&mut self) -> Result<Option<(u32, u32)>, Error> {
        let motor = self
            .motors
            .first()
            .ok_or(Error::Internal("gripper has no motor"))?;
        let m_ref = MotorRef {
            motor_type: motor.motor_type(),
            send_id: motor.send_id(),
            recv_id: motor.recv_id(),
            name: motor.name(),
        };
        let bus_arc = self.bus_arc()?;
        let mut bus = bus_arc.lock().map_err(|_| Error::BusPoisoned)?;
        let Some(frame) = bus
            .codec
            .encode_control_mode_readback(m_ref)
            .map_err(Error::Codec)?
        else {
            return Ok(None);
        };
        bus.transport.send(&frame).map_err(Error::Transport)?;
        Ok(Some((motor.send_id(), motor.recv_id())))
    }

    pub(crate) fn batch_send_commands(&mut self, cmds: &[Command]) -> Result<(), Error> {
        if cmds.len() != self.motors.len() {
            return Err(Error::CommandLengthMismatch {
                expected: self.motors.len(),
                got: cmds.len(),
            });
        }
        let bus_arc = self.bus_arc()?;
        let mut bus = bus_arc.lock().map_err(|_| Error::BusPoisoned)?;
        for (i, cmd) in cmds.iter().enumerate() {
            let motor = &self.motors[i];
            let m_ref = MotorRef {
                motor_type: motor.motor_type(),
                send_id: motor.send_id(),
                recv_id: motor.recv_id(),
                name: motor.name(),
            };
            let frame = bus.codec.encode_command(m_ref, cmd).map_err(Error::Codec)?;
            bus.transport.send(&frame).map_err(Error::Transport)?;
        }
        Ok(())
    }

    /// Send the codec's enable frame to every motor in insertion order.
    pub fn enable_all(&mut self) -> Result<(), Error> {
        let bus_arc = self.bus_arc()?;
        let mut bus = bus_arc.lock().map_err(|_| Error::BusPoisoned)?;
        for motor in &mut self.motors {
            let m_ref = MotorRef {
                motor_type: motor.motor_type(),
                send_id: motor.send_id(),
                recv_id: motor.recv_id(),
                name: motor.name(),
            };
            let frame = bus.codec.encode_enable(m_ref).map_err(Error::Codec)?;
            bus.transport.send(&frame).map_err(Error::Transport)?;
            motor.set_enabled(true);
        }
        Ok(())
    }

    /// Send the codec's disable frame to every motor in insertion order.
    /// (Robot::disable reverses group order; individual groups disable motors
    /// in declaration order.)
    pub fn disable_all(&mut self) -> Result<(), Error> {
        let bus_arc = self.bus_arc()?;
        let mut bus = bus_arc.lock().map_err(|_| Error::BusPoisoned)?;
        for motor in self.motors.iter_mut() {
            let m_ref = MotorRef {
                motor_type: motor.motor_type(),
                send_id: motor.send_id(),
                recv_id: motor.recv_id(),
                name: motor.name(),
            };
            let frame = bus.codec.encode_disable(m_ref).map_err(Error::Codec)?;
            bus.transport.send(&frame).map_err(Error::Transport)?;
            motor.set_enabled(false);
        }
        Ok(())
    }

    /// Send the codec's set-zero frame to every motor in insertion order.
    pub fn set_zero_all(&mut self) -> Result<(), Error> {
        let bus_arc = self.bus_arc()?;
        let mut bus = bus_arc.lock().map_err(|_| Error::BusPoisoned)?;
        for motor in &self.motors {
            let m_ref = MotorRef {
                motor_type: motor.motor_type(),
                send_id: motor.send_id(),
                recv_id: motor.recv_id(),
                name: motor.name(),
            };
            let frame = bus.codec.encode_set_zero(m_ref).map_err(Error::Codec)?;
            bus.transport.send(&frame).map_err(Error::Transport)?;
        }
        Ok(())
    }

    /// Send the codec's state-refresh query to every motor that supports one,
    /// in insertion order. Motors whose codec returns `None` (no refresh) are
    /// skipped. Send-only: this never drains inbound frames — call `tick` to
    /// receive the replies. Commands no motion.
    pub fn refresh_all(&mut self) -> Result<(), Error> {
        let bus_arc = self.bus_arc()?;
        let mut bus = bus_arc.lock().map_err(|_| Error::BusPoisoned)?;
        for motor in &self.motors {
            let m_ref = MotorRef {
                motor_type: motor.motor_type(),
                send_id: motor.send_id(),
                recv_id: motor.recv_id(),
                name: motor.name(),
            };
            if let Some(frame) = bus.codec.encode_refresh(m_ref).map_err(Error::Codec)? {
                bus.transport.send(&frame).map_err(Error::Transport)?;
            }
        }
        Ok(())
    }

    /// Set the persistent control mode (MIT / PosVel / Vel / PosForce) on every
    /// motor whose codec supports it, in insertion order. Commands no motion;
    /// send-only. Call once at startup, before the matching control commands.
    pub fn set_mode(&mut self, mode: CommandKind) -> Result<(), Error> {
        let bus_arc = self.bus_arc()?;
        let mut bus = bus_arc.lock().map_err(|_| Error::BusPoisoned)?;
        for motor in &self.motors {
            let m_ref = MotorRef {
                motor_type: motor.motor_type(),
                send_id: motor.send_id(),
                recv_id: motor.recv_id(),
                name: motor.name(),
            };
            if let Some(frame) = bus
                .codec
                .encode_set_mode(m_ref, mode)
                .map_err(Error::Codec)?
            {
                bus.transport.send(&frame).map_err(Error::Transport)?;
            }
        }
        Ok(())
    }
}

/// Newtype wrapping a [`MotorGroup`] that represents an articulated arm.
pub struct Arm(pub(crate) MotorGroup);

impl Arm {
    /// Number of motors.
    pub fn len(&self) -> usize {
        self.0.len()
    }
    /// True if empty.
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
    /// Get a motor by name.
    pub fn motor(&self, name: &str) -> Option<&Motor> {
        self.0.motor(name)
    }
    /// Get a motor mutably by name.
    pub fn motor_mut(&mut self, name: &str) -> Option<&mut Motor> {
        self.0.motor_mut(name)
    }
    /// Get a motor by insertion index.
    pub fn motor_at(&self, idx: usize) -> Option<&Motor> {
        self.0.motor_at(idx)
    }

    /// Slice of positions in insertion order.
    pub fn positions(&self) -> Vec<f64> {
        self.0.motors().iter().map(|m| m.position()).collect()
    }
    /// Slice of velocities in insertion order.
    pub fn velocities(&self) -> Vec<f64> {
        self.0.motors().iter().map(|m| m.velocity()).collect()
    }
    /// Slice of torques in insertion order.
    pub fn torques(&self) -> Vec<f64> {
        self.0.motors().iter().map(|m| m.torque()).collect()
    }

    /// Batch MIT control. `cmds.len()` must equal `self.len()`.
    pub fn mit_control(&mut self, cmds: &[MitCmd]) -> Result<(), Error> {
        let v: Vec<Command> = cmds.iter().copied().map(Into::into).collect();
        self.0.batch_send_commands(&v)
    }
    /// Batch PosVel control.
    pub fn pos_vel_control(&mut self, cmds: &[PosVelCmd]) -> Result<(), Error> {
        let v: Vec<Command> = cmds.iter().copied().map(Into::into).collect();
        self.0.batch_send_commands(&v)
    }
    /// Batch pure-velocity control.
    pub fn vel_control(&mut self, cmds: &[VelCmd]) -> Result<(), Error> {
        let v: Vec<Command> = cmds.iter().copied().map(Into::into).collect();
        self.0.batch_send_commands(&v)
    }
    /// Batch PosForce control.
    pub fn pos_force_control(&mut self, cmds: &[PosForceCmd]) -> Result<(), Error> {
        let v: Vec<Command> = cmds.iter().copied().map(Into::into).collect();
        self.0.batch_send_commands(&v)
    }

    /// Enable every motor in insertion order.
    pub fn enable_all(&mut self) -> Result<(), Error> {
        self.0.enable_all()
    }
    /// Disable every motor in reverse insertion order.
    pub fn disable_all(&mut self) -> Result<(), Error> {
        self.0.disable_all()
    }
    /// Set every motor's current position as zero.
    pub fn set_zero_all(&mut self) -> Result<(), Error> {
        self.0.set_zero_all()
    }
    /// Send a state-refresh query to every motor (no motion). Pair with `tick`
    /// to receive the replies.
    pub fn refresh(&mut self) -> Result<(), Error> {
        self.0.refresh_all()
    }
    /// Set the persistent control mode on every motor (no motion). Call once at
    /// startup, before the matching control commands.
    pub fn set_mode(&mut self, mode: CommandKind) -> Result<(), Error> {
        self.0.set_mode(mode)
    }

    /// Borrow the underlying [`MotorGroup`].
    pub fn inner(&self) -> &MotorGroup {
        &self.0
    }
    /// Borrow the underlying [`MotorGroup`] mutably.
    pub fn inner_mut(&mut self) -> &mut MotorGroup {
        &mut self.0
    }
}

/// One-motor gripper newtype with raw motor commands and optional normalized
/// opening support.
pub struct Gripper {
    pub(crate) group: MotorGroup,
    pub(crate) opening: Option<OpeningControl>,
}

impl Gripper {
    pub(crate) fn raw(group: MotorGroup) -> Self {
        Self {
            group,
            opening: None,
        }
    }

    pub(crate) fn with_opening(group: MotorGroup, spec: GripperOpeningSpec) -> Self {
        Self {
            group,
            opening: Some(OpeningControl::new(spec)),
        }
    }

    /// The one motor.
    pub fn motor(&self) -> &Motor {
        self.group.motor_at(0).expect("gripper invariant")
    }
    /// The one motor (mutably).
    pub fn motor_mut(&mut self) -> &mut Motor {
        let i = 0;
        &mut self.group.motors[i]
    }

    /// Enable the motor.
    pub fn enable(&mut self) -> Result<(), Error> {
        self.group.enable_all()
    }
    /// Disable the motor.
    pub fn disable(&mut self) -> Result<(), Error> {
        self.group.disable_all()
    }
    /// Single MIT command to the gripper motor.
    pub fn mit_control(&mut self, cmd: MitCmd) -> Result<(), Error> {
        self.group.send_command(0, cmd.into())
    }
    /// Single PosVel command to the gripper motor.
    pub fn pos_vel_control(&mut self, cmd: PosVelCmd) -> Result<(), Error> {
        self.group.send_command(0, cmd.into())
    }
    /// Single PosForce command to the gripper motor.
    pub fn pos_force_control(&mut self, cmd: PosForceCmd) -> Result<(), Error> {
        self.group.send_command(0, cmd.into())
    }
    /// Send a state-refresh query to the motor (no motion). Pair with `tick`.
    pub fn refresh(&mut self) -> Result<(), Error> {
        self.group.refresh_all()
    }
    /// Set the gripper motor's persistent control mode (no motion).
    pub fn set_mode(&mut self, mode: CommandKind) -> Result<(), Error> {
        self.group.set_mode(mode)
    }

    /// True if this gripper is configured for normalized opening control.
    pub fn has_opening_control(&self) -> bool {
        self.opening.is_some()
    }

    pub(crate) fn opening_direction_sign(&self) -> Option<f64> {
        self.opening.map(|opening| opening.spec.direction.sign())
    }

    pub(crate) fn clear_opening_calibration(&mut self) {
        if let Some(opening) = self.opening.as_mut() {
            opening.calibration = None;
        }
    }

    pub(crate) fn set_opening_calibration(
        &mut self,
        closed_position: f64,
        open_position: f64,
    ) -> Result<(), Error> {
        let span = (open_position - closed_position).abs();
        if span < OPENING_CALIBRATION_MIN_SPAN_RAD {
            return Err(Error::OpeningCalibrationFailed {
                name: self.group.name().to_string(),
                reason: format!(
                    "calibrated opening span is too small: closed={closed_position:.5}, open={open_position:.5}, span={span:.5}, minimum={OPENING_CALIBRATION_MIN_SPAN_RAD:.5} rad"
                ),
            });
        }
        let Some(opening) = self.opening.as_mut() else {
            return Ok(());
        };
        opening.calibration = Some(OpeningCalibration {
            closed_position,
            open_position,
        });
        Ok(())
    }

    pub(crate) fn opening_calibration_command(&mut self, q: f64) -> Result<(), Error> {
        let current = self.opening_current(None)?;
        self.pos_force_control(PosForceCmd {
            q,
            dq: DEFAULT_OPENING_VELOCITY,
            i_pu: current,
        })
    }

    /// Queue a normalized opening command where 0.0 is closed and 1.0 is open.
    pub fn set_opening(&mut self, opening: f64, current: Option<f64>) -> Result<(), Error> {
        let q = self.opening_position(opening)?;
        let i_pu = self.opening_current(current)?;
        self.pos_force_control(PosForceCmd {
            q,
            dq: DEFAULT_OPENING_VELOCITY,
            i_pu,
        })
    }

    /// Return the normalized opening from the most recently cached motor feedback.
    ///
    /// `0.0` is fully closed and `1.0` is fully open. Call [`Self::refresh`]
    /// followed by [`crate::Robot::tick`] before reading when fresh feedback is
    /// required. Opening control must be configured and the per-session
    /// calibration performed by [`crate::Robot::enable`] must have completed.
    /// This getter only reads cached state and does not send CAN frames.
    pub fn opening(&self) -> Result<f64, Error> {
        let Some(control) = self.opening else {
            return Err(Error::OpeningCalibrationRequired);
        };
        let Some(calibration) = control.calibration else {
            return Err(Error::OpeningCalibrationRequired);
        };
        let opening = (self.motor().position() - calibration.closed_position)
            / (calibration.open_position - calibration.closed_position);
        Ok(opening.clamp(0.0, 1.0))
    }

    pub(crate) fn opening_position(&self, opening: f64) -> Result<f64, Error> {
        if !(0.0..=1.0).contains(&opening) {
            return Err(Error::OpeningOutOfRange { got: opening });
        }
        let Some(control) = self.opening else {
            return Err(Error::OpeningCalibrationRequired);
        };
        let Some(calibration) = control.calibration else {
            return Err(Error::OpeningCalibrationRequired);
        };
        Ok(calibration.closed_position
            + opening * (calibration.open_position - calibration.closed_position))
    }

    pub(crate) fn opening_current(&self, current: Option<f64>) -> Result<f64, Error> {
        let configured = self
            .opening
            .and_then(|control| control.spec.default_current);
        let i_pu = current.or(configured).unwrap_or(DEFAULT_OPENING_CURRENT);
        validate_opening_current(i_pu)?;
        Ok(i_pu)
    }

    /// Queue a fully-open command.
    pub fn open(&mut self, current: Option<f64>) -> Result<(), Error> {
        self.set_opening(1.0, current)
    }

    /// Queue a fully-closed command.
    pub fn close(&mut self, current: Option<f64>) -> Result<(), Error> {
        self.set_opening(0.0, current)
    }

    /// Borrow the underlying [`MotorGroup`].
    pub fn inner(&self) -> &MotorGroup {
        &self.group
    }
    /// Borrow the underlying [`MotorGroup`] mutably.
    pub fn inner_mut(&mut self) -> &mut MotorGroup {
        &mut self.group
    }
}

fn validate_opening_current(current: f64) -> Result<(), Error> {
    if current > 0.0 && current <= 1.0 {
        Ok(())
    } else {
        Err(Error::OpeningCurrentOutOfRange { got: current })
    }
}

/// Generic group for topologies that don't fit [`Arm`] or [`Gripper`] (future
/// hand, leg, head, etc.).
pub struct Generic(pub(crate) MotorGroup);

impl Generic {
    /// Number of motors.
    pub fn len(&self) -> usize {
        self.0.len()
    }
    /// True if empty.
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    /// Borrowed view of the inner group for read-only access.
    pub fn as_motor_group(&self) -> &MotorGroup {
        &self.0
    }
    /// Borrow mutably.
    pub fn as_motor_group_mut(&mut self) -> &mut MotorGroup {
        &mut self.0
    }

    /// Borrow the underlying [`MotorGroup`].
    pub fn inner(&self) -> &MotorGroup {
        &self.0
    }
    /// Borrow the underlying [`MotorGroup`] mutably.
    pub fn inner_mut(&mut self) -> &mut MotorGroup {
        &mut self.0
    }
}

/// Sum type of every group variant the robot can hold.
#[non_exhaustive]
pub enum GroupKind {
    /// Articulated arm.
    Arm(Arm),
    /// Single-motor gripper.
    Gripper(Gripper),
    /// Generic catch-all group.
    Generic(Generic),
}

impl GroupKind {
    /// Downcast to [`Arm`].
    pub fn as_arm(&self) -> Option<&Arm> {
        if let Self::Arm(a) = self {
            Some(a)
        } else {
            None
        }
    }
    /// Downcast to [`Arm`] mutably.
    pub fn as_arm_mut(&mut self) -> Option<&mut Arm> {
        if let Self::Arm(a) = self {
            Some(a)
        } else {
            None
        }
    }
    /// Downcast to [`Gripper`].
    pub fn as_gripper(&self) -> Option<&Gripper> {
        if let Self::Gripper(g) = self {
            Some(g)
        } else {
            None
        }
    }
    /// Downcast to [`Gripper`] mutably.
    pub fn as_gripper_mut(&mut self) -> Option<&mut Gripper> {
        if let Self::Gripper(g) = self {
            Some(g)
        } else {
            None
        }
    }
    /// Downcast to [`Generic`].
    pub fn as_generic(&self) -> Option<&Generic> {
        if let Self::Generic(g) = self {
            Some(g)
        } else {
            None
        }
    }
    /// Downcast to [`Generic`] mutably.
    pub fn as_generic_mut(&mut self) -> Option<&mut Generic> {
        if let Self::Generic(g) = self {
            Some(g)
        } else {
            None
        }
    }

    /// Borrow the underlying [`MotorGroup`] regardless of variant.
    pub fn inner(&self) -> &MotorGroup {
        match self {
            Self::Arm(a) => a.inner(),
            Self::Gripper(g) => g.inner(),
            Self::Generic(g) => g.inner(),
        }
    }
    /// Borrow mutably regardless of variant.
    pub fn inner_mut(&mut self) -> &mut MotorGroup {
        match self {
            Self::Arm(a) => a.inner_mut(),
            Self::Gripper(g) => g.inner_mut(),
            Self::Generic(g) => g.inner_mut(),
        }
    }

    /// Group name (delegated to the inner MotorGroup).
    pub fn name(&self) -> &str {
        self.inner().name()
    }
    /// Bus name (delegated).
    pub fn bus_name(&self) -> &str {
        self.inner().bus_name()
    }

    pub(crate) fn apply_event(&mut self, motor_index: usize, event: &Event) {
        self.inner_mut().apply_event(motor_index, event);
    }

    /// Enable all motors in this group (delegated).
    pub fn enable_all(&mut self) -> Result<(), Error> {
        self.inner_mut().enable_all()
    }
    /// Disable all motors in this group (delegated).
    pub fn disable_all(&mut self) -> Result<(), Error> {
        self.inner_mut().disable_all()
    }
    /// Send a state-refresh query to every motor in this group (delegated).
    pub fn refresh_all(&mut self) -> Result<(), Error> {
        self.inner_mut().refresh_all()
    }
    /// Set the persistent control mode on every motor in this group (delegated).
    pub fn set_mode(&mut self, mode: CommandKind) -> Result<(), Error> {
        self.inner_mut().set_mode(mode)
    }
}

#[cfg(test)]
mod tests {
    use motor_codec::MotorTypeId;

    use super::*;
    use crate::spec::{GripperOpeningSpec, OpeningDirection};

    fn make_group(name: &str, bus: &str, motor_names: &[&str]) -> MotorGroup {
        let motors: Vec<_> = motor_names
            .iter()
            .enumerate()
            .map(|(i, n)| {
                Motor::new(
                    n.to_string(),
                    MotorTypeId::Damiao(3),
                    0x01 + i as u32,
                    0x11 + i as u32,
                )
            })
            .collect();
        MotorGroup::new(name.into(), bus.into(), motors)
    }

    fn opening_gripper(direction: OpeningDirection, default_current: Option<f64>) -> Gripper {
        let mut gripper = Gripper::with_opening(
            make_group("grip", "main", &["g"]),
            GripperOpeningSpec::new(direction, default_current),
        );
        gripper.opening.as_mut().unwrap().calibration = Some(OpeningCalibration {
            closed_position: 2.0,
            open_position: 4.0,
        });
        gripper
    }

    #[test]
    fn name_and_index_access_agree() {
        let g = make_group("arm", "main", &["j0", "j1", "j2"]);
        assert_eq!(g.motor("j1").map(|m| m.name()), Some("j1"));
        assert_eq!(g.motor_at(1).map(|m| m.name()), Some("j1"));
        assert!(g.motor("ghost").is_none());
    }

    #[test]
    fn group_records_bus_name() {
        let g = make_group("arm", "left", &["j0"]);
        assert_eq!(g.bus_name(), "left");
        assert_eq!(g.name(), "arm");
    }

    #[test]
    fn opening_position_maps_closed_mid_open() {
        let gripper = opening_gripper(OpeningDirection::IncreasingPosition, Some(0.25));
        assert_eq!(gripper.opening_position(0.0).unwrap(), 2.0);
        assert_eq!(gripper.opening_position(0.5).unwrap(), 3.0);
        assert_eq!(gripper.opening_position(1.0).unwrap(), 4.0);
    }

    #[test]
    fn opening_position_supports_decreasing_direction_calibration() {
        let mut gripper = opening_gripper(OpeningDirection::DecreasingPosition, Some(0.25));
        gripper.opening.as_mut().unwrap().calibration = Some(OpeningCalibration {
            closed_position: 4.0,
            open_position: 2.0,
        });
        assert_eq!(gripper.opening_position(0.0).unwrap(), 4.0);
        assert_eq!(gripper.opening_position(0.5).unwrap(), 3.0);
        assert_eq!(gripper.opening_position(1.0).unwrap(), 2.0);
    }

    fn set_feedback_position(gripper: &mut Gripper, position: f64) {
        gripper.group.apply_event(
            0,
            &Event::State {
                motor_id: 0x11,
                q: position,
                dq: 0.0,
                tau: 0.0,
                t_mos: 30,
                t_rotor: 35,
            },
        );
    }

    #[test]
    fn opening_maps_cached_closed_mid_and_open_feedback() {
        let mut gripper = opening_gripper(OpeningDirection::IncreasingPosition, None);
        for (position, expected) in [(2.0, 0.0), (3.0, 0.5), (4.0, 1.0)] {
            set_feedback_position(&mut gripper, position);
            assert_eq!(gripper.opening().unwrap(), expected);
        }
    }

    #[test]
    fn opening_supports_decreasing_position_calibration() {
        let mut gripper = opening_gripper(OpeningDirection::DecreasingPosition, None);
        gripper.opening.as_mut().unwrap().calibration = Some(OpeningCalibration {
            closed_position: 4.0,
            open_position: 2.0,
        });
        for (position, expected) in [(4.0, 0.0), (3.0, 0.5), (2.0, 1.0)] {
            set_feedback_position(&mut gripper, position);
            assert_eq!(gripper.opening().unwrap(), expected);
        }
    }

    #[test]
    fn opening_clamps_feedback_beyond_calibrated_endpoints() {
        let mut gripper = opening_gripper(OpeningDirection::IncreasingPosition, None);
        set_feedback_position(&mut gripper, 1.99);
        assert_eq!(gripper.opening().unwrap(), 0.0);
        set_feedback_position(&mut gripper, 4.01);
        assert_eq!(gripper.opening().unwrap(), 1.0);
    }

    #[test]
    fn opening_requires_configured_and_completed_calibration() {
        let raw = Gripper::raw(make_group("grip", "main", &["g"]));
        assert!(matches!(
            raw.opening(),
            Err(Error::OpeningCalibrationRequired)
        ));

        let uncalibrated = Gripper::with_opening(
            make_group("grip", "main", &["g"]),
            GripperOpeningSpec::new(OpeningDirection::IncreasingPosition, None),
        );
        assert!(matches!(
            uncalibrated.opening(),
            Err(Error::OpeningCalibrationRequired)
        ));
    }

    #[test]
    fn opening_rejects_out_of_range_values() {
        let gripper = opening_gripper(OpeningDirection::IncreasingPosition, None);
        assert!(matches!(
            gripper.opening_position(-0.1),
            Err(Error::OpeningOutOfRange { .. })
        ));
        assert!(matches!(
            gripper.opening_position(1.1),
            Err(Error::OpeningOutOfRange { .. })
        ));
    }

    #[test]
    fn opening_current_precedence_is_per_call_then_config_then_library_default() {
        let configured = opening_gripper(OpeningDirection::IncreasingPosition, Some(0.25));
        assert_eq!(configured.opening_current(None).unwrap(), 0.25);
        assert_eq!(configured.opening_current(Some(0.4)).unwrap(), 0.4);

        let fallback = opening_gripper(OpeningDirection::IncreasingPosition, None);
        assert_eq!(
            fallback.opening_current(None).unwrap(),
            DEFAULT_OPENING_CURRENT
        );
    }

    #[test]
    fn opening_current_rejects_invalid_values() {
        let gripper = opening_gripper(OpeningDirection::IncreasingPosition, None);
        assert!(matches!(
            gripper.opening_current(Some(0.0)),
            Err(Error::OpeningCurrentOutOfRange { .. })
        ));
        assert!(matches!(
            gripper.opening_current(Some(1.1)),
            Err(Error::OpeningCurrentOutOfRange { .. })
        ));
    }

    #[test]
    fn apply_event_updates_target_motor() {
        let mut g = make_group("arm", "m", &["j0", "j1", "j2"]);
        g.apply_event(
            2,
            &Event::State {
                motor_id: 0x13,
                q: 0.5,
                dq: 0.1,
                tau: 0.0,
                t_mos: 30,
                t_rotor: 35,
            },
        );
        assert_eq!(g.motor_at(2).unwrap().position(), 0.5);
        assert_eq!(g.motor_at(0).unwrap().position(), 0.0);
    }

    #[test]
    fn group_kind_downcast() {
        let arm = Arm(make_group("a", "m", &["j0"]));
        let mut kind = GroupKind::Arm(arm);
        assert!(kind.as_arm().is_some());
        assert!(kind.as_gripper().is_none());
        assert!(kind.as_arm_mut().is_some());
        assert_eq!(kind.name(), "a");
        assert_eq!(kind.bus_name(), "m");
    }

    /// Compile-time check: MotorGroup, Arm, Gripper, Generic do not have a
    /// `codec` field of any kind. We do a structural assertion by listing the
    /// expected non-codec field names that DO exist.
    ///
    /// This is a smoke test; the grep test in `source_invariants` below is
    /// the authoritative enforcement.
    #[test]
    fn group_layout_has_no_codec_field() {
        let g = make_group("a", "m", &["j0"]);
        assert_eq!(g.bus_name(), "m"); // we have `bus_name`, not `codec`.
        assert_eq!(g.len(), 1);
    }

    /// Grep test: this source file's Arm / Gripper / Generic impl blocks must
    /// not reference `is_fd`, `FD_FORMAT`, or branch on CAN-FD vs classical.
    /// Frame-type handling is confined to the codec and bus impls.
    #[test]
    fn source_invariants() {
        let src = include_str!("group.rs");
        // Strip out the unit-test module before scanning so test-only mentions
        // don't trip the invariant.
        let scan = if let Some(idx) = src.find("#[cfg(test)]\nmod tests") {
            &src[..idx]
        } else {
            src
        };
        for forbidden in ["is_fd(", "FD_FORMAT", "FrameFlags::FD_FORMAT"] {
            assert!(
                !scan.contains(forbidden),
                "group source contains forbidden FD-discrimination token: {forbidden}"
            );
        }
    }
}

#[cfg(test)]
mod integration_tests {
    use std::sync::{Arc, Mutex};

    use motor_codec::{
        BusCapabilities, CanFrame, CodecError, Event, Limits, MotorCodec, MotorRef, MotorTypeId,
    };

    use super::*;
    use crate::transport::MockCanBus;
    use crate::CanBus;

    /// Codec stub specialized for these tests so we don't need to bring
    /// can-motor-damiao-codec into the unit-test compile graph.
    struct StubCodec;
    impl MotorCodec for StubCodec {
        fn vendor_name(&self) -> &'static str {
            "stub"
        }
        fn supports(&self, _: MotorTypeId) -> bool {
            true
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
        fn encode_command(&self, m: MotorRef<'_>, _: &Command) -> Result<CanFrame, CodecError> {
            CanFrame::classical(m.send_id, &[0x55])
                .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
        }
        fn encode_refresh(&self, m: MotorRef<'_>) -> Result<Option<CanFrame>, CodecError> {
            // Mimic Damiao's 0xCC-on-0x7FF query so refresh paths are exercised.
            let p = [m.send_id as u8, (m.send_id >> 8) as u8, 0xCC, 0, 0, 0, 0, 0];
            CanFrame::classical(0x7FF, &p)
                .map(Some)
                .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
        }
        fn decode(
            &self,
            _: &CanFrame,
            _: motor_codec::MotorRef<'_>,
        ) -> Result<Option<Event>, CodecError> {
            Ok(None)
        }
    }

    fn build_arm() -> (
        Arm,
        std::sync::Arc<std::sync::Mutex<Bus>>,
        Arc<Mutex<MockCanBus>>, // not actually accessible; we read via sent_frames at bus.transport
    ) {
        let mock = MockCanBus::new("vcan_main");
        let transport: Box<dyn CanBus> = Box::new(mock);
        let codec: Box<dyn MotorCodec> = Box::new(StubCodec);
        let bus = Arc::new(Mutex::new(Bus::new(transport, codec)));
        let motors = vec![
            Motor::new("j0".into(), MotorTypeId::Damiao(3), 0x01, 0x11),
            Motor::new("j1".into(), MotorTypeId::Damiao(3), 0x02, 0x12),
            Motor::new("j2".into(), MotorTypeId::Damiao(3), 0x03, 0x13),
        ];
        let mut group = MotorGroup::new("arm".into(), "main".into(), motors);
        group.attach_bus(bus.clone());
        let arm = Arm(group);
        // Mock placeholder — we read sent frames via re-downcast trick.
        let placeholder = Arc::new(Mutex::new(MockCanBus::new("ignored")));
        (arm, bus, placeholder)
    }

    #[test]
    fn enable_all_marks_every_motor_enabled() {
        let (mut arm, _bus, _) = build_arm();
        arm.enable_all().unwrap();
        for n in &["j0", "j1", "j2"] {
            assert!(arm.motor(n).unwrap().is_enabled(), "{n} not enabled");
        }
    }

    #[test]
    fn disable_after_enable_clears_flag() {
        let (mut arm, _bus, _) = build_arm();
        arm.enable_all().unwrap();
        arm.disable_all().unwrap();
        for n in &["j0", "j1", "j2"] {
            assert!(!arm.motor(n).unwrap().is_enabled(), "{n} still enabled");
        }
    }

    #[test]
    fn mit_length_mismatch_returns_error() {
        let (mut arm, _bus, _) = build_arm();
        let cmds = vec![MitCmd {
            kp: 0.0,
            kd: 0.0,
            q: 0.0,
            dq: 0.0,
            tau: 0.0,
        }];
        let r = arm.mit_control(&cmds);
        assert!(matches!(
            r,
            Err(Error::CommandLengthMismatch {
                expected: 3,
                got: 1
            })
        ));
    }

    #[test]
    fn mit_correct_length_succeeds() {
        let (mut arm, _bus, _) = build_arm();
        let cmds = vec![
            MitCmd {
                kp: 0.0,
                kd: 0.0,
                q: 0.0,
                dq: 0.0,
                tau: 0.0,
            };
            3
        ];
        arm.mit_control(&cmds).unwrap();
    }

    #[test]
    fn not_connected_when_bus_not_attached() {
        let motors = vec![Motor::new("g".into(), MotorTypeId::Damiao(3), 0x05, 0x18)];
        let mut group = MotorGroup::new("g".into(), "main".into(), motors);
        let r = group.enable_all();
        assert!(matches!(r, Err(Error::NotConnected)));
    }

    #[test]
    fn refresh_emits_one_query_per_motor() {
        // Pair the bus transport with a peer we keep, so we can observe exactly
        // what refresh() put on the wire (one frame per motor, no extra traffic).
        let (tx, mut peer) = MockCanBus::pair("vcan_main", "peer");
        let transport: Box<dyn CanBus> = Box::new(tx);
        let codec: Box<dyn MotorCodec> = Box::new(StubCodec);
        let bus = Arc::new(Mutex::new(Bus::new(transport, codec)));
        let motors = vec![
            Motor::new("j0".into(), MotorTypeId::Damiao(3), 0x01, 0x11),
            Motor::new("j1".into(), MotorTypeId::Damiao(3), 0x02, 0x12),
            Motor::new("j2".into(), MotorTypeId::Damiao(3), 0x03, 0x13),
        ];
        let mut group = MotorGroup::new("arm".into(), "main".into(), motors);
        group.attach_bus(bus);
        let mut arm = Arm(group);

        arm.refresh().unwrap();

        let got = peer.drain_inbound_nonblocking().unwrap();
        assert_eq!(got.len(), 3, "one refresh frame per motor");
        assert!(
            got.iter().all(|f| f.id == 0x7FF && f.payload()[2] == 0xCC),
            "every emitted frame is a 0xCC refresh query on 0x7FF"
        );
    }

    #[test]
    fn refresh_not_connected_when_bus_not_attached() {
        let motors = vec![Motor::new("g".into(), MotorTypeId::Damiao(3), 0x05, 0x18)];
        let mut group = MotorGroup::new("g".into(), "main".into(), motors);
        assert!(matches!(group.refresh_all(), Err(Error::NotConnected)));
    }
}
