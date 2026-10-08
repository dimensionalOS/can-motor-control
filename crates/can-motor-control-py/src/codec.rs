//! Python wrappers for the Damiao codec (and the vendor's motor-type enum).

use std::sync::{Arc, Mutex};

use damiao_codec::{DamiaoCodec, DamiaoMotorType};
use motor_codec::{
    CanFrame, CodecError, Command, Event, Limits, MotorCodec, MotorRef, MotorTypeId,
};
use pyo3::prelude::*;

/// Same shape as `TransportHandle`: a Box-trait-object that the builder
/// `take()`s once.
#[derive(Clone)]
pub struct CodecHandle {
    pub(crate) inner: Arc<Mutex<Option<Box<dyn MotorCodec>>>>,
}

impl CodecHandle {
    pub fn new<C: MotorCodec + 'static>(c: C) -> Self {
        Self {
            inner: Arc::new(Mutex::new(Some(Box::new(c)))),
        }
    }
    pub fn take(&self) -> Option<Box<dyn MotorCodec>> {
        self.inner.lock().ok()?.take()
    }
}

/// The Damiao protocol codec.
///
/// Encodes commands to and decodes feedback from Damiao-family motors. Pass an
/// instance to `RobotBuilder.add_bus` to bind a bus to the Damiao
/// protocol. A codec is consumed by the builder and may be added to only one
/// bus.
#[pyclass(name = "DamiaoCodec", module = "can_motor_control.damiao")]
pub struct PyDamiaoCodec {
    pub(crate) handle: CodecHandle,
}

#[pymethods]
impl PyDamiaoCodec {
    /// Create a Damiao codec.
    #[new]
    fn new() -> Self {
        Self {
            handle: CodecHandle::new(DamiaoCodec::new()),
        }
    }
}

struct MockFeedbackCodec {
    recv_id: u32,
    position: Mutex<f64>,
}

impl MockFeedbackCodec {
    fn new(recv_id: u32) -> Self {
        Self {
            recv_id,
            position: Mutex::new(0.0),
        }
    }
}

impl MotorCodec for MockFeedbackCodec {
    fn vendor_name(&self) -> &'static str {
        "mock-feedback"
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

    fn bind_to_bus(&mut self, _: motor_codec::BusCapabilities) {}

    fn encode_enable(&self, m: MotorRef<'_>) -> Result<CanFrame, CodecError> {
        CanFrame::classical(m.send_id, &[0xFC])
            .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
    }

    fn encode_disable(&self, m: MotorRef<'_>) -> Result<CanFrame, CodecError> {
        CanFrame::classical(m.send_id, &[0xFD])
            .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
    }

    fn encode_set_zero(&self, m: MotorRef<'_>) -> Result<CanFrame, CodecError> {
        if let Ok(mut position) = self.position.lock() {
            *position = 0.0;
        }
        CanFrame::classical(m.recv_id, &[0xFE])
            .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
    }

    fn encode_command(&self, m: MotorRef<'_>, command: &Command) -> Result<CanFrame, CodecError> {
        if let Command::PosForce { q, .. } = *command {
            if let Ok(mut position) = self.position.lock() {
                *position = q;
            }
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
        if frame.id != self.recv_id {
            return Ok(None);
        }
        let q = *self.position.lock().map_err(|_| CodecError::DecodeFailed {
            reason: "mock lock",
        })?;
        Ok(Some(Event::State {
            motor_id: self.recv_id,
            q,
            dq: 0.0,
            tau: 0.0,
            t_mos: 30,
            t_rotor: 35,
        }))
    }
}

/// A tiny feedback codec for examples and tests that use `MockCanBus`.
///
/// It accepts Damiao motor type ids and echoes each `PosForce` command's raw
/// target position as a state frame from `recv_id`. This lets dry-run examples
/// exercise lifecycle code such as automatic opening calibration without real
/// hardware. Do not use it with `SocketCanBus`.
#[pyclass(name = "MockFeedbackCodec", module = "can_motor_control")]
pub struct PyMockFeedbackCodec {
    pub(crate) handle: CodecHandle,
}

#[pymethods]
impl PyMockFeedbackCodec {
    /// Create a mock feedback codec for a single motor receive id.
    #[new]
    fn new(recv_id: u32) -> Self {
        Self {
            handle: CodecHandle::new(MockFeedbackCodec::new(recv_id)),
        }
    }
}

/// Damiao motor SKU identifiers.
///
/// An ``IntEnum``-compatible class naming each supported Damiao motor model
/// (e.g. ``DM4310``, ``DM8009``). Pass a member as the ``type`` of a
/// `can_motor_control.MotorSpec` so the codec applies the right limits and
/// scaling for that model.
#[pyclass(
    name = "MotorType",
    module = "can_motor_control.damiao",
    eq,
    eq_int,
    frozen
)]
#[derive(Copy, Clone, PartialEq, Eq, Hash)]
#[allow(non_camel_case_types)]
pub enum PyMotorType {
    DM3507 = 0,
    DM4310 = 1,
    DM4310_48V = 2,
    DM4340 = 3,
    DM4340_48V = 4,
    DM6006 = 5,
    DM8006 = 6,
    DM8009 = 7,
    DM10010L = 8,
    DM10010 = 9,
    DMH3510 = 10,
    DMH6215 = 11,
    DMG6220 = 12,
}

impl From<PyMotorType> for MotorTypeId {
    fn from(t: PyMotorType) -> Self {
        let dm = match t {
            PyMotorType::DM3507 => DamiaoMotorType::DM3507,
            PyMotorType::DM4310 => DamiaoMotorType::DM4310,
            PyMotorType::DM4310_48V => DamiaoMotorType::DM4310_48V,
            PyMotorType::DM4340 => DamiaoMotorType::DM4340,
            PyMotorType::DM4340_48V => DamiaoMotorType::DM4340_48V,
            PyMotorType::DM6006 => DamiaoMotorType::DM6006,
            PyMotorType::DM8006 => DamiaoMotorType::DM8006,
            PyMotorType::DM8009 => DamiaoMotorType::DM8009,
            PyMotorType::DM10010L => DamiaoMotorType::DM10010L,
            PyMotorType::DM10010 => DamiaoMotorType::DM10010,
            PyMotorType::DMH3510 => DamiaoMotorType::DMH3510,
            PyMotorType::DMH6215 => DamiaoMotorType::DMH6215,
            PyMotorType::DMG6220 => DamiaoMotorType::DMG6220,
        };
        dm.into()
    }
}

/// RobStride classical CAN codec with one host address per bus.
#[pyclass(name = "RobstrideCodec", module = "can_motor_control.robstride")]
pub struct PyRobstrideCodec {
    pub(crate) handle: CodecHandle,
}
#[pymethods]
impl PyRobstrideCodec {
    /// Construct without opening CAN or sending commands.
    #[new]
    #[pyo3(signature = (host_id=0xFD))]
    fn new(host_id: u8) -> Self {
        Self {
            handle: CodecHandle::new(robstride_codec::RobstrideCodec::with_host_id(host_id)),
        }
    }
}
/// RobStride models supported by the pinned protocol profile.
#[pyclass(
    name = "RobstrideMotorType",
    module = "can_motor_control.robstride",
    eq,
    eq_int,
    frozen
)]
#[derive(Copy, Clone, PartialEq, Eq, Hash)]
pub enum PyRobstrideMotorType {
    RS00 = 0,
    RS06 = 6,
}
impl From<PyRobstrideMotorType> for MotorTypeId {
    fn from(value: PyRobstrideMotorType) -> Self {
        Self::Robostride(value as u16)
    }
}
