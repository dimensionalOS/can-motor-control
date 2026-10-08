#![no_std]
#![forbid(unsafe_code)]
//! RobStride RS00 and RS06 extended classical CAN protocol.
use motor_codec::{
    BusCapabilities, CanFrame, CodecError, Command, CommandKind, Event, FrameFlags, Limits,
    MotorCodec, MotorRef, MotorTypeId, ParamValue,
};

/// Registry vendor name.
pub const VENDOR_NAME: &str = "robstride";
/// Supported motor models using the November 2025 protocol ranges.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u16)]
pub enum RobstrideMotorType {
    RS00 = 0,
    RS06 = 6,
}
impl From<RobstrideMotorType> for MotorTypeId {
    fn from(value: RobstrideMotorType) -> Self {
        Self::Robostride(value as u16)
    }
}
/// Resolve canonical model names and Seeed configuration names.
pub fn parse_motor_type(value: &str) -> Option<MotorTypeId> {
    match value {
        "RS00" | "rs-00" => Some(RobstrideMotorType::RS00.into()),
        "RS06" | "rs-06" => Some(RobstrideMotorType::RS06.into()),
        _ => None,
    }
}
fn model(value: MotorTypeId) -> Result<RobstrideMotorType, CodecError> {
    match value {
        MotorTypeId::Robostride(0) => Ok(RobstrideMotorType::RS00),
        MotorTypeId::Robostride(6) => Ok(RobstrideMotorType::RS06),
        _ => Err(CodecError::UnknownMotorType {
            vendor: VENDOR_NAME,
            type_id: match value {
                MotorTypeId::Robostride(id) | MotorTypeId::Damiao(id) => id,
                _ => u16::MAX,
            },
        }),
    }
}
fn ranges(value: RobstrideMotorType) -> (Limits, f64, f64) {
    match value {
        RobstrideMotorType::RS00 => (
            Limits {
                p_max: 12.57,
                v_max: 33.0,
                t_max: 14.0,
            },
            500.0,
            5.0,
        ),
        RobstrideMotorType::RS06 => (
            Limits {
                p_max: 12.57,
                v_max: 50.0,
                t_max: 36.0,
            },
            5000.0,
            100.0,
        ),
    }
}
fn pack(value: f64, low: f64, high: f64, field: &'static str) -> Result<u16, CodecError> {
    if !value.is_finite() || value < low || value > high {
        return Err(CodecError::OutOfRange { field });
    }
    Ok(((value - low) * 65535.0 / (high - low)) as u16)
}
fn unpack(raw: u16, maximum: f64) -> f64 {
    (raw as f64 * 2.0 / 65535.0 - 1.0) * maximum
}
/// Stateful model table for a bus whose motors share one host address.
pub struct RobstrideCodec {
    host_id: u8,
    models: [Option<RobstrideMotorType>; 256],
}
impl Default for RobstrideCodec {
    fn default() -> Self {
        Self::new()
    }
}
impl RobstrideCodec {
    /// Create a codec with host address 0xFD. This sends nothing.
    pub fn new() -> Self {
        Self::with_host_id(0xFD)
    }
    /// Create a codec for a different host address. This sends nothing.
    pub fn with_host_id(host_id: u8) -> Self {
        Self {
            host_id,
            models: [None; 256],
        }
    }
    fn identity(&self, motor: MotorRef<'_>) -> Result<RobstrideMotorType, CodecError> {
        let value = model(motor.motor_type)?;
        if !(1..=255).contains(&motor.send_id) {
            return Err(CodecError::OutOfRange { field: "send_id" });
        }
        if motor.recv_id != motor.send_id {
            return Err(CodecError::OutOfRange { field: "recv_id" });
        }
        Ok(value)
    }
    fn frame(
        &self,
        motor: MotorRef<'_>,
        kind: u32,
        data: u16,
        payload: &[u8; 8],
    ) -> Result<CanFrame, CodecError> {
        self.identity(motor)?;
        CanFrame::classical_extended((kind << 24) | ((data as u32) << 8) | motor.send_id, payload)
            .map_err(|_| CodecError::DecodeFailed {
                reason: "invalid RobStride frame",
            })
    }
    /// Encode a typed parameter read. It does not enable or move the motor.
    pub fn encode_read_parameter(
        &self,
        motor: MotorRef<'_>,
        register: u16,
    ) -> Result<CanFrame, CodecError> {
        let mut data = [0; 8];
        data[..2].copy_from_slice(&register.to_le_bytes());
        self.frame(motor, 17, self.host_id as u16, &data)
    }
    /// Encode a finite f32 register value. Parameter writes require caller authorization.
    pub fn encode_write_f32(
        &self,
        motor: MotorRef<'_>,
        register: u16,
        value: f32,
    ) -> Result<CanFrame, CodecError> {
        if !value.is_finite() {
            return Err(CodecError::OutOfRange { field: "parameter" });
        }
        self.write_bytes(motor, register, value.to_le_bytes())
    }
    /// Encode an unsigned register value such as the CAN timeout at 0x7028.
    pub fn encode_write_u32(
        &self,
        motor: MotorRef<'_>,
        register: u16,
        value: u32,
    ) -> Result<CanFrame, CodecError> {
        self.write_bytes(motor, register, value.to_le_bytes())
    }
    fn write_bytes(
        &self,
        motor: MotorRef<'_>,
        register: u16,
        value: [u8; 4],
    ) -> Result<CanFrame, CodecError> {
        let mut data = [0; 8];
        data[..2].copy_from_slice(&register.to_le_bytes());
        data[4..].copy_from_slice(&value);
        self.frame(motor, 18, self.host_id as u16, &data)
    }
}
impl MotorCodec for RobstrideCodec {
    fn vendor_name(&self) -> &'static str {
        VENDOR_NAME
    }
    fn supports(&self, value: MotorTypeId) -> bool {
        model(value).is_ok()
    }
    fn limits(&self, value: MotorTypeId) -> Result<Limits, CodecError> {
        Ok(ranges(model(value)?).0)
    }
    fn bind_to_bus(&mut self, _: BusCapabilities) {}
    fn bind_motor(&mut self, motor: MotorRef<'_>) -> Result<(), CodecError> {
        let value = self.identity(motor)?;
        let slot = &mut self.models[motor.send_id as usize];
        if slot.is_some_and(|old| old != value) {
            return Err(CodecError::OutOfRange {
                field: "motor_type",
            });
        }
        *slot = Some(value);
        Ok(())
    }
    fn encode_enable(&self, motor: MotorRef<'_>) -> Result<CanFrame, CodecError> {
        self.frame(motor, 3, self.host_id as u16, &[0; 8])
    }
    fn encode_disable(&self, motor: MotorRef<'_>) -> Result<CanFrame, CodecError> {
        self.frame(motor, 4, self.host_id as u16, &[0; 8])
    }
    fn encode_set_zero(&self, motor: MotorRef<'_>) -> Result<CanFrame, CodecError> {
        self.frame(motor, 6, self.host_id as u16, &[1, 0, 0, 0, 0, 0, 0, 0])
    }
    fn encode_command(
        &self,
        motor: MotorRef<'_>,
        command: &Command,
    ) -> Result<CanFrame, CodecError> {
        let (limits, kp_max, kd_max) = ranges(self.identity(motor)?);
        let Command::Mit { q, dq, kp, kd, tau } = *command else {
            return Err(CodecError::CommandNotSupported {
                vendor: VENDOR_NAME,
                mode: command.kind(),
            });
        };
        let words = [
            pack(q, -limits.p_max, limits.p_max, "q")?,
            pack(dq, -limits.v_max, limits.v_max, "dq")?,
            pack(kp, 0.0, kp_max, "kp")?,
            pack(kd, 0.0, kd_max, "kd")?,
        ];
        let mut data = [0; 8];
        for (index, word) in words.iter().enumerate() {
            data[index * 2..index * 2 + 2].copy_from_slice(&word.to_be_bytes());
        }
        self.frame(
            motor,
            1,
            pack(tau, -limits.t_max, limits.t_max, "tau")?,
            &data,
        )
    }
    fn encode_set_mode(
        &self,
        motor: MotorRef<'_>,
        mode: CommandKind,
    ) -> Result<Option<CanFrame>, CodecError> {
        if mode != CommandKind::Mit {
            return Err(CodecError::CommandNotSupported {
                vendor: VENDOR_NAME,
                mode,
            });
        }
        self.encode_write_u32(motor, 0x7005, 0).map(Some)
    }
    fn encode_control_mode_readback(
        &self,
        motor: MotorRef<'_>,
    ) -> Result<Option<CanFrame>, CodecError> {
        self.encode_read_parameter(motor, 0x7005).map(Some)
    }
    fn decode_control_mode_readback(
        &self,
        frame: &CanFrame,
        motor: MotorRef<'_>,
    ) -> Result<Option<u32>, CodecError> {
        self.identity(motor)?;
        if frame.is_extended()
            && !frame.is_fd()
            && !frame.flags.contains(FrameFlags::REMOTE_REQUEST)
            && frame.id == ((17 << 24) | (motor.send_id << 8) | self.host_id as u32)
            && frame.len == 8
            && frame.payload()[..4] == [5, 112, 0, 0]
        {
            Ok(Some(frame.payload()[4] as u32))
        } else {
            Ok(None)
        }
    }
    fn decode(&self, frame: &CanFrame) -> Result<Option<Event>, CodecError> {
        if !frame.is_extended()
            || frame.is_fd()
            || frame.flags.contains(FrameFlags::REMOTE_REQUEST)
            || frame.id > 0x1FFF_FFFF
            || frame.id & 255 != self.host_id as u32
        {
            return Ok(None);
        }
        let kind = (frame.id >> 24) & 31;
        if !matches!(kind, 2 | 17 | 21) {
            return Ok(None);
        }
        let id = (frame.id >> 8) & 255;
        let Some(value) = self.models[id as usize] else {
            return Ok(None);
        };
        if frame.len != 8 {
            return Err(CodecError::DecodeFailed {
                reason: "RobStride payload must contain eight bytes",
            });
        }
        let data = frame.payload();
        if kind == 21 {
            let fault = u32::from_le_bytes(data[..4].try_into().unwrap());
            let warning = u32::from_le_bytes(data[4..].try_into().unwrap());
            return Ok(Some(Event::Fault {
                motor_id: id,
                code: ((fault | warning) & 0xFFFF) as u16
                    | if (fault | warning) >> 16 != 0 {
                        0x8000
                    } else {
                        0
                    },
            }));
        }
        if kind == 17 {
            let rid = u16::from_le_bytes([data[0], data[1]]);
            let value = match rid {
                0x7005 | 0x7029 => ParamValue::UInt(data[4] as u32),
                0x7028 => ParamValue::UInt(u32::from_le_bytes(data[4..].try_into().unwrap())),
                0x700B | 0x7016 | 0x7017 | 0x7019 | 0x701B | 0x701C => {
                    let raw = f32::from_le_bytes(data[4..].try_into().unwrap());
                    if !raw.is_finite() {
                        return Err(CodecError::DecodeFailed {
                            reason: "nonfinite parameter",
                        });
                    }
                    ParamValue::Float(raw as f64)
                }
                _ => return Ok(None),
            };
            return Ok(Some(Event::ParamReply {
                motor_id: id,
                rid,
                value,
            }));
        }
        let faults = ((frame.id >> 16) & 63) as u16;
        if faults != 0 {
            return Ok(Some(Event::Fault {
                motor_id: id,
                code: faults,
            }));
        }
        let words: [u16; 4] =
            core::array::from_fn(|i| u16::from_be_bytes([data[2 * i], data[2 * i + 1]]));
        let limits = ranges(value).0;
        Ok(Some(Event::State {
            motor_id: id,
            q: unpack(words[0], limits.p_max),
            dq: unpack(words[1], limits.v_max),
            tau: unpack(words[2], limits.t_max),
            t_mos: (words[3] / 10) as i16,
            t_rotor: i16::MIN,
        }))
    }
}

#[cfg(test)]
mod tests;
