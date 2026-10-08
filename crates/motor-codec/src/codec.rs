//! The [`MotorCodec`] trait — the seam between `can-motor-control` and vendor-specific
//! motor protocols.

use thiserror::Error;

use crate::{
    BusCapabilities, CanFrame, Command, CommandKind, Event, Limits, MotorRef, MotorTypeId,
};

/// The vendor-agnostic codec contract.
///
/// Every vendor codec (Damiao, Robostride, MyActuator, ...) implements this
/// trait. `can-motor-control` uses `Box<dyn MotorCodec>` exclusively — it never
/// depends on a specific vendor's crate.
///
/// The trait is intentionally object-safe (no generics, no `Self: Sized`
/// constraints on the methods) so a `Box<dyn MotorCodec>` is a valid field
/// type.
pub trait MotorCodec: Send + Sync {
    /// Vendor short name used in error messages and the TOML registry
    /// (e.g. `"damiao"`).
    fn vendor_name(&self) -> &'static str;

    /// True iff this codec can encode commands for and decode events from the
    /// supplied motor type.
    fn supports(&self, motor_type: MotorTypeId) -> bool;

    /// Per-motor-type physical limits.
    ///
    /// Returns [`CodecError::UnknownMotorType`] when the motor type is not in
    /// this codec's vendor space (or is an unknown SKU within the vendor space).
    fn limits(&self, motor_type: MotorTypeId) -> Result<Limits, CodecError>;

    /// Called exactly once when this codec is bound to a [`crate::caps::BusCapabilities`].
    ///
    /// The codec MAY remember the capabilities for later use (e.g. to decide
    /// whether to emit CAN-FD frames). Codecs may assume `bind_to_bus` has
    /// been called by the time any encode method is invoked.
    fn bind_to_bus(&mut self, caps: BusCapabilities);

    /// Bind motor identity before receiving frames. The default preserves stateless codecs.
    fn bind_motor(&mut self, motor: MotorRef<'_>) -> Result<(), CodecError> {
        let _ = motor;
        Ok(())
    }

    /// Encode the lifecycle "enable motor" command.
    fn encode_enable(&self, motor: MotorRef<'_>) -> Result<CanFrame, CodecError>;

    /// Encode the lifecycle "disable motor" command.
    fn encode_disable(&self, motor: MotorRef<'_>) -> Result<CanFrame, CodecError>;

    /// Encode the lifecycle "set this position as zero" command.
    fn encode_set_zero(&self, motor: MotorRef<'_>) -> Result<CanFrame, CodecError>;

    /// Encode a control-mode command.
    fn encode_command(&self, motor: MotorRef<'_>, cmd: &Command) -> Result<CanFrame, CodecError>;

    /// Encode a "request current motor state" frame, if the vendor protocol has
    /// one.
    ///
    /// The returned frame MUST request a state-feedback reply **without
    /// commanding any motion**. Returns `Ok(None)` (the default) when the codec
    /// has no such query; callers skip those motors. This lets a read loop poll
    /// state via `refresh` then [`MotorCodec::decode`] without applying torque.
    fn encode_refresh(&self, motor: MotorRef<'_>) -> Result<Option<CanFrame>, CodecError> {
        let _ = motor;
        Ok(None)
    }

    /// Encode a "set the motor's persistent control mode" frame, if the vendor
    /// protocol supports it.
    ///
    /// `mode` selects which control law the motor will accept
    /// (MIT / PosVel / Vel / PosForce). Returns `Ok(None)` (the default) when
    /// the codec has no such command; callers skip those motors. This commands
    /// no motion — call it once at startup, before the matching
    /// [`MotorCodec::encode_command`] mode.
    fn encode_set_mode(
        &self,
        motor: MotorRef<'_>,
        mode: CommandKind,
    ) -> Result<Option<CanFrame>, CodecError> {
        let _ = (motor, mode);
        Ok(None)
    }

    /// Encode a private control-mode read-back query, when supported.
    fn encode_control_mode_readback(
        &self,
        motor: MotorRef<'_>,
    ) -> Result<Option<CanFrame>, CodecError> {
        let _ = motor;
        Ok(None)
    }

    /// Decode a private control-mode read-back response for `motor`.
    fn decode_control_mode_readback(
        &self,
        frame: &CanFrame,
        motor: MotorRef<'_>,
    ) -> Result<Option<u32>, CodecError> {
        let _ = (frame, motor);
        Ok(None)
    }

    /// Decode an inbound frame.
    ///
    /// Returns `Ok(Some(event))` for a recognized inbound message,
    /// `Ok(None)` for frames the codec does not recognize (foreign vendor, or
    /// a CAN ID outside the codec's address range), and
    /// `Err(CodecError::DecodeFailed { .. })` for frames that look like the
    /// codec's vendor but fail to parse.
    fn decode(&self, frame: &CanFrame) -> Result<Option<Event>, CodecError>;
}

/// Errors returned by [`MotorCodec`] implementations.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
#[non_exhaustive]
pub enum CodecError {
    /// The motor type is unknown to this vendor codec.
    #[error("{vendor} codec does not know motor type id {type_id:#06x}")]
    UnknownMotorType {
        /// Vendor short name.
        vendor: &'static str,
        /// Opaque discriminant the codec did not recognize.
        type_id: u16,
    },

    /// The requested command mode is not implemented for the (codec, motor type)
    /// pair.
    #[error("{vendor} codec does not support command mode {mode:?}")]
    CommandNotSupported {
        /// Vendor short name.
        vendor: &'static str,
        /// The command discriminant.
        mode: CommandKind,
    },

    /// A recognized vendor frame could not be parsed (truncated payload,
    /// unsupported sub-protocol revision, etc.).
    #[error("frame decode failed: {reason}")]
    DecodeFailed {
        /// Human-readable explanation.
        reason: &'static str,
    },

    /// A command field exceeds the motor's published limit.
    #[error("value out of range: field {field}")]
    OutOfRange {
        /// Field name (`"q"`, `"dq"`, `"tau"`, `"kp"`, ...).
        field: &'static str,
    },
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::boxed::Box;

    /// Dummy codec used only for object-safety assertions.
    struct Dummy;
    impl MotorCodec for Dummy {
        fn vendor_name(&self) -> &'static str {
            "dummy"
        }
        fn supports(&self, _: MotorTypeId) -> bool {
            false
        }
        fn limits(&self, _: MotorTypeId) -> Result<Limits, CodecError> {
            Err(CodecError::UnknownMotorType {
                vendor: "dummy",
                type_id: 0,
            })
        }
        fn bind_to_bus(&mut self, _: BusCapabilities) {}
        fn encode_enable(&self, _: MotorRef<'_>) -> Result<CanFrame, CodecError> {
            Err(CodecError::DecodeFailed { reason: "stub" })
        }
        fn encode_disable(&self, _: MotorRef<'_>) -> Result<CanFrame, CodecError> {
            Err(CodecError::DecodeFailed { reason: "stub" })
        }
        fn encode_set_zero(&self, _: MotorRef<'_>) -> Result<CanFrame, CodecError> {
            Err(CodecError::DecodeFailed { reason: "stub" })
        }
        fn encode_command(&self, _: MotorRef<'_>, _: &Command) -> Result<CanFrame, CodecError> {
            Err(CodecError::DecodeFailed { reason: "stub" })
        }
        fn decode(&self, _: &CanFrame) -> Result<Option<Event>, CodecError> {
            Ok(None)
        }
    }

    #[test]
    fn trait_object_safe() {
        let _: Box<dyn MotorCodec> = Box::new(Dummy);
    }

    #[test]
    fn default_encode_refresh_is_none() {
        let m = MotorRef {
            motor_type: MotorTypeId::Damiao(3),
            send_id: 1,
            recv_id: 0x11,
            name: "j",
        };
        assert!(matches!(Dummy.encode_refresh(m), Ok(None)));
    }

    #[test]
    fn default_encode_set_mode_is_none() {
        let m = MotorRef {
            motor_type: MotorTypeId::Damiao(3),
            send_id: 1,
            recv_id: 0x11,
            name: "j",
        };
        assert!(matches!(
            Dummy.encode_set_mode(m, CommandKind::Mit),
            Ok(None)
        ));
    }

    #[test]
    fn out_of_range_variant_constructible() {
        let e = CodecError::OutOfRange { field: "tau" };
        let s = alloc::format!("{e}");
        assert!(s.contains("tau"));
    }

    #[test]
    fn unknown_motor_type_display() {
        let e = CodecError::UnknownMotorType {
            vendor: "damiao",
            type_id: 0xFFFF,
        };
        let s = alloc::format!("{e}");
        assert!(s.contains("damiao"));
        assert!(s.contains("0xffff"));
    }
}
