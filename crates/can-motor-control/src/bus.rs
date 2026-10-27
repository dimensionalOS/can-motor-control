//! Bus = transport + codec + per-bus recv-id routing table.

use std::collections::HashMap;

use motor_codec::{BusCapabilities, MotorCodec, MotorTypeId};

use crate::transport::CanBus;

/// (group_name, motor_index) reached via a bus's recv-id routing table.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RouteKey {
    /// Group name that owns the target motor.
    pub group_name: String,
    /// Insertion-order index within that group.
    pub motor_index: usize,
}

/// One CAN interface plus the vendor codec attached to it.
///
/// One [`Bus`] instance per physical interface; the codec is shared across
/// every group attached to the bus, so inbound frames are decoded exactly
/// once per frame.
pub struct Bus {
    pub(crate) transport: Box<dyn CanBus>,
    pub(crate) codec: Box<dyn MotorCodec>,
    pub(crate) routes: HashMap<u32, RouteKey>,
}

impl Bus {
    /// Construct a bus and invoke `codec.bind_to_bus(transport.capabilities())`
    /// exactly once.
    pub fn new(transport: Box<dyn CanBus>, mut codec: Box<dyn MotorCodec>) -> Self {
        codec.bind_to_bus(transport.capabilities());
        Self {
            transport,
            codec,
            routes: HashMap::new(),
        }
    }

    /// Vendor short-name of the attached codec.
    pub fn vendor(&self) -> &str {
        self.codec.vendor_name()
    }

    /// Capabilities reported by the underlying transport.
    pub fn capabilities(&self) -> BusCapabilities {
        self.transport.capabilities()
    }

    /// True iff the bus's codec can encode/decode this motor type.
    pub fn codec_supports(&self, mt: MotorTypeId) -> bool {
        self.codec.supports(mt)
    }

    /// Read-only access to the per-bus recv-id routing table (populated by
    /// `Robot::connect`).
    pub fn routes(&self) -> &HashMap<u32, RouteKey> {
        &self.routes
    }
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::Arc;

    use motor_codec::{
        BusCapabilities, CanFrame, CodecError, Command, Event, Limits, MotorCodec, MotorRef,
        MotorTypeId,
    };

    use super::*;
    use crate::transport::MockCanBus;

    /// Codec that counts how many times `bind_to_bus` is invoked.
    pub(crate) struct CountingCodec {
        pub binds: Arc<AtomicUsize>,
        pub decodes: Arc<AtomicUsize>,
    }
    impl CountingCodec {
        pub fn new() -> (Self, Arc<AtomicUsize>, Arc<AtomicUsize>) {
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
            "mock"
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
        fn decode(
            &self,
            frame: &CanFrame,
            _motor: motor_codec::MotorRef<'_>,
        ) -> Result<Option<Event>, CodecError> {
            self.decodes.fetch_add(1, Ordering::SeqCst);
            Ok(Some(Event::State {
                motor_id: frame.id,
                q: 0.0,
                dq: 0.0,
                tau: 0.0,
                t_mos: 0,
                t_rotor: 0,
            }))
        }
    }

    #[test]
    fn bus_new_calls_bind_to_bus_once() {
        let (codec, binds, _) = CountingCodec::new();
        let _bus = Bus::new(Box::new(MockCanBus::new("m")), Box::new(codec));
        assert_eq!(binds.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn bus_exposes_vendor_and_caps() {
        let (codec, _, _) = CountingCodec::new();
        let bus = Bus::new(Box::new(MockCanBus::new("m")), Box::new(codec));
        assert_eq!(bus.vendor(), "mock");
        assert_eq!(bus.capabilities(), BusCapabilities::classical());
    }
}
