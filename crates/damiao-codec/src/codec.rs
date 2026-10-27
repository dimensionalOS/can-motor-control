//! [`DamiaoCodec`] — implements [`motor_codec::MotorCodec`] for Damiao motors.

use motor_codec::{
    BusCapabilities, CanFrame, CodecError, Command, CommandKind, Event, FrameFlags, Limits,
    MotorCodec, MotorRef, MotorTypeId, ParamValue,
};

use crate::bitpack::{pack_mit_payload, unpack_state_payload};
use crate::limits::limits_for;
use crate::types::DamiaoMotorType;

const VENDOR: &str = "damiao";

/// Damiao motor codec.
///
/// Construct with [`DamiaoCodec::new`]. Implements
/// [`MotorCodec`] for the vendor-agnostic surface and [`crate::DamiaoCodecExt`]
/// for the Damiao-specific `0x7FF` parameter sub-protocol.
/// Feedback decoding requires the configured motor identity and uses its limits.
#[derive(Default, Debug, Clone)]
pub struct DamiaoCodec {
    bound_caps: Option<BusCapabilities>,
}

impl DamiaoCodec {
    /// Construct a new codec. `bind_to_bus` must be called before any encode.
    pub fn new() -> Self {
        Self { bound_caps: None }
    }

    fn motor_type(&self, id: MotorTypeId) -> Result<DamiaoMotorType, CodecError> {
        match id {
            MotorTypeId::Damiao(d) => {
                DamiaoMotorType::from_discriminant(d).ok_or(CodecError::UnknownMotorType {
                    vendor: VENDOR,
                    type_id: d,
                })
            }
            _ => Err(CodecError::UnknownMotorType {
                vendor: VENDOR,
                type_id: 0,
            }),
        }
    }

    fn encode_special(
        &self,
        motor: MotorRef<'_>,
        trailing_byte: u8,
    ) -> Result<CanFrame, CodecError> {
        let payload = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, trailing_byte];
        CanFrame::classical(motor.send_id, &payload).map_err(|_| CodecError::DecodeFailed {
            reason: "classical frame construction failed",
        })
    }

    fn decode_state(&self, frame: &CanFrame, limits: Limits) -> Result<Option<Event>, CodecError> {
        // Damiao state responses come on the recv_id assigned to the motor, with
        // the response command in the top nibble of byte 0.
        if frame.is_fd() && !self.bound_caps.is_some_and(|c| c.supports_fd) {
            // Not bound to an FD bus: an FD frame isn't a Damiao state frame we
            // expect here. (A classical binding therefore behaves exactly as v1.)
            return Ok(None);
        }
        if frame.flags.contains(FrameFlags::REMOTE_REQUEST) {
            return Ok(None);
        }
        // Parameter traffic is not a state frame. In particular, a 0x55 write
        // acknowledgement must never update motor state.
        if frame.id == 0x7FF || (frame.payload()[2] == 0x33 || frame.payload()[2] == 0x55) {
            return Ok(None);
        }
        if frame.len != 8 {
            // Damiao always uses 8 bytes; a different length is not a Damiao state frame.
            return Ok(None);
        }
        let byte0 = frame.payload()[0];
        let reported_id = byte0 & 0x0f;
        let payload: [u8; 8] =
            frame
                .payload()
                .try_into()
                .map_err(|_| CodecError::DecodeFailed {
                    reason: "expected 8-byte payload",
                })?;
        let (_, _, q, dq, tau, t_mos, t_rotor) =
            unpack_state_payload(&payload, limits.p_max, limits.v_max, limits.t_max);
        if reported_id == 0 && frame.id == 0 {
            return Ok(None);
        }
        Ok(Some(Event::State {
            motor_id: frame.id,
            q,
            dq,
            tau,
            t_mos,
            t_rotor,
        }))
    }
}

impl MotorCodec for DamiaoCodec {
    fn vendor_name(&self) -> &'static str {
        VENDOR
    }

    fn supports(&self, motor_type: MotorTypeId) -> bool {
        matches!(motor_type, MotorTypeId::Damiao(d) if DamiaoMotorType::from_discriminant(d).is_some())
    }

    fn limits(&self, motor_type: MotorTypeId) -> Result<Limits, CodecError> {
        let t = self.motor_type(motor_type)?;
        Ok(limits_for(t))
    }

    fn bind_to_bus(&mut self, caps: BusCapabilities) {
        // The bound capabilities decide which frame formats this codec accepts
        // on decode. Emission stays conservative — Damiao command/state frames
        // are 8 bytes, valid on a classical or an FD bus — so a classical
        // binding reproduces the v1 byte layout exactly.
        self.bound_caps = Some(caps);
    }

    fn encode_enable(&self, motor: MotorRef<'_>) -> Result<CanFrame, CodecError> {
        self.encode_special(motor, 0xFC)
    }

    fn encode_disable(&self, motor: MotorRef<'_>) -> Result<CanFrame, CodecError> {
        self.encode_special(motor, 0xFD)
    }

    fn encode_set_zero(&self, motor: MotorRef<'_>) -> Result<CanFrame, CodecError> {
        self.encode_special(motor, 0xFE)
    }

    fn encode_command(&self, motor: MotorRef<'_>, cmd: &Command) -> Result<CanFrame, CodecError> {
        let t = self.motor_type(motor.motor_type)?;
        let lim = limits_for(t);
        match cmd {
            Command::Mit { kp, kd, q, dq, tau } => {
                check_range("q", *q, lim.p_max)?;
                check_range("dq", *dq, lim.v_max)?;
                check_range("tau", *tau, lim.t_max)?;
                check_unsigned("kp", *kp, 500.0)?;
                check_unsigned("kd", *kd, 5.0)?;
                let payload =
                    pack_mit_payload(*q, *dq, *kp, *kd, *tau, lim.p_max, lim.v_max, lim.t_max);
                CanFrame::classical(motor.send_id, &payload).map_err(|_| CodecError::DecodeFailed {
                    reason: "MIT frame construction failed",
                })
            }
            Command::PosVel { q, dq } => {
                let mut payload = [0u8; 8];
                payload[0..4].copy_from_slice(&(*q as f32).to_le_bytes());
                payload[4..8].copy_from_slice(&(*dq as f32).to_le_bytes());
                CanFrame::classical(0x100 + motor.send_id, &payload).map_err(|_| {
                    CodecError::DecodeFailed {
                        reason: "PosVel frame construction failed",
                    }
                })
            }
            Command::Vel { dq } => {
                let mut payload = [0u8; 8];
                payload[0..4].copy_from_slice(&(*dq as f32).to_le_bytes());
                CanFrame::classical(0x200 + motor.send_id, &payload).map_err(|_| {
                    CodecError::DecodeFailed {
                        reason: "Vel frame construction failed",
                    }
                })
            }
            Command::PosForce { q, dq, i_pu } => {
                let mut payload = [0u8; 8];
                payload[0..4].copy_from_slice(&(*q as f32).to_le_bytes());
                let dq_u = (dq * 100.0) as u16;
                let i_u = (i_pu * 10000.0) as u16;
                payload[4..6].copy_from_slice(&dq_u.to_le_bytes());
                payload[6..8].copy_from_slice(&i_u.to_le_bytes());
                CanFrame::classical(0x300 + motor.send_id, &payload).map_err(|_| {
                    CodecError::DecodeFailed {
                        reason: "PosForce frame construction failed",
                    }
                })
            }
            _ => Err(CodecError::CommandNotSupported {
                vendor: VENDOR,
                mode: cmd.kind(),
            }),
        }
    }

    fn encode_refresh(&self, motor: MotorRef<'_>) -> Result<Option<CanFrame>, CodecError> {
        // The `refresh_motor_status` query (0xCC on 0x7FF) requests a feedback
        // frame without commanding motion. Disambiguate from the same-named
        // `DamiaoCodecExt::encode_refresh`, which returns the raw frame.
        Ok(Some(crate::DamiaoCodecExt::encode_refresh(self, motor)))
    }

    fn encode_set_mode(
        &self,
        motor: MotorRef<'_>,
        mode: CommandKind,
    ) -> Result<Option<CanFrame>, CodecError> {
        // Damiao CTRL_MODE register (RID 10): MIT=1, PosVel=2, Vel=3, PosForce=4.
        let value: u32 = match mode {
            CommandKind::Mit => 1,
            CommandKind::PosVel => 2,
            CommandKind::Vel => 3,
            CommandKind::PosForce => 4,
            _ => {
                return Err(CodecError::CommandNotSupported {
                    vendor: VENDOR,
                    mode,
                })
            }
        };
        Ok(Some(crate::DamiaoCodecExt::encode_write_param(
            self,
            motor,
            crate::DamiaoRid::CTRL_MODE,
            ParamValue::UInt(value),
        )))
    }

    fn encode_control_mode_readback(
        &self,
        motor: MotorRef<'_>,
    ) -> Result<Option<CanFrame>, CodecError> {
        Ok(Some(crate::DamiaoCodecExt::encode_read_param(
            self,
            motor,
            crate::DamiaoRid::CTRL_MODE,
        )))
    }

    fn decode_control_mode_readback(
        &self,
        frame: &CanFrame,
        motor: MotorRef<'_>,
    ) -> Result<Option<u32>, CodecError> {
        if frame.len != 8 {
            return Ok(None);
        }
        let p = frame.payload();
        let send_id = u16::from_le_bytes([p[0], p[1]]) as u32;
        if send_id != motor.send_id || p[2] != 0x33 || p[3] != u8::from(crate::DamiaoRid::CTRL_MODE)
        {
            return Ok(None);
        }
        Ok(Some(u32::from_le_bytes([p[4], p[5], p[6], p[7]])))
    }

    fn decode(&self, frame: &CanFrame, motor: MotorRef<'_>) -> Result<Option<Event>, CodecError> {
        if frame.id != motor.recv_id {
            return Ok(None);
        }
        self.decode_state(frame, self.limits(motor.motor_type)?)
    }
}

fn check_range(field: &'static str, x: f64, magnitude: f64) -> Result<(), CodecError> {
    if x.abs() > magnitude {
        Err(CodecError::OutOfRange { field })
    } else {
        Ok(())
    }
}

fn check_unsigned(field: &'static str, x: f64, max: f64) -> Result<(), CodecError> {
    if !(0.0..=max).contains(&x) {
        Err(CodecError::OutOfRange { field })
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::boxed::Box;

    fn ref_motor(name: &str, mt: DamiaoMotorType, send: u32, recv: u32) -> MotorRef<'_> {
        MotorRef {
            motor_type: mt.into(),
            send_id: send,
            recv_id: recv,
            name,
        }
    }

    #[test]
    fn constructible_as_trait_object() {
        let c: Box<dyn MotorCodec> = Box::new(DamiaoCodec::new());
        assert_eq!(c.vendor_name(), "damiao");
    }

    #[test]
    fn supports_all_known_skus() {
        let c = DamiaoCodec::new();
        for d in 0..=12u16 {
            assert!(c.supports(MotorTypeId::Damiao(d)), "discriminant {d}");
            assert!(c.limits(MotorTypeId::Damiao(d)).is_ok());
        }
    }

    #[test]
    fn rejects_unknown_and_other_vendors() {
        let c = DamiaoCodec::new();
        assert!(!c.supports(MotorTypeId::Damiao(0xFFFF)));
        assert!(!c.supports(MotorTypeId::Robostride(0)));
        assert!(matches!(
            c.limits(MotorTypeId::Damiao(0xFFFF)),
            Err(CodecError::UnknownMotorType {
                vendor: "damiao",
                type_id: 0xFFFF,
            })
        ));
    }

    #[test]
    fn enable_disable_setzero_byte_patterns() {
        let c = DamiaoCodec::new();
        let m = ref_motor("g", DamiaoMotorType::DM4310, 0x05, 0x18);
        let e = c.encode_enable(m).unwrap();
        assert_eq!(e.id, 0x05);
        assert_eq!(e.len, 8);
        assert_eq!(
            e.payload(),
            &[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
        );

        let d = c.encode_disable(m).unwrap();
        assert_eq!(d.payload()[7], 0xFD);
        let z = c.encode_set_zero(m).unwrap();
        assert_eq!(z.payload()[7], 0xFE);
    }

    #[test]
    fn mit_frame_id_equals_send_id_and_not_fd() {
        let c = DamiaoCodec::new();
        let m = ref_motor("j0", DamiaoMotorType::DM4340, 0x01, 0x11);
        let f = c
            .encode_command(
                m,
                &Command::Mit {
                    kp: 50.0,
                    kd: 1.0,
                    q: 0.0,
                    dq: 0.0,
                    tau: 0.0,
                },
            )
            .unwrap();
        assert_eq!(f.id, 0x01);
        assert_eq!(f.len, 8);
        assert!(!f.is_fd());
    }

    #[test]
    fn mit_out_of_range_tau() {
        let c = DamiaoCodec::new();
        let m = ref_motor("j0", DamiaoMotorType::DM4340, 0x01, 0x11);
        let r = c.encode_command(
            m,
            &Command::Mit {
                kp: 0.0,
                kd: 0.0,
                q: 0.0,
                dq: 0.0,
                tau: 1000.0,
            },
        );
        assert!(matches!(r, Err(CodecError::OutOfRange { field: "tau" })));
    }

    #[test]
    fn posvel_uses_0x100_offset() {
        let c = DamiaoCodec::new();
        let m = ref_motor("j0", DamiaoMotorType::DM4340, 0x01, 0x11);
        let f = c
            .encode_command(m, &Command::PosVel { q: 1.0, dq: 2.0 })
            .unwrap();
        assert_eq!(f.id, 0x101);
        assert_eq!(&f.payload()[0..4], &1.0f32.to_le_bytes());
        assert_eq!(&f.payload()[4..8], &2.0f32.to_le_bytes());
    }

    #[test]
    fn vel_uses_0x200_offset() {
        let c = DamiaoCodec::new();
        let m = ref_motor("j0", DamiaoMotorType::DM4340, 0x01, 0x11);
        let f = c.encode_command(m, &Command::Vel { dq: 1.5 }).unwrap();
        assert_eq!(f.id, 0x201);
        assert_eq!(&f.payload()[0..4], &1.5f32.to_le_bytes());
    }

    #[test]
    fn posforce_uses_0x300_offset_with_integer_scaling() {
        let c = DamiaoCodec::new();
        let m = ref_motor("j0", DamiaoMotorType::DM4340, 0x01, 0x11);
        let f = c
            .encode_command(
                m,
                &Command::PosForce {
                    q: 1.0,
                    dq: 2.0,
                    i_pu: 0.5,
                },
            )
            .unwrap();
        assert_eq!(f.id, 0x301);
        assert_eq!(&f.payload()[0..4], &1.0f32.to_le_bytes());
        assert_eq!(&f.payload()[4..6], &200u16.to_le_bytes());
        assert_eq!(&f.payload()[6..8], &5000u16.to_le_bytes());
    }

    #[test]
    fn decode_foreign_frame_returns_none() {
        let c = DamiaoCodec::new();
        let f = CanFrame::classical(0x00, &[0x00; 8]).unwrap();
        assert!(matches!(
            c.decode(&f, ref_motor("j", DamiaoMotorType::DM4340, 1, 0x11)),
            Ok(None)
        ));
    }

    #[test]
    fn contextual_feedback_decodes_known_wire_values_with_each_profile() {
        let c = DamiaoCodec::new();
        let frame = CanFrame::classical(0x11, &[1, 0, 0, 0xff, 0xf0, 0, 30, 35]).unwrap();
        for (model, velocity, torque) in [
            (DamiaoMotorType::DM4340, 10.0, -28.0),
            (DamiaoMotorType::DM4310, 30.0, -10.0),
        ] {
            let motor = ref_motor("j", model, 1, 0x11);
            match c.decode(&frame, motor).unwrap().unwrap() {
                Event::State {
                    motor_id,
                    q,
                    dq,
                    tau,
                    t_mos,
                    t_rotor,
                } => {
                    assert_eq!(motor_id, 0x11);
                    assert_eq!(q, -12.5);
                    assert_eq!(dq, velocity);
                    assert_eq!(tau, torque);
                    assert_eq!((t_mos, t_rotor), (30, 35));
                }
                other => panic!("expected state, got {other:?}"),
            }
        }
    }

    #[test]
    fn contextual_feedback_ignores_another_motors_receive_id() {
        let c = DamiaoCodec::new();
        let frame = CanFrame::classical(0x12, &[1, 0, 0, 0xff, 0xf0, 0, 30, 35]).unwrap();
        let motor = ref_motor("j", DamiaoMotorType::DM4310, 1, 0x11);
        assert!(c.decode(&frame, motor).unwrap().is_none());
    }

    #[test]
    fn decode_truncated_returns_none() {
        let c = DamiaoCodec::new();
        let f = CanFrame::classical(0x11, &[0x11, 0x00, 0x00]).unwrap();
        assert!(matches!(
            c.decode(&f, ref_motor("j", DamiaoMotorType::DM4340, 1, 0x11)),
            Ok(None)
        ));
    }

    #[test]
    fn decode_state_frame_round_trips_within_lsb() {
        let c = DamiaoCodec::new();
        let lim = limits_for(DamiaoMotorType::DM4340);
        let (q, dq, tau) = (1.0, 0.5, 5.0);
        let q_u = crate::bitpack::float_to_uint(q, -lim.p_max, lim.p_max, 16);
        let dq_u = crate::bitpack::float_to_uint(dq, -lim.v_max, lim.v_max, 12);
        let tau_u = crate::bitpack::float_to_uint(tau, -lim.t_max, lim.t_max, 12);
        let payload = [
            // byte 0: high nibble = err (0), low nibble = cmd_id (1 = MIT response)
            0x01,
            ((q_u >> 8) & 0xff) as u8,
            (q_u & 0xff) as u8,
            ((dq_u >> 4) & 0xff) as u8,
            ((((dq_u & 0xf) << 4) | ((tau_u >> 8) & 0xf)) & 0xff) as u8,
            (tau_u & 0xff) as u8,
            30,
            35,
        ];
        let f = CanFrame::classical(0x11, &payload).unwrap();
        match c
            .decode(&f, ref_motor("j", DamiaoMotorType::DM4340, 1, 0x11))
            .unwrap()
            .unwrap()
        {
            Event::State {
                motor_id,
                q: qo,
                dq: dqo,
                tau: tauo,
                t_mos,
                t_rotor,
            } => {
                assert_eq!(motor_id, 0x11);
                assert!((qo - q).abs() < 0.001);
                assert!((dqo - dq).abs() < 0.01);
                assert!((tauo - tau).abs() < 0.05);
                assert_eq!(t_mos, 30);
                assert_eq!(t_rotor, 35);
            }
            _ => panic!("expected State"),
        }
    }

    #[test]
    fn decode_dm4310_enable_disable_replies_from_socketcan_capture() {
        let c = DamiaoCodec::new();
        for payload in [
            [0x18, 0x81, 0x07, 0x7F, 0xE7, 0xFF, 0x1D, 0x1B],
            [0x08, 0x81, 0x07, 0x80, 0x07, 0xFF, 0x1D, 0x1B],
        ] {
            match c
                .decode(
                    &CanFrame::classical(0x18, &payload).unwrap(),
                    ref_motor("gripper", DamiaoMotorType::DM4310, 8, 0x18),
                )
                .unwrap()
            {
                Some(Event::State {
                    motor_id,
                    t_mos,
                    t_rotor,
                    ..
                }) => {
                    assert_eq!(motor_id, 0x18);
                    assert_eq!(t_mos, 29);
                    assert_eq!(t_rotor, 27);
                }
                other => panic!("expected State, got {other:?}"),
            }
        }
    }

    #[test]
    fn classical_emission_regardless_of_bound_caps() {
        let m = ref_motor("j0", DamiaoMotorType::DM4340, 0x01, 0x11);
        for caps in [BusCapabilities::classical(), BusCapabilities::fd()] {
            let mut c = DamiaoCodec::new();
            c.bind_to_bus(caps);
            let f = c
                .encode_command(
                    m,
                    &Command::Mit {
                        kp: 0.0,
                        kd: 0.0,
                        q: 0.0,
                        dq: 0.0,
                        tau: 0.0,
                    },
                )
                .unwrap();
            assert!(!f.is_fd(), "caps={caps:?} produced FD frame");
            assert_eq!(f.len, 8);
        }
    }

    /// Encoding is byte-for-byte identical whether bound classical, bound FD, or
    /// unbound — the conservative-emission invariant the FD change must preserve.
    #[test]
    fn classical_binding_byte_identical_to_unbound() {
        let m = ref_motor("j0", DamiaoMotorType::DM4340, 0x01, 0x11);
        let cmd = Command::Mit {
            kp: 50.0,
            kd: 1.0,
            q: 0.25,
            dq: -0.5,
            tau: 0.1,
        };
        let unbound = DamiaoCodec::new().encode_command(m, &cmd).unwrap();
        let mut classical = DamiaoCodec::new();
        classical.bind_to_bus(BusCapabilities::classical());
        let mut fd = DamiaoCodec::new();
        fd.bind_to_bus(BusCapabilities::fd());
        let cf = classical.encode_command(m, &cmd).unwrap();
        let ff = fd.encode_command(m, &cmd).unwrap();
        assert!(!cf.is_fd() && !ff.is_fd() && !unbound.is_fd());
        assert_eq!(cf.id, unbound.id);
        assert_eq!(cf.payload(), unbound.payload());
        assert_eq!(ff.payload(), unbound.payload());
    }

    /// A representative Damiao state frame, carried in FD format, decodes only
    /// when the codec is bound to an FD bus; a classical/unbound codec treats it
    /// as not-ours (preserving v1 behavior).
    #[test]
    fn fd_state_frame_decoded_only_when_bound_fd() {
        let lim = limits_for(DamiaoMotorType::DM4340);
        let (q, dq, tau) = (1.0, 0.5, 5.0);
        let q_u = crate::bitpack::float_to_uint(q, -lim.p_max, lim.p_max, 16);
        let dq_u = crate::bitpack::float_to_uint(dq, -lim.v_max, lim.v_max, 12);
        let tau_u = crate::bitpack::float_to_uint(tau, -lim.t_max, lim.t_max, 12);
        let payload = [
            0x01,
            ((q_u >> 8) & 0xff) as u8,
            (q_u & 0xff) as u8,
            ((dq_u >> 4) & 0xff) as u8,
            ((((dq_u & 0xf) << 4) | ((tau_u >> 8) & 0xf)) & 0xff) as u8,
            (tau_u & 0xff) as u8,
            30,
            35,
        ];
        // 8-byte payload is a valid FD DLC, so this is a genuine FD-format frame.
        let fd_frame = CanFrame::fd(0x11, &payload).unwrap();
        assert!(fd_frame.is_fd());

        // Unbound and classical-bound: not ours → None (v1 behavior).
        assert!(matches!(
            DamiaoCodec::new().decode(&fd_frame, ref_motor("j", DamiaoMotorType::DM4340, 1, 0x11)),
            Ok(None)
        ));
        let mut classical = DamiaoCodec::new();
        classical.bind_to_bus(BusCapabilities::classical());
        assert!(matches!(
            classical.decode(&fd_frame, ref_motor("j", DamiaoMotorType::DM4340, 1, 0x11)),
            Ok(None)
        ));

        // FD-bound: decoded as a state event.
        let mut fd = DamiaoCodec::new();
        fd.bind_to_bus(BusCapabilities::fd());
        match fd
            .decode(&fd_frame, ref_motor("j", DamiaoMotorType::DM4340, 1, 0x11))
            .unwrap()
            .unwrap()
        {
            Event::State { motor_id, .. } => assert_eq!(motor_id, 0x11),
            _ => panic!("expected State"),
        }
    }

    #[test]
    fn encode_refresh_matches_openarm_layout() {
        let c = DamiaoCodec::new();
        let m = ref_motor("j0", DamiaoMotorType::DM4340, 0x01, 0x11);
        // The MotorCodec trait method wraps the Damiao 0xCC/0x7FF query.
        let f = MotorCodec::encode_refresh(&c, m)
            .unwrap()
            .expect("damiao supports refresh");
        assert_eq!(f.id, 0x7FF);
        assert_eq!(f.len, 8);
        assert_eq!(f.payload(), &[0x01, 0x00, 0xCC, 0, 0, 0, 0, 0]);
        assert!(!f.is_fd());
    }

    #[test]
    fn encode_set_mode_writes_ctrl_mode_register() {
        let c = DamiaoCodec::new();
        let m = ref_motor("j0", DamiaoMotorType::DM4340, 0x01, 0x11);
        let f = MotorCodec::encode_set_mode(&c, m, CommandKind::Mit)
            .unwrap()
            .expect("damiao supports set_mode");
        assert_eq!(f.id, 0x7FF);
        assert_eq!(f.len, 8);
        let p = f.payload();
        assert_eq!(&p[0..2], &0x01u16.to_le_bytes()); // send id, LE
        assert_eq!(p[2], 0x55); // write-param command
        assert_eq!(p[3], 10); // CTRL_MODE register
        assert_eq!(&p[4..8], &1u32.to_le_bytes()); // MIT == 1
    }

    #[test]
    fn ctrl_mode_readback_is_contextual_and_not_state() {
        let c = DamiaoCodec::new();
        let m = ref_motor("g", DamiaoMotorType::DM4310, 0x05, 0x18);
        let query = c.encode_control_mode_readback(m).unwrap().unwrap();
        assert_eq!(query.id, 0x7FF);
        assert_eq!(&query.payload()[0..4], &[0x05, 0x00, 0x33, 0x0A]);

        let mut response = [0u8; 8];
        response[0..2].copy_from_slice(&0x05u16.to_le_bytes());
        response[2] = 0x33;
        response[3] = 0x0A;
        response[4..8].copy_from_slice(&4u32.to_le_bytes());
        let response = CanFrame::classical(0x18, &response).unwrap();
        assert_eq!(
            c.decode_control_mode_readback(&response, m).unwrap(),
            Some(4)
        );
        assert!(c.decode(&response, m).unwrap().is_none());

        let wrong_send = CanFrame::classical(0x18, &[0x06, 0, 0x33, 0x0A, 4, 0, 0, 0]).unwrap();
        assert_eq!(
            c.decode_control_mode_readback(&wrong_send, m).unwrap(),
            None
        );
        let write_ack = CanFrame::classical(0x18, &[0x05, 0, 0x55, 0x0A, 4, 0, 0, 0]).unwrap();
        assert_eq!(c.decode_control_mode_readback(&write_ack, m).unwrap(), None);
    }

    extern crate alloc;
}
