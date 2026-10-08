use super::*;
fn motor(id: u32, kind: RobstrideMotorType) -> MotorRef<'static> {
    MotorRef {
        motor_type: kind.into(),
        send_id: id,
        recv_id: id,
        name: "joint",
    }
}
fn codec() -> RobstrideCodec {
    let mut c = RobstrideCodec::new();
    c.bind_motor(motor(1, RobstrideMotorType::RS06)).unwrap();
    c.bind_motor(motor(7, RobstrideMotorType::RS00)).unwrap();
    c
}
fn status(id: u32, flags: u32, data: &[u8]) -> CanFrame {
    CanFrame::classical_extended((2 << 24) | (flags << 16) | (id << 8) | 0xFD, data).unwrap()
}
#[test]
fn lifecycle_and_mode_golden_frames() {
    let c = codec();
    let m = motor(7, RobstrideMotorType::RS00);
    assert_eq!(c.encode_enable(m).unwrap().id, 0x0300FD07);
    assert_eq!(c.encode_disable(m).unwrap().id, 0x0400FD07);
    let zero = c.encode_set_zero(m).unwrap();
    assert_eq!(zero.id, 0x0600FD07);
    assert_eq!(zero.payload(), &[1, 0, 0, 0, 0, 0, 0, 0]);
    let mode = c.encode_set_mode(m, CommandKind::Mit).unwrap().unwrap();
    assert_eq!(mode.id, 0x1200FD07);
    assert_eq!(mode.payload(), &[5, 112, 0, 0, 0, 0, 0, 0]);
    assert!(c.encode_refresh(m).unwrap().is_none());
    assert!(c.encode_set_mode(m, CommandKind::PosForce).is_err());
}
#[test]
fn mit_midpoint_and_endpoint_packets() {
    let c = codec();
    let m = motor(7, RobstrideMotorType::RS00);
    let frame = c
        .encode_command(
            m,
            &Command::Mit {
                q: 0.0,
                dq: 0.0,
                tau: 0.0,
                kp: 500.0,
                kd: 5.0,
            },
        )
        .unwrap();
    assert_eq!(frame.id, 0x017FFF07);
    assert_eq!(frame.payload(), &[127, 255, 127, 255, 255, 255, 255, 255]);
    assert!(frame.is_extended());
    assert!(!frame.is_fd());
    let frame = c
        .encode_command(
            m,
            &Command::Mit {
                q: 12.57,
                dq: 33.0,
                tau: 14.0,
                kp: 0.0,
                kd: 0.0,
            },
        )
        .unwrap();
    assert_eq!(frame.id, 0x01FFFF07);
    assert_eq!(frame.payload(), &[255, 255, 255, 255, 0, 0, 0, 0]);
}
#[test]
fn rejects_nonfinite_and_outside_limits() {
    let c = codec();
    let m = motor(7, RobstrideMotorType::RS00);
    for q in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 12.58, -12.58] {
        assert!(c
            .encode_command(
                m,
                &Command::Mit {
                    q,
                    dq: 0.0,
                    tau: 0.0,
                    kp: 1.0,
                    kd: 0.1
                }
            )
            .is_err());
    }
    for command in [
        Command::Mit {
            q: 0.0,
            dq: 34.0,
            tau: 0.0,
            kp: 1.0,
            kd: 0.1,
        },
        Command::Mit {
            q: 0.0,
            dq: 0.0,
            tau: 15.0,
            kp: 1.0,
            kd: 0.1,
        },
        Command::Mit {
            q: 0.0,
            dq: 0.0,
            tau: 0.0,
            kp: 501.0,
            kd: 0.1,
        },
        Command::Mit {
            q: 0.0,
            dq: 0.0,
            tau: 0.0,
            kp: 1.0,
            kd: 6.0,
        },
        Command::Vel { dq: 0.0 },
    ] {
        assert!(c.encode_command(m, &command).is_err());
    }
}
#[test]
fn shared_host_model_scaling_and_missing_temperature() {
    let c = codec();
    for (id, velocity, torque) in [(1, 50.0, 36.0), (7, 33.0, 14.0)] {
        let e = c
            .decode(&status(id, 0x80, &[255, 255, 255, 255, 255, 255, 1, 54]))
            .unwrap()
            .unwrap();
        assert_eq!(
            e,
            Event::State {
                motor_id: id,
                q: 12.57,
                dq: velocity,
                tau: torque,
                t_mos: 31,
                t_rotor: i16::MIN
            }
        );
    }
}
#[test]
fn routing_filters_wrong_host_unknown_motor_and_foreign_frames() {
    let c = codec();
    let mut frame = status(7, 0, &[0; 8]);
    frame.id = (frame.id & !255) | 0xFE;
    assert!(c.decode(&frame).unwrap().is_none());
    assert!(c.decode(&status(6, 0, &[0; 8])).unwrap().is_none());
    assert!(c
        .decode(&CanFrame::classical(7, &[0; 8]).unwrap())
        .unwrap()
        .is_none());
    assert!(c
        .decode(&CanFrame::fd_extended(status(7, 0, &[0; 8]).id, &[0; 8]).unwrap())
        .unwrap()
        .is_none());
}
#[test]
fn malformed_status_and_fault_bits() {
    let c = codec();
    assert!(c.decode(&status(7, 0, &[0; 7])).is_err());
    for bit in 0..6 {
        assert_eq!(
            c.decode(&status(7, 1 << bit, &[0; 8])).unwrap(),
            Some(Event::Fault {
                motor_id: 7,
                code: 1 << bit
            })
        );
    }
    let f = CanFrame::classical_extended(0x150007FD, &[0, 0, 1, 0, 0, 0, 0, 0]).unwrap();
    assert_eq!(
        c.decode(&f).unwrap(),
        Some(Event::Fault {
            motor_id: 7,
            code: 0x8000
        })
    );
}
#[test]
fn parameters_and_contextual_mode_readback() {
    let c = codec();
    let m = motor(7, RobstrideMotorType::RS00);
    let f = c.encode_write_u32(m, 0x7028, 250).unwrap();
    assert_eq!(f.id, 0x1200FD07);
    assert_eq!(f.payload(), &[40, 112, 0, 0, 250, 0, 0, 0]);
    assert!(c.encode_write_f32(m, 0x700B, f32::NAN).is_err());
    let f = CanFrame::classical_extended(0x110007FD, &[5, 112, 0, 0, 0, 0, 0, 0]).unwrap();
    assert_eq!(c.decode_control_mode_readback(&f, m).unwrap(), Some(0));
    assert_eq!(
        c.decode(&f).unwrap(),
        Some(Event::ParamReply {
            motor_id: 7,
            rid: 0x7005,
            value: ParamValue::UInt(0)
        })
    );
    assert_eq!(
        c.decode_control_mode_readback(&f, motor(1, RobstrideMotorType::RS06))
            .unwrap(),
        None
    );
}
#[test]
fn identity_and_model_rejections() {
    let mut c = codec();
    let mut m = motor(7, RobstrideMotorType::RS00);
    m.recv_id = 0xFD;
    assert!(c.bind_motor(m).is_err());
    assert!(c.bind_motor(motor(0, RobstrideMotorType::RS00)).is_err());
    assert!(c.bind_motor(motor(256, RobstrideMotorType::RS00)).is_err());
    assert!(c.bind_motor(motor(7, RobstrideMotorType::RS06)).is_err());
    assert!(!c.supports(MotorTypeId::Damiao(0)));
    assert!(!c.supports(MotorTypeId::Robostride(1)));
    assert_eq!(
        parse_motor_type("rs-06"),
        Some(RobstrideMotorType::RS06.into())
    );
}

#[test]
fn fd_capable_bus_still_emits_classical_and_custom_host_isolated() {
    let mut c = RobstrideCodec::with_host_id(0xFE);
    c.bind_to_bus(BusCapabilities::fd());
    let m = motor(7, RobstrideMotorType::RS00);
    c.bind_motor(m).unwrap();
    let frame = c.encode_enable(m).unwrap();
    assert_eq!(frame.id, 0x0300FE07);
    assert!(!frame.is_fd());
    assert!(c.decode(&status(7, 0, &[0; 8])).unwrap().is_none());
    let mut frame = status(7, 0, &[0; 8]);
    frame.id = (frame.id & !255) | 0xFE;
    assert!(matches!(
        c.decode(&frame).unwrap(),
        Some(Event::State { motor_id: 7, .. })
    ));
}

#[test]
fn rs06_full_scale_packet() {
    let c = codec();
    let m = motor(1, RobstrideMotorType::RS06);
    let f = c
        .encode_command(
            m,
            &Command::Mit {
                q: -12.57,
                dq: -50.0,
                tau: -36.0,
                kp: 5000.0,
                kd: 100.0,
            },
        )
        .unwrap();
    assert_eq!(f.id, 0x01000001);
    assert_eq!(f.payload(), &[0, 0, 0, 0, 255, 255, 255, 255]);
}
