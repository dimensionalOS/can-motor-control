use can_motor_control::{MitCmd, MockCanBus, MotorSpec, RobotBuilder};
use motor_codec::CanFrame;
use robstride_codec::{RobstrideCodec, RobstrideMotorType};
use std::time::Duration;

#[test]
fn mixed_models_share_host_and_route_without_connect_motion() {
    let mock = MockCanBus::new("mock");
    let observer = mock.clone();
    let mut robot = RobotBuilder::new()
        .add_bus("main", Box::new(mock), Box::new(RobstrideCodec::new()))
        .add_arm(
            "arm",
            "main",
            vec![
                MotorSpec::new("j1", RobstrideMotorType::RS06, 1, 1),
                MotorSpec::new("j2", RobstrideMotorType::RS00, 7, 7),
            ],
        )
        .build()
        .unwrap();
    robot.connect().unwrap();
    assert!(observer.sent_frames().is_empty());
    for id in [1, 7] {
        observer.inject_frame(
            CanFrame::classical_extended(
                (2 << 24) | (2 << 22) | (id << 8) | 0xFD,
                &[255, 255, 255, 255, 255, 255, 1, 54],
            )
            .unwrap(),
        );
    }
    robot.tick(Duration::ZERO).unwrap();
    let arm = robot.group("arm").unwrap().as_arm().unwrap();
    assert_eq!(arm.positions(), vec![12.57, 12.57]);
    assert_eq!(arm.velocities(), vec![50.0, 33.0]);
    robot.enable().unwrap();
    let commands = [MitCmd {
        kp: 2.0,
        kd: 0.1,
        q: 0.0,
        dq: 0.0,
        tau: 0.0,
    }; 2];
    robot
        .group_mut("arm")
        .unwrap()
        .as_arm_mut()
        .unwrap()
        .mit_control(&commands)
        .unwrap();
    robot.disable().unwrap();
    assert!(observer
        .sent_frames()
        .iter()
        .all(|f| f.is_extended() && !f.is_fd()));
    assert!(observer
        .sent_frames()
        .iter()
        .all(|f| matches!(f.id >> 24, 1 | 3 | 4)));
}

#[test]
#[ignore = "Explicit timing measurement without physical CAN"]
fn native_codec_timing() {
    use motor_codec::{Command, MotorCodec, MotorRef};
    use std::{hint::black_box, time::Instant};
    let mut codec = RobstrideCodec::new();
    let motors: Vec<_> = (1..=7)
        .map(|id| MotorRef {
            motor_type: if id <= 3 {
                RobstrideMotorType::RS06.into()
            } else {
                RobstrideMotorType::RS00.into()
            },
            send_id: id,
            recv_id: id,
            name: "joint",
        })
        .collect();
    for motor in &motors {
        codec.bind_motor(*motor).unwrap();
    }
    let command = Command::Mit {
        q: 0.0,
        dq: 0.0,
        tau: 0.0,
        kp: 2.0,
        kd: 0.1,
    };
    let statuses: Vec<_> = motors
        .iter()
        .map(|m| {
            CanFrame::classical_extended(
                (2 << 24) | (2 << 22) | (m.send_id << 8) | 0xFD,
                &[127, 255, 127, 255, 127, 255, 1, 54],
            )
            .unwrap()
        })
        .collect();
    let mut samples = Vec::with_capacity(100_000);
    for _ in 0..100_000 {
        let start = Instant::now();
        for (motor, status) in motors.iter().zip(&statuses) {
            black_box(
                codec
                    .encode_command(black_box(*motor), black_box(&command))
                    .unwrap(),
            );
            black_box(codec.decode(black_box(status)).unwrap());
        }
        samples.push(start.elapsed().as_nanos());
    }
    samples.sort_unstable();
    println!(
        "seven motor encode and decode ns: p50={} p99={} max={}",
        samples[50_000], samples[99_000], samples[99_999]
    );
}
