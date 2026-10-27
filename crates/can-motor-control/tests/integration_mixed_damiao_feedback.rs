use std::time::Duration;

use can_motor_control::{CanFrame, MockCanBus, MotorSpec, RobotBuilder};
use damiao_codec::{DamiaoCodec, DamiaoMotorType};

#[test]
fn mixed_damiao_feedback_uses_each_motors_profile_and_bus() {
    let main = MockCanBus::new("main");
    let other = MockCanBus::new("other");
    let mut robot = RobotBuilder::new()
        .add_bus("main", Box::new(main.clone()), Box::new(DamiaoCodec::new()))
        .add_bus(
            "other",
            Box::new(other.clone()),
            Box::new(DamiaoCodec::new()),
        )
        .add_arm(
            "arm",
            "main",
            vec![
                MotorSpec::new("shoulder", DamiaoMotorType::DM4340, 1, 0x11),
                MotorSpec::new("wrist", DamiaoMotorType::DM4310, 2, 0x12),
            ],
        )
        .add_arm(
            "other_arm",
            "other",
            vec![MotorSpec::new("wrist", DamiaoMotorType::DM4310, 1, 0x11)],
        )
        .add_gripper(
            "gripper",
            "main",
            MotorSpec::new("gripper", DamiaoMotorType::DM4310, 8, 0x18),
        )
        .build()
        .unwrap();
    robot.connect().unwrap();

    // Position minimum, velocity maximum, torque minimum in the wire format.
    let payload = [0x01, 0, 0, 0xff, 0xf0, 0, 30, 35];
    main.inject_frame(CanFrame::classical(0x11, &payload).unwrap());
    main.inject_frame(CanFrame::classical(0x12, &payload).unwrap());
    main.inject_frame(CanFrame::classical(0x18, &payload).unwrap());
    other.inject_frame(CanFrame::classical(0x11, &payload).unwrap());
    robot.tick(Duration::ZERO).unwrap();

    let arm = robot.group("arm").unwrap().as_arm().unwrap();
    assert_eq!(arm.positions(), vec![-12.5, -12.5]);
    assert_eq!(arm.velocities(), vec![10.0, 30.0]);
    assert_eq!(arm.torques(), vec![-28.0, -10.0]);
    let other_arm = robot.group("other_arm").unwrap().as_arm().unwrap();
    assert_eq!(other_arm.positions(), vec![-12.5]);
    assert_eq!(other_arm.velocities(), vec![30.0]);
    assert_eq!(other_arm.torques(), vec![-10.0]);
    let gripper = robot.group("gripper").unwrap().as_gripper().unwrap();
    assert_eq!(gripper.motor().position(), -12.5);
    assert_eq!(gripper.motor().velocity(), 30.0);
    assert_eq!(gripper.motor().torque(), -10.0);
}
