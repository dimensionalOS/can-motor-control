import numpy as np
import pytest

import can_motor_control as c
from can_motor_control.robstride import MotorType, RobstrideCodec


def build_robot(codec=None, recv_id=7):
    return (
        c.RobotBuilder()
        .add_bus("main", c.MockCanBus("mock"), codec or RobstrideCodec())
        .add_arm(
            "arm",
            bus="main",
            motors=[
                c.MotorSpec("gripper_motor", MotorType.RS00, send_id=7, recv_id=recv_id)
            ],
        )
        .build()
    )


def test_native_robstride_mit_loop():
    robot = build_robot()
    robot.connect()
    arm = robot["arm"]
    arm.set_mode("mit")
    robot.enable()
    commands = np.array([[2.0, 0.1, 0.0, 0.0, 0.0]], dtype=np.float64)
    for _ in range(100):
        arm.mit_control(commands)
        robot.tick(0)
    robot.disable()


def test_shared_host_is_not_a_receive_routing_key():
    robot = build_robot(recv_id=0xFD)
    with pytest.raises(c.CodecError):
        robot.connect()


def test_invalid_type_and_host_rejected():
    with pytest.raises(TypeError):
        c.MotorSpec("bad", 6, send_id=1, recv_id=1)
    with pytest.raises(OverflowError):
        RobstrideCodec(host_id=256)


def test_codec_can_only_be_consumed_once():
    codec = RobstrideCodec()
    build_robot(codec)
    with pytest.raises(ValueError):
        build_robot(codec)
