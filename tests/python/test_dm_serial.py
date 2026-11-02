"""DmSerialBus against a pseudo-terminal standing in for the Damiao USB bridge."""

import os
import select
import struct
import sys

import pytest
import can_motor_control
from can_motor_control.damiao import DamiaoCodec, MotorType

pytestmark = pytest.mark.skipif(
    sys.platform not in ("linux", "darwin"), reason="DmSerialBus needs a POSIX tty"
)


@pytest.fixture
def bridge():
    master, slave = os.openpty()
    try:
        yield master, os.ttyname(slave)
    finally:
        os.close(slave)
        os.close(master)


def read_packet(fd: int, size: int = 30) -> bytes:
    data = b""
    while len(data) < size:
        ready, _, _ = select.select([fd], [], [], 1.0)
        assert ready, "bridge packet not written"
        data += os.read(fd, size - len(data))
    return data


def feedback_packet(recv_id: int, payload: bytes) -> bytes:
    return b"\xaa\x11\x08" + struct.pack("<I", recv_id) + payload + b"\x55"


@pytest.mark.parametrize("frame_gap", [-1.0, float("nan"), float("inf"), 1.5, 1e19, 1e30])
def test_rejects_invalid_frame_gap(bridge, frame_gap):
    _, path = bridge
    with pytest.raises(ValueError):
        can_motor_control.DmSerialBus(path, frame_gap=frame_gap)


def test_second_open_of_the_same_port_raises_transport_error(bridge):
    _, path = bridge
    first = can_motor_control.DmSerialBus(path)
    with pytest.raises(can_motor_control.TransportError):
        can_motor_control.DmSerialBus(path)
    del first


def test_missing_device_raises_transport_error():
    with pytest.raises(can_motor_control.TransportError):
        can_motor_control.DmSerialBus("/dev/does-not-exist-dm-bridge")


def test_robot_round_trip_over_bridge(bridge):
    master, path = bridge
    robot = (
        can_motor_control.RobotBuilder()
        .add_bus("main", can_motor_control.DmSerialBus(path, frame_gap=0.0), DamiaoCodec())
        .add_arm(
            "arm",
            bus="main",
            motors=[can_motor_control.MotorSpec("j0", MotorType.DM4310, 0x01, 0x11)],
        )
        .build()
    )
    robot.connect()
    robot.refresh()
    packet = read_packet(master)
    assert packet[:4] == b"\x55\xaa\x1e\x03"
    assert struct.unpack_from("<I", packet, 13)[0] == 0x7FF
    assert packet[21:24] == b"\x01\x00\xcc"

    # Status 1 (enabled), motor ID 1, position at the encoder midpoint (0 rad).
    os.write(master, feedback_packet(0x11, bytes([0x11, 0x80, 0x00, 0x80, 0x08, 0x00, 30, 31])))
    for _ in range(100):
        robot.tick(10_000)
        if robot["arm"]["j0"].temperature_mos == 30:
            break
    assert robot["arm"]["j0"].temperature_mos == 30
    assert abs(robot["arm"].positions()[0]) < 1e-3
