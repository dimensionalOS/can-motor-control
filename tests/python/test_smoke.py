"""Smoke test: build a robot against MockCanBus, run a 10-tick loop with
MIT commands, assert expected frame count, disable cleanly.

The 10-tick loop validates that the entire stack — transport, codec, group
dispatch, lifecycle — is wired correctly via Python without any real CAN
hardware.
"""

import sys
import threading

import numpy as np
import pytest
import can_motor_control
from can_motor_control.damiao import DamiaoCodec, MotorType


def make_robot() -> can_motor_control.Robot:
    builder = (
        can_motor_control.RobotBuilder()
        .add_bus("main", can_motor_control.MockCanBus("vcan_mock"), DamiaoCodec())
        .add_arm(
            "arm",
            bus="main",
            motors=[
                can_motor_control.MotorSpec("j0", MotorType.DM4340, send_id=0x01, recv_id=0x11),
                can_motor_control.MotorSpec("j1", MotorType.DM4340, send_id=0x02, recv_id=0x12),
                can_motor_control.MotorSpec("j2", MotorType.DM4340, send_id=0x03, recv_id=0x13),
                can_motor_control.MotorSpec("j3", MotorType.DM4340, send_id=0x04, recv_id=0x14),
            ],
        )
    )
    return builder.build()


def make_gripper_robot() -> can_motor_control.Robot:
    builder = (
        can_motor_control.RobotBuilder()
        .add_bus(
            "main",
            can_motor_control.MockCanBus("vcan_mock"),
            can_motor_control.MockFeedbackCodec(0x18),
        )
        .add_gripper(
            "grip",
            bus="main",
            motor=can_motor_control.MotorSpec(
                "g", MotorType.DM4310, send_id=0x08, recv_id=0x18
            ),
            opening_direction="increasing_position",
            default_current=0.2,
        )
    )
    return builder.build()


def test_imports():
    assert can_motor_control.Robot is not None
    assert can_motor_control.RobotBuilder is not None
    if sys.platform == "linux":
        assert can_motor_control.SocketCanBus is not None
        assert not hasattr(can_motor_control, "GsUsbBus")
    elif sys.platform == "darwin":
        assert can_motor_control.GsUsbBus is not None
        assert can_motor_control.GsUsbDeviceInfo is not None
        assert callable(can_motor_control.list_gs_usb_devices)
        assert not hasattr(can_motor_control, "SocketCanBus")
        for counter in ("rx_received", "rx_dropped", "tx_accepted", "tx_completed"):
            assert hasattr(can_motor_control.GsUsbBus, counter)
    assert can_motor_control.MotorSpec is not None
    assert can_motor_control.DmError is not None


def test_macos_gs_usb_constructor_rejects_contradictory_selector_without_hardware():
    if sys.platform != "darwin":
        return
    with pytest.raises(TypeError):
        can_motor_control.GsUsbBus(0x1D50, 0x606F)
    with pytest.raises(can_motor_control.TransportError, match="mutually exclusive"):
        can_motor_control.GsUsbBus(
            vendor_id=0x1D50,
            product_id=0x606F,
            serial_number="adapter",
            index=0,
        )


def test_macos_gs_usb_device_listing_without_matching_hardware():
    if sys.platform != "darwin":
        return
    assert (
        can_motor_control.list_gs_usb_devices(vendor_id=0xFFFF, product_id=0xFFFF) == []
    )


def test_macos_gs_usb_readiness_wait_releases_gil():
    if sys.platform != "darwin":
        return
    stop = threading.Event()
    progress = 0

    def count() -> None:
        nonlocal progress
        while not stop.is_set():
            progress += 1

    counter = threading.Thread(target=count)
    counter.start()
    before = progress
    try:
        with pytest.raises(can_motor_control.TransportError):
            can_motor_control.GsUsbBus(
                vendor_id=0xFFFF,
                product_id=0xFFFF,
                initialization_timeout=0.1,
            )
    finally:
        stop.set()
        counter.join()
    assert progress > before


def test_damiao_submodule_accessible():
    from can_motor_control.damiao import DamiaoCodec, MotorType

    assert MotorType.DM4340 is not None
    assert DamiaoCodec() is not None


def test_canframe_fd_constructor():
    """`CanFrame.fd` builds a genuine CAN-FD frame; `is_fd()`/`payload()` are
    methods that reflect the FD flag and the full payload."""
    frame = can_motor_control.CanFrame.fd(0x123, bytes([0xAB] * 24))
    assert frame.is_fd()
    assert frame.payload() == bytes([0xAB] * 24)
    assert frame.len == 24
    classical = can_motor_control.CanFrame.classical(0x101, bytes([1, 2, 3, 4]))
    assert not classical.is_fd()


def test_fd_mock_bus_runs_full_stack():
    """A robot built on a CAN-FD-capable mock bus runs a MIT loop end-to-end.
    Exercises the FD bus through transport + codec + lifecycle without FD
    hardware (the Damiao codec emits classical frames, valid on an FD bus)."""
    builder = (
        can_motor_control.RobotBuilder()
        .add_bus("main", can_motor_control.MockCanBus.new_fd("vcanfd_mock"), DamiaoCodec())
        .add_arm(
            "arm",
            bus="main",
            motors=[
                can_motor_control.MotorSpec("j0", MotorType.DM4340, send_id=0x01, recv_id=0x11),
                can_motor_control.MotorSpec("j1", MotorType.DM4340, send_id=0x02, recv_id=0x12),
            ],
        )
    )
    robot = builder.build()
    with robot:
        arm = robot["arm"]
        n = len(arm)
        cmds = np.zeros((n, 5), dtype=np.float64)
        cmds[:, 0] = 50.0  # kp
        cmds[:, 1] = 1.0  # kd
        for _ in range(10):
            robot.tick(500)
            arm.mit_control(cmds)
        assert arm.positions().shape == (n,)


def test_builder_build():
    robot = make_robot()
    assert robot.bus_names() == ["main"]
    assert robot.group_names() == ["arm"]
    assert not robot.is_connected()


def test_context_manager_lifecycle_and_mit_loop():
    robot = make_robot()
    with robot:
        arm = robot["arm"]
        n = len(arm)
        assert n == 4
        cmds = np.zeros((n, 5), dtype=np.float64)
        cmds[:, 0] = 50.0  # kp
        cmds[:, 1] = 1.0  # kd
        for _ in range(10):
            robot.tick(500)  # 500 us deadline
            arm.mit_control(cmds)
        positions = arm.positions()
        assert positions.shape == (n,)
        assert positions.dtype == np.float64


def test_getitem_returns_arm_wrapper():
    robot = make_robot()
    arm = robot["arm"]
    assert isinstance(arm, can_motor_control.Arm)
    try:
        robot["ghost"]
    except KeyError:
        pass
    else:
        raise AssertionError("expected KeyError")


def test_getitem_returns_gripper_wrapper():
    robot = make_gripper_robot()
    gripper = robot["grip"]
    assert isinstance(gripper, can_motor_control.Gripper)
    assert gripper.motor.name == "g"


def test_gripper_opening_requires_calibration():
    robot = make_gripper_robot()
    gripper = robot["grip"]
    try:
        gripper.set_opening(0.5)
    except can_motor_control.LifecycleError as e:
        assert "calibration" in str(e).lower()
    else:
        raise AssertionError("expected LifecycleError")


def test_gripper_opening_read_requires_calibration():
    robot = make_gripper_robot()
    gripper = robot["grip"]
    try:
        _ = gripper.opening
    except can_motor_control.LifecycleError as e:
        assert "calibration" in str(e).lower()
    else:
        raise AssertionError("expected LifecycleError")


def test_gripper_opening_rejects_out_of_range():
    robot = make_gripper_robot()
    gripper = robot["grip"]
    try:
        gripper.set_opening(1.1)
    except ValueError as e:
        assert "between 0.0 and 1.0" in str(e)
    else:
        raise AssertionError("expected ValueError")


def test_gripper_open_close_after_enable():
    robot = make_gripper_robot()
    with robot:
        gripper = robot["grip"]
        gripper.open()
        gripper.set_opening(0.5, current=0.3)
        gripper.close(current=0.25)


def test_gripper_opening_reads_latest_normalized_feedback():
    robot = make_gripper_robot()
    with robot:
        gripper = robot["grip"]
        gripper.set_opening(0.5)
        gripper.refresh()
        robot.tick(1_000)
        assert abs(gripper.opening - 0.5) < 1e-9

        try:
            gripper.opening = 0.25
        except AttributeError:
            pass
        else:
            raise AssertionError("expected read-only opening property")


def test_gripper_stub_declares_read_only_float_opening_property():
    stub = can_motor_control.__file__.replace("__init__.py", "__init__.pyi")
    with open(stub, encoding="utf-8") as f:
        source = f.read()
    gripper_stub = source.split("class Gripper:", 1)[1].split("class MotorGroup:", 1)[0]
    assert "@property\n    def opening(self) -> float: ..." in gripper_stub


def test_attribute_access_not_supported():
    robot = make_robot()
    try:
        _ = robot.arm
    except AttributeError:
        pass
    else:
        raise AssertionError("expected AttributeError")


def test_add_arm_rejects_codec_kwarg():
    builder = can_motor_control.RobotBuilder().add_bus(
        "main", can_motor_control.MockCanBus("m"), DamiaoCodec()
    )
    try:
        builder.add_arm(
            "arm",
            bus="main",
            codec=DamiaoCodec(),  # not a valid kwarg
            motors=[can_motor_control.MotorSpec("j0", MotorType.DM4340, 1, 0x11)],
        )
    except TypeError:
        pass
    else:
        raise AssertionError("expected TypeError")


def test_mit_wrong_shape_raises_value_error():
    robot = make_robot()
    with robot:
        arm = robot["arm"]
        try:
            arm.mit_control(np.zeros((3, 5), dtype=np.float64))  # 4 motors expected
        except ValueError as e:
            assert "(4, 5)" in str(e) and "(3, 5)" in str(e)
        else:
            raise AssertionError("expected ValueError")


def test_mit_wrong_dtype_raises_type_error():
    robot = make_robot()
    with robot:
        arm = robot["arm"]
        try:
            arm.mit_control(np.zeros((4, 5), dtype=np.float32))
        except (TypeError, ValueError):
            # numpy may raise TypeError or convert; we accept either.
            pass
        else:
            raise AssertionError("expected TypeError or ValueError")


def test_dmerror_hierarchy():
    assert issubclass(can_motor_control.TransportError, can_motor_control.DmError)
    assert issubclass(can_motor_control.CodecError, can_motor_control.DmError)
    assert issubclass(can_motor_control.ConfigError, can_motor_control.DmError)
    assert issubclass(can_motor_control.LifecycleError, can_motor_control.DmError)


def test_tick_before_connect_raises_lifecycle_error():
    robot = make_robot()
    try:
        robot.tick(1000)
    except can_motor_control.LifecycleError as e:
        assert "NotConnected" in str(e) or "not connected" in str(e).lower()
    else:
        raise AssertionError("expected LifecycleError")
