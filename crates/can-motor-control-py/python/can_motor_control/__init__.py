"""can_motor_control — Rust-backed control library for Damiao-family CAN motors."""

import sys

from can_motor_control._native import (
    CanFrame,
    MockCanBus,
    MotorSpec,
    Motor,
    Arm,
    Gripper,
    MotorGroup,
    Robot,
    RobotBuilder,
    DmError,
    TransportError,
    CodecError,
    ConfigError,
    LifecycleError,
)

if sys.platform == "linux":
    from can_motor_control._native import SocketCanBus
elif sys.platform == "darwin":
    from can_motor_control._native import GsUsbBus, GsUsbDeviceInfo, list_gs_usb_devices

try:
    from can_motor_control._native import MockFeedbackCodec
except ImportError:
    MockFeedbackCodec = None  # type: ignore[assignment]

from can_motor_control import damiao

__all__ = [
    "CanFrame",
    "MockCanBus",
    "MotorSpec",
    "Motor",
    "Arm",
    "Gripper",
    "MotorGroup",
    "Robot",
    "RobotBuilder",
    "DmError",
    "TransportError",
    "CodecError",
    "ConfigError",
    "LifecycleError",
    "damiao",
]

if sys.platform == "linux":
    __all__.append("SocketCanBus")
elif sys.platform == "darwin":
    __all__.extend(["GsUsbBus", "GsUsbDeviceInfo", "list_gs_usb_devices"])

if MockFeedbackCodec is not None:
    __all__.append("MockFeedbackCodec")

__version__ = "0.1.0"
