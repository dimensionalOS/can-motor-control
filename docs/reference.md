# Python API

The reference below is generated from the installed `can_motor_control` package by
[mkdocstrings](https://mkdocstrings.github.io/). The prose comes from the
docstrings authored in the Rust PyO3 source, so it matches exactly what
`help()` and Jupyter `?` show.

!!! note
    This page requires the native extension to be importable. Build it first
    with `maturin develop` (the `make docs` / `make docs-build` targets do this
    for you).

## Robot and builder

::: can_motor_control.Robot

::: can_motor_control.RobotBuilder

## Groups

::: can_motor_control.Arm

::: can_motor_control.Gripper

::: can_motor_control.MotorGroup

::: can_motor_control.Motor

## Specs and frames

::: can_motor_control.MotorSpec

::: can_motor_control.CanFrame

## Transports

::: can_motor_control.MockCanBus

::: can_motor_control.SocketCanBus

::: can_motor_control.DmSerialBus

The hardware classes are platform-dependent. Linux exports `SocketCanBus`;
macOS exports `GsUsbBus` and `list_gs_usb_devices`. Both export `DmSerialBus`
and `MockCanBus`. See [CAN transports](transports.md) for all constructor
signatures and [Native macOS gs_usb](gs-usb-macos.md) for adapter counters.
The generated SocketCAN entry requires a Linux build of this site.

## Damiao codec

::: can_motor_control.damiao.DamiaoCodec

::: can_motor_control.damiao.MotorType

## Errors

::: can_motor_control.DmError

::: can_motor_control.TransportError

::: can_motor_control.CodecError

::: can_motor_control.ConfigError

::: can_motor_control.LifecycleError
