## Why

The Damiao USB-CAN bridge (USB 2e88:4603) ships with Damiao motor kits and Seeed reBot arms. It enumerates as a CDC-ACM serial port, not as a SocketCAN or gs_usb device, so `can-motor-control` cannot drive those robots without a second, Python-side Damiao stack. DimOS asked integrations to extend this library's transports instead of duplicating the motor driver. The reBot B601-DM gripper also travels about 6.4 rad between its stops, more than the fixed 4.0 rad opening calibration travel, so calibration cannot reach both stops.

## What Changes

- Add `DmSerialBus`, a classical-CAN `CanBus` over the bridge's serial port on Linux and macOS. It opens the port exclusively in raw mode, writes each packet as one whole write, parses and resynchronizes the bridge's receive packets, and exposes a pollable fd.
- Accept `DmSerialBus` in the Python `RobotBuilder.add_bus`, with type stubs, and export it from Linux and macOS builds alongside the native CAN transport.
- Add an optional per-gripper `calibration_travel` (Rust `GripperOpeningSpec::with_calibration_travel`, Python `add_gripper(calibration_travel=...)`), validated before calibration moves the gripper. The default stays 4.0 rad.

## Capabilities

### New Capabilities

- `damiao-serial-transport`: The Damiao USB-CAN serial bridge as a `CanBus`.

### Modified Capabilities

- `gripper-opening-control`: Configurable opening calibration travel.

## Impact

- Rust transport module, crate exports, robot calibration, PyO3 wrappers and stubs, Python and Rust tests, README platform table.
- No change to existing transports or to the default calibration behavior.
- Downstream DimOS can run the Seeed reBot B601-DM on the shared Damiao adapter through the bridge shipped with the arm.
