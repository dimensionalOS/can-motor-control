## 1. Transport

- [x] 1.1 Add `DmSerialBus` with packet encoding, an incremental receive parser, raw exclusive open and pollable fd
- [x] 1.2 Write each packet whole; mark the bus broken after an incomplete packet
- [x] 1.3 Bound the frame gap and validate it in Rust and Python
- [x] 1.4 Export `DmSerialBus` from Rust and Python on Linux and macOS, accept it in `RobotBuilder.add_bus`, update stubs and the README platform table

## 2. Gripper calibration travel

- [x] 2.1 Add `GripperOpeningSpec::calibration_travel` and the Python `add_gripper(calibration_travel=...)` option
- [x] 2.2 Validate the travel at enable before any gripper command

## 3. Verification

- [x] 3.1 Rust tests: encoding, rejection, parser split/merge/noise, pty round trip, double open, hang-up, frame gap bound, calibration travel
- [x] 3.2 Python tests: robot round trip over a pty, invalid frame gaps, double open
- [x] 3.3 Hardware: Seeed reBot B601-DM through its bridge on Linux (reads, enable, gripper calibration over 6.39 rad, 100 Hz control)
- [ ] 3.4 Hardware on macOS
