# RobStride support

The Rust codec supports RS00 and RS06 in MIT operation mode using extended classical CAN. Python sends native Rust codec commands through the existing transport. No pure Python CAN encoder is used. Scheduling calls from a Python loop still requires timing measurements on the target host.

Use `can_motor_control.robstride.RobstrideCodec(host_id=0xFD)` and `can_motor_control.robstride.MotorType.RS00` or `RS06`. Set `MotorSpec.send_id` and `recv_id` to the motor device ID, such as 7. Here recv_id is the logical routing key, not the complete variable wire identifier. All motors share a host address, and the identifier contains fault and mode bits that change over time. At connect the codec binds motor IDs to their model limits without sending frames. Wrong hosts and unknown device IDs are ignored.

TOML uses vendor `robstride`, models `RS00` or `RS06` and the default host 0xFD. A custom host currently requires the programmatic builder. Separate buses may use different hosts. MIT mode selection writes run_mode 0; the caller must disable the motor before changing its mode. Mode readback is available through the codec. Set zero is explicit and emits only the type 6 frame. It does not write zero_state or save parameters.

Only MIT commands are supported by the generic command interface. PosVel, Vel and PosForce fail explicitly. Normalized gripper opening calibration depends on PosForce and is therefore unsupported. Control the gripper motor through bounded MIT coordinates in an application that supplies its own calibration and safety layer. Refresh is unsupported because a single parameter read does not yield a complete state. Enabled motors report status in response to MIT commands. The codec neither enables active reporting nor silently disables a motor to obtain feedback.

Status faults produce Fault events rather than updating a healthy state. Type 21 fault and warning words are folded into the existing u16 fault interface; a high bit marker preserves the presence of upper word bits. The original full words are not retained by that interface. RobStride reports one temperature. MOS temperature uses the existing integer Celsius interface with truncation, and unavailable rotor temperature uses i16::MIN. Packet receipt freshness and safety watchdogs are responsibilities of the control application.

## Protocol evidence

Repository base: ef5c082b94d77fbfbf50b4a73a41dca44f578acf.

[RS00 manual](https://files.seeedstudio.com/products/RobStride/Product%20Literature/RS00/RS00User%20Manual251112.pdf), SHA256 4644a4d68d0c94e0873b766a4c72b2f98234f1f1c6113ebae811adcc6531364a.

[RS06 manual](https://files.seeedstudio.com/products/RobStride/Product%20Literature/RS06/RS06User%20Manual251112.pdf), SHA256 8db82ff1e0248ae5a12c5bed5a6eb66280d018178bfdff4a40e5c3dd973f3e0c.

The November 2025 manuals specify position magnitude 12.57 radians, RS00 velocity 33 and torque 14, and RS06 velocity 50 and torque 36. Gain maxima are 500 and 5 for RS00, 5000 and 100 for RS06. The RS00 sample lists positive V_MIN despite a signed range; the codec uses the symmetric range. Command quantization follows the manual C formula with 65535 levels.

[Official Python sample](https://github.com/RobStride/Python_Sample/tree/cbf977e56c842d57a65f3f17c1b1ecaef002c424) confirms message types, identifier fields, endianness and register layouts. Its table has different model ranges and uses midpoint 32767 scaling. Those values are not silently combined with the newer manual profile. Firmware ranges must be checked before hardware use. No physical motion or timing qualification is claimed.

## Validation on October 7, 2026

The locked Rust workspace suite passes, including ten RobStride packet tests and a mixed model routing integration test. Rustfmt and Clippy with warnings treated as errors pass. The codec also passes a no default features check on the x86 host. Existing hardware dependent tests retain their upstream conditional skips when virtual CAN is absent; these results do not represent physical CAN qualification.

A release abi3 wheel for Python 3.10 and later was built for manylinux_2_34_x86_64 and installed in a fresh Python 3.11 environment. All 28 Python tests pass, including four RobStride tests and the existing Damiao regressions.

The release native codec benchmark over 100,000 iterations measured seven motor encode and decode p50 170 ns and p99 260 ns. A separate run reached a maximum 21,040 ns and used 4,004 KiB peak resident memory. These measurements exclude transport and Python.

The release wheel benchmark over 10,000 iterations with OPENBLAS_NUM_THREADS=1 measured seven motor command submission and MockCanBus tick p50 1,030 ns, p99 3,590 ns and maximum 73,799 ns. Elapsed time was 0.01403 seconds, process CPU time 0.01403 seconds and peak resident memory 35,388 KiB. MockCanBus retains sent frames, so the memory measurement includes accumulated test traffic. This is not a CAN wire latency or a hard real time guarantee. The reproducible script is `scripts/benchmark_robstride_mock.py`.

Native Jetson validation is pending because Tailscale requires an operator authentication check. Physical motor operation was not attempted. Raspberry Pi qualification remains pending.

The codec also builds successfully without default features for thumbv7em-none-eabihf, an embedded target without an operating system. This is a compilation check and does not satisfy physical ARM qualification.

## Native Jetson validation on October 8, 2026

The locked Rust workspace tests passed on the physical development Jetson Orin, including ten protocol tests and mixed model routing. The release native Python extension built on aarch64 and all 28 Python tests passed there in 6.39 seconds. A seven motor MockCanBus loop over 10,000 iterations with one BLAS thread measured p50 2,593 ns, p99 5,440 ns and maximum 273,131 ns, with elapsed time 0.03284 seconds, process CPU time 0.03272 seconds and peak resident memory 34,964 KiB. No physical CAN access occurred. Physical motor behavior and Pi qualification remain pending.
