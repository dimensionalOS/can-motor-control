# can-motor-control

Rust control library for Damiao-family CAN motors, with first-class Python bindings.

## Status

v0.1.0 (in development) — single-arm control over Linux SocketCAN or native macOS gs_usb. Linux supports CAN-FD; macOS gs_usb supports classical CAN. See [`docs/can-fd.md`](./docs/can-fd.md) and [`docs/gs-usb-macos.md`](./docs/gs-usb-macos.md).

## Why

The two reference implementations — `cmjang/DM_Control_Python` and `enactic/openarm_can` — are either a single-file script that fuses sends with receives, or a C++ tree with a binding crate that rebuilds on every release. This library is built as a Rust core with a thin PyO3 binding so it installs as a single wheel and runs in realtime control loops without GIL contention.

## Installation

```bash
# Python (once 0.1.0 is published):
pip install can-motor-control

# Rust from GitHub while crates.io publishing is deferred:
[dependencies]
can-motor-control = { git = "https://github.com/TomCC7/can-motor-control", tag = "v0.0.1" }
damiao-codec = { git = "https://github.com/TomCC7/can-motor-control", package = "can-motor-damiao-codec", tag = "v0.0.1" }
```

Rust users can depend on the public GitHub repository directly while crates.io
publication is deferred. Pin a tag or commit for reproducible builds. The
companion package names are prefixed (`can-motor-codec`,
`can-motor-damiao-codec`) for a future crates.io release, while their Rust
import crate names remain `motor_codec` and `damiao_codec`.

Build from source:

```bash
# Rust workspace
cargo build --workspace --release

# Python wheel
cd crates/can-motor-control-py
maturin build --release
pip install target/wheels/can_motor_control-*.whl
```

## Layout

```
crates/
  motor-codec/          # no_std, package can-motor-codec, import motor_codec
  damiao-codec/         # no_std, package can-motor-damiao-codec, import damiao_codec
  can-motor-control/       # std, native transport + Robot/Group/Motor + builder
  can-motor-control-py/    # PyO3 bindings, built by maturin
configs/            # example TOML robot configs
examples/           # Python hardware bring-up ladder (00_-06_) + Rust example
tests/python/       # Python smoke + integration tests
docs/               # architecture + multi-vendor + CAN-FD notes
openspec/           # source of truth: proposals, designs, specs, tasks
```

## Hardware bring-up examples

Real hardware bring-up should be done by walking the `examples/00_*` ->
`examples/06_*` ladder in order. Each step adds exactly one kind of risk;
do not skip steps when bringing up an arm for the first time. Mock and
`MockCanBus`-driven tests prove the API surface and CI wiring, **not**
that real hardware will respond correctly.

| Python example                         | Rust example           | Sends                  | Hardware required             |
| -------------------------------------- | ---------------------- | ---------------------- | ----------------------------- |
| `00_can_interface_check.py`            | `00_can_interface_check` | nothing              | none (checks `/sys/class/net`) |
| `01_single_motor_enable_disable.py`    | `01_single_motor_enable_disable` | enable + disable | one powered motor             |
| `02_read_single_motor.py`              | --                     | enable + live refresh-poll + disable | one powered motor |
| `03_single_motor_mit_hold.py`          | `03_single_motor_mit_hold` | bounded MIT hold   | one powered motor             |
| `04_single_motor_control_modes.py`     | --                     | one bounded mode/run   | one powered motor             |
| `05_gripper_opening_control.py`        | `05_gripper_opening_control` | normalized gripper opening (`open`, `set_opening`, `close`) | one gripper motor |
| `05_gripper_control.py`                | `05_gripper_control`   | raw gripper PosForce sequence, MIT, or PosVel | one gripper motor |
| `06_read_robot_motor.py`               | --                     | live OpenArm/OpenYam zero-torque reader | configured OpenArm or OpenYam single arm |

Each example prints its assumptions before sending anything and disables
on exit. Durations are hard-capped; live readers accept `--mock` for API-shape
checks, but mock execution does not validate hardware.

For Linux setup, see [`docs/socketcan-setup.md`](./docs/socketcan-setup.md). For
macOS adapter selection and diagnostics, see
[`docs/gs-usb-macos.md`](./docs/gs-usb-macos.md).
Run Rust examples with `cargo run -p can-motor-control --example <Rust example> -- ...`.

## Documentation

Hosted documentation is published from `main` to
<https://tomcc7.github.io/can-motor-control/> by the GitHub Pages workflow.

A local docs site (Python API reference, guides, and hosted Rust rustdoc) is
built with MkDocs plus `cargo doc`. The Python API prose is generated from the
package's docstrings, so the native extension is built first. The environment
is managed by [uv](https://docs.astral.sh/uv/) and pinned by `uv.lock` for
reproducibility (the root `pyproject.toml` is a dev-only environment, not a
published package).

```bash
# Serve with live reload at http://127.0.0.1:8000
make docs

# Or render a static site into ./site
make docs-build
```

`make docs-build` runs `scripts/build-docs.sh`, which provisions the uv
environment, builds the MkDocs site, runs `cargo doc --no-deps --workspace --locked`,
and copies rustdoc into `site/rustdoc/` for deployment. No separate virtualenv
or `pip install` step is needed — uv provisions everything from `uv.lock`.

## Releasing

Maintainers should follow the release runbook in
[`docs/release.md`](./docs/release.md) before publishing to PyPI or crates.io.
It covers registry setup, release-candidate dry-runs, crates.io publish order,
PyPI Trusted Publishing, and post-publish install checks.

## Source of truth

The architecture, every requirement, and the implementation plan live under [`openspec/`](./openspec). New features should propose changes there first.

## Platforms

| Target | Hardware transport | Scope |
| --- | --- | --- |
| Linux | `SocketCanBus` | Classical CAN and CAN-FD |
| macOS | `GsUsbBus` | gs_usb channel 0, classical CAN |

Each build exports only its native hardware transport. Both export
`MockCanBus`. The macOS wheel uses IOKit through Rust `nusb`; it requires no
Homebrew libusb, PyUSB, or Python gs_usb package.

## License

Dual-licensed under Apache-2.0 OR MIT.

### Custom Damiao firmware profiles

Users can define firmware scaling without editing this library or adding a
motor SKU enum. Register a profile on the bus codec and use it in `MotorSpec`:

```python
from can_motor_control import MockCanBus, MotorSpec, Robot
from can_motor_control.damiao import DamiaoCodec, MotorProfile

shoulder = MotorProfile(0x8000, p_max=6.28, v_max=10.0, t_max=120.0)
robot = (
    Robot.builder()
    .add_bus("main", MockCanBus("example"), DamiaoCodec(profiles=[shoulder]))
    .add_arm("arm", bus="main", motors=[MotorSpec("joint", shoulder, 1, 0x11)])
    .build()
)
```

`id` is a codec-local profile key, not a CAN ID. Use 0x8000..=0xffff for
custom profiles; built-in SKU IDs cannot be overridden. Register every custom
profile used on a bus in that bus's codec. IDs may be reused on independent
codecs. Duplicate registrations are rejected. Existing `MotorType` members
and `DamiaoCodec()` work unchanged.

Rust users call `DamiaoCodec::new().with_profile(id, motor_codec::Limits { ... })`
and use `motor_codec::MotorTypeId::Damiao(id)` in `MotorSpec`. The codec applies
the registered ranges to both MIT command encoding and feedback decoding,
including mixed custom/built-in motors on the same bus. This API is for code-built
robots; the built-in TOML loader continues to resolve stock SKU names.

`p_max`, `v_max`, and `t_max` are positive finite magnitudes in radians, rad/s,
and Nm; their full spans must also be finite. These configure host protocol
scaling, not mechanical safety limits, gains, gear ratios, or firmware registers.
They must match the actual motor firmware configuration. The example values
come from the [MakerMods Metal J2 firmware table](https://github.com/makermods-robotics/metal-python-ros/blob/bccf3c82a21453a7b1784a94966456e16ff0f008/metal_sdk/native/config/motor_config.cpp).
