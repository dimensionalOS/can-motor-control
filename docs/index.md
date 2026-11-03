# can_motor_control

Rust control library for Damiao-family CAN motors, with first-class Python
bindings.

The Python package `can_motor_control` is a thin [PyO3](https://pyo3.rs) layer over a
Rust core, so it installs as a single wheel and runs in realtime control loops
without GIL contention.

## Where to go

- **[CAN transports](transports.md)**: APIs, platform support, adapter protocols,
  and bitrate configuration.

- **[SocketCAN setup](socketcan-setup.md)** — bring up a Linux CAN interface
  for the first time.
- **[CAN-FD](can-fd.md)** — notes on classical CAN vs CAN-FD.
- **[Multi-vendor](multi-vendor.md)** — how the vendor-agnostic codec seam works.
- **[Python API](reference.md)** — the full `can_motor_control` reference.
- **[Rust API](rust.md)** — pointer to the rustdoc for the Rust crates.

## Quick taste

```python
import numpy as np
from can_motor_control import Robot

# Connect + enable on entry, disable on exit.
with Robot.from_config("configs/single_arm.toml") as robot:
    arm = robot["arm"]
    for _ in range(1000):
        # One row [kp, kd, q, dq, tau] per motor.
        cmds = np.zeros((len(arm), 5))
        arm.mit_control(cmds)
        robot.tick(per_bus_deadline_us=2000)
        print(arm.positions())
```

## Status

v0.1.0 is in development. Implemented transports are Linux SocketCAN,
native macOS gs_usb, the Damiao USB2CAN serial bridge on Linux and macOS,
and an in-memory mock. SocketCAN and the mock support CAN-FD. The gs_usb and
Damiao serial backends support classical CAN. See [CAN transports](transports.md)
for the exact limits. Additional motor vendors require their own codecs.
