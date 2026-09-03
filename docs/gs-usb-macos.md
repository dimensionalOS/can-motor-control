# Native macOS gs_usb

macOS builds expose `GsUsbBus` instead of `SocketCanBus`. The transport uses
IOKit through Rust `nusb`; installing the wheel does not require libusb,
PyUSB, or the Python `gs_usb` package.

The first version supports channel 0 in normal classical-CAN mode. It supports
standard and extended identifiers. It does not support CAN-FD, listen-only
mode, hardware timestamps, multiple channels, or automatic reconnection.

## Open an adapter

Python requires keyword arguments:

```python
from can_motor_control import GsUsbBus

bus = GsUsbBus(
    vendor_id=0x1D50,
    product_id=0x606F,
    serial_number="A1B2C3",  # or index=0, but not both
    bitrate=1_000_000,
    initialization_timeout=5.0,
)
```

Rust uses `GsUsbConfig`:

```rust,no_run
use can_motor_control::{GsUsbBus, GsUsbConfig};

let mut config = GsUsbConfig::new(0x1d50, 0x606f);
config.serial_number = Some("A1B2C3".into());
config.bitrate = 1_000_000;
let bus = GsUsbBus::open(config)?;
# Ok::<(), can_motor_control::TransportError>(())
```

List matching adapters before opening one:

```python
from can_motor_control import list_gs_usb_devices

for device in list_gs_usb_devices(vendor_id=0x1D50, product_id=0x606F):
    print(device.index, device.serial_number)
```

Choose either `serial_number` or `index`. If you omit both, the transport uses
index zero. USB enumeration order can change after reconnects and reboots, so
use a serial number for a fixed deployment. Duplicate serial numbers fail as
ambiguous instead of selecting an arbitrary adapter.

Construction waits until the adapter is ready. It discovers endpoints from
USB descriptors, negotiates little-endian host format, reads channel and
timing capabilities, resets channel 0, calculates nominal timing from the
reported CAN clock, starts the channel, discards stale input until the bus is
quiet, and primes queued reads. The default five-second deadline covers
temporary re-enumeration after reset.

## Send, receive, and failures

`send()` returns after a bounded in-memory queue accepts the frame. USB and CAN
completion happen on the worker thread. A full queue returns
`SendBufferFull`; the transport never evicts an older outbound frame.

`Robot.tick()` drains the transport's in-memory RX queue on every tick. The
queue holds 8,192 frames. On overflow it removes the oldest frame, retains the
newest frame, and increments `rx_dropped`.

Read the shared counters in Python after or before adding the bus to a builder:

```python
print(bus.rx_received, bus.rx_dropped)
print(bus.tx_accepted, bus.tx_completed)
```

Adding a bus to `RobotBuilder` consumes its transport handle exactly once. The
counter properties remain readable on the consumed Python wrapper.

A disconnect, malformed transfer, invalid echo, or failed endpoint recovery
faults the worker. The next send or drain reports `TransportError`; the worker
does not reconnect. Drop the owning bus or robot and construct a new bus.

Dropping the owner closes TX, cancels and collects queued USB transfers, resets
channel 0, releases the interface, and joins the worker. There is no separate
`close()` method.

## Diagnostics

- `invalid transport configuration`: fix contradictory selectors, an
  impossible bitrate, unsupported channel data, or an ambiguous descriptor.
- `interface not found`: verify the VID/PID and selector. Increase
  `initialization_timeout` only when the device is re-enumerating slowly.
- `permission denied`: close software that already owns the interface and
  check macOS USB access policy.
- `transport IO error`: disconnect or protocol failure occurred after opening.
  Drop the bus, reconnect the adapter if needed, and construct a new bus.
- Rising `rx_dropped`: the consumer is not draining at the required rate.
  Record the workload and stop the run rather than accepting stale feedback.

The Rust bring-up examples accept `--interface VID:PID` on macOS, for example
`--interface 1d50:606f`. Linux continues to interpret `--interface` as a
SocketCAN interface name.
