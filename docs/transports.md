# CAN transports

This page describes the transports implemented in this branch. Availability in a
released wheel depends on whether that release includes the implementation.
The transport carries CAN frames between the host and adapter. `DamiaoCodec`
encodes motor commands separately, so the same codec works with each hardware
transport below.

## Platform and build support

| Public transport | Rust compile gate | Python export | Frame support |
| --- | --- | --- | --- |
| `SocketCanBus` | `target_os = "linux"` | Linux | Classical CAN, optional CAN-FD, standard and extended IDs |
| `GsUsbBus` | `target_os = "macos"` | macOS | Classical CAN, standard and extended IDs, channel 0 |
| `DmSerialBus` | `unix` | Linux and macOS | Standard 11-bit data frames with exactly 8 payload bytes |
| `MockCanBus` | No transport-specific gate | Linux and macOS builds | In-memory classical or FD loopback |

Linux and macOS are the supported host platforms. The `unix` gate on
`DmSerialBus` is not a claim of tested support on other Unix systems. Windows
is not supported by the control crate, which uses Unix file-descriptor APIs.
The standalone codec crates have a separate platform scope.

There are no Cargo feature flags that select hardware transports. Target gates
select them at compile time. The crate declares `default = ["mock"]`, but
`MockCanBus` is exported unconditionally, including with `--no-default-features`.
Rust gs_usb uses the macOS-only `nusb` dependency and IOKit. Python wheels use
these Rust implementations and do not require python-can, PyUSB, Python gs_usb,
pyserial, or Homebrew libusb.

Build the Rust crate from a checkout with Rust 1.85 or later:

```bash
cargo build -p can-motor-control --locked
```

Build and install the local Python extension with Python 3.10 or later and
[uv](https://docs.astral.sh/uv/):

```bash
uv sync --locked --reinstall-package can-motor-control
```

The root development environment builds the local extension through maturin
and installs its Python dependencies, including NumPy. Building from this branch
provides `DmSerialBus`; installing an earlier wheel does not add that API.

## SocketCAN on Linux

`SocketCanBus` opens an existing Linux CAN network interface such as `can0`.
The kernel driver handles the adapter protocol. This includes USB adapters with
compatible gs_usb firmware and other adapters supported by a SocketCAN driver.
A virtual `vcan` interface also works for host-side checks without hardware.

Python constructor:

```python
from can_motor_control import SocketCanBus

bus = SocketCanBus("can0", fd=False)
```

Rust constructor:

```rust,no_run
use can_motor_control::SocketCanBus;

let bus = SocketCanBus::open("can0", false)?;
# Ok::<(), can_motor_control::TransportError>(())
```

The interface must already exist, be up, and have the correct CAN bitrate.
Network setup requires the appropriate administrative privileges. Opening the
raw CAN socket requires the host's raw-socket permissions. The constructor does
not configure the link bitrate. `fd=True` in Python or `true` in Rust enables
CAN-FD on the socket; the interface and adapter must also support FD. Configure
nominal `bitrate` and FD data-phase `dbitrate` on the Linux link. There is no
serial baudrate argument.

See [SocketCAN setup](socketcan-setup.md) for link commands and [CAN-FD](can-fd.md)
for frame flags and codec behavior. A working virtual interface does not verify
motor communication.

## Native gs_usb on macOS

`GsUsbBus` requires an adapter whose active firmware implements the gs_usb USB
protocol, such as compatible candleLight firmware. A USB connector or matching
VID/PID alone does not establish compatibility. The transport uses vendor
control requests to initialize CAN and USB bulk transfers for CAN frames.

Python constructor, with keyword-only arguments:

```python
from can_motor_control import GsUsbBus

bus = GsUsbBus(
	vendor_id=0x1D50,
	product_id=0x606F,
	index=0,
	bitrate=1_000_000,
	initialization_timeout=5.0,
)
```

Rust constructor:

```rust,no_run
use can_motor_control::{GsUsbBus, GsUsbConfig};

let mut config = GsUsbConfig::new(0x1d50, 0x606f);
config.index = Some(0);
config.bitrate = 1_000_000;
let bus = GsUsbBus::open(config)?;
# Ok::<(), can_motor_control::TransportError>(())
```

`serial_number` can replace `index`; they cannot be supplied together.
The default index is zero, bitrate is 1,000,000 bit/s, and initialization timeout
is five seconds. Python exposes `list_gs_usb_devices(vendor_id=..., product_id=...)`;
Rust exposes `list_gs_usb_devices(vendor_id, product_id)`. The device's USB
interface must be accessible and available for this process to claim.

`bitrate` sets the CAN nominal bitrate through gs_usb. There is no tty or serial
baudrate. This implementation supports exactly one CAN channel, channel 0, in
normal classical-CAN mode. CAN-FD, listen-only mode, hardware timestamps, and
automatic reconnection are not implemented. These are backend limits, even when
the adapter supports more. Linux uses `SocketCanBus` with the kernel driver
instead of this native macOS class. See [Native macOS gs_usb](gs-usb-macos.md)
for selection, queues, shutdown, counters, and diagnostics.

## Damiao USB2CAN serial bridge

`DmSerialBus` opens the Damiao USB2CAN bridge's CDC-ACM serial port. The tested
bridge has USB ID `2e88:4603` and ships with the Seeed reBot B601-DM.
[Seeed's B601-DM documentation](https://wiki.seeedstudio.com/rebot_arm_b601_dm_ros2_integration/)
identifies the supplied adapter as a USB2CAN serial bridge, with `/dev/ttyACM0`
as the default Linux port. macOS paths typically start with `/dev/cu.usbmodem`.
This device's Damiao serial protocol is different from gs_usb.

Python constructor:

```python
from can_motor_control import DmSerialBus

bus = DmSerialBus("/dev/ttyACM0", frame_gap=0.0)
```

Rust constructor:

```rust,no_run
use can_motor_control::DmSerialBus;
use std::time::Duration;

let bus = DmSerialBus::open("/dev/ttyACM0", Duration::ZERO)?;
# Ok::<(), can_motor_control::TransportError>(())
```

Python `frame_gap` defaults to `None`, which selects zero seconds. Rust takes a
`Duration`; `DEFAULT_DM_SERIAL_FRAME_GAP` is zero. The maximum gap is one second.
It controls host packet spacing, not CAN bitrate or serial baudrate.

The port must exist and the user must have read and write permission, for example
through the Linux `dialout` or `uucp` group. The backend takes an advisory lock
and requests tty exclusive access. Close other software using the port first.
The backend sets raw tty mode. On Linux it sets the tty line rate to 921600;
on macOS it does not set a new line rate. There is no public baudrate option.
The implementation assumes the CDC-ACM device ignores the line rate. That tty
setting does not set the CAN bitrate.

The backend does not configure CAN bitrate. Configure the bridge separately to
match the motors before opening it. The official
[USB2CAN protocol table](https://github.com/dmBots/usb-2-can/blob/master/communication-protocol/USB%20%E8%BD%ACCAN%20%E5%B8%A7%E6%A0%BC%E5%BC%8F.xlsx)
defines a CAN-bitrate command, extended IDs, remote frames, and a payload-length
field. The current backend implements only standard 11-bit data frames with
exactly eight bytes. It rejects other lengths, extended IDs, remote frames, and
CAN-FD before I/O. These restrictions describe this implementation, not all
capabilities of the bridge hardware or protocol.

Host packets are 30 bytes beginning with `55 AA 1E 03`. This backend accepts
16-byte replies beginning with `AA 11 08` and ending with `55`, and retains
partial receive packets across reads. In the reported B601-DM experiment,
seven packets in one tty write lost all replies; seven separate writes lost
none. The implementation therefore starts each packet in a separate write and
completes partial writes before starting another packet. A tty write does not
guarantee a USB-transfer boundary. The experiment does not establish how the
firmware parses USB transfers. A failure after a partial packet faults later
sends rather than continuing an incomplete stream.

Linux hardware operation is reported in [PR #7](https://github.com/dimensionalOS/can-motor-control/pull/7).
macOS hardware has not been tested. This is the USB2CAN serial bridge, not an
assertion of support for every Damiao USB2CANFD model or firmware. Damiao also
publishes [gs_usb firmware for USB2CANFD](https://github.com/dmBots/usb2canfd/tree/main/firmware/socketcan/gsusb-driverless-firmware);
that does not establish that the tested USB2CAN box can run it.

## In-memory mock

`MockCanBus` uses no adapter, kernel CAN interface, bitrate, or serial baudrate.
Its default constructor loops classical frames back to the same bus. Its FD
constructor advertises FD capabilities and also accepts FD frames.

```python
from can_motor_control import MockCanBus

bus = MockCanBus("test")
fd_bus = MockCanBus.new_fd("test-fd")
```

```rust
use can_motor_control::MockCanBus;

let bus = MockCanBus::new("test");
let fd_bus = MockCanBus::new_fd("test-fd");
let (a, b) = MockCanBus::pair("a", "b");
```

Rust also exposes `with_capabilities`, frame injection, and sent-frame
inspection. `pair` is a Rust-only classical loopback between two mocks.
A mock does not simulate motor feedback unless the test supplies it, and does
not validate hardware timing or firmware compatibility.

## Robot integration and configuration

Python `RobotBuilder.add_bus(name, transport, codec)` accepts the native
transport classes available on that platform. Adding a bus consumes its handle
once. It cannot be reused in a second robot. Rust
`RobotBuilder::add_bus(name, Box<dyn CanBus>, Box<dyn MotorCodec>)` accepts these
transports and custom implementations of the `CanBus` trait.

The TOML loader supports `kind = "socketcan"` on Linux and `kind = "gs_usb"` on
macOS. `DmSerialBus` and `MockCanBus` require a code-built robot; there is no
TOML kind for either. Existing bring-up example flags select SocketCAN on Linux
or gs_usb on macOS; they do not select the Damiao serial bridge.

There is no Python `BusABC` adapter in this library. python-can's `serial`,
`slcan`, and `seeedstudio` backends use different wire formats and are not
aliases for `DmSerialBus`. Supporting another serial protocol requires a
matching transport implementation or a bridge that translates it.
