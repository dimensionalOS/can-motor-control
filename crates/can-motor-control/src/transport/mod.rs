//! CAN transport abstraction.

use std::io;
use std::os::fd::RawFd;

use motor_codec::{BusCapabilities, CanFrame, FrameError};
use thiserror::Error;

mod gs_usb;
mod mock;
mod poller;
#[cfg(target_os = "linux")]
mod socketcan;

#[cfg(target_os = "macos")]
pub use gs_usb::GsUsbStatistics;
#[cfg(target_os = "macos")]
pub use gs_usb::{list_gs_usb_devices, GsUsbBus, GsUsbConfig, GsUsbDeviceInfo};
pub use mock::{MockCanBus, MockRecordedCall};
pub use poller::BusPoller;
#[cfg(target_os = "linux")]
pub use socketcan::SocketCanBus;

/// The contract every CAN transport must satisfy.
///
/// The trait is intentionally object-safe so that `Box<dyn CanBus>` is a valid
/// field type in [`crate::Bus`] / `Robot`. Implementations must be `Send`
/// because they may move between threads (background IO thread, future async
/// adapter).
pub trait CanBus: Send {
    /// Human-readable interface name (`"vcan0"`, `"can0"`, `"mock-0"`).
    fn name(&self) -> &str;

    /// Runtime capabilities of this bus (FD support, max payload length).
    fn capabilities(&self) -> BusCapabilities;

    /// Send a single frame. Must not call `read` or `poll` internally.
    fn send(&mut self, frame: &CanFrame) -> Result<(), TransportError>;

    /// Drain every frame currently in the receive queue and return them in
    /// arrival order. Returns `Ok(vec![])` immediately if the queue is empty.
    fn drain_inbound_nonblocking(&mut self) -> Result<Vec<CanFrame>, TransportError>;

    /// Pollable file descriptor for `poll(2)`-based multiplexing. Returns
    /// `None` for transports that have no single pollable fd. The robot drains
    /// those transports from memory on every [`crate::Robot::tick`] call.
    fn raw_fd(&self) -> Option<RawFd>;
}

/// Errors returned by transport operations.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum TransportError {
    /// The requested transport setup is contradictory or unsupported.
    #[error("invalid transport configuration: {0}")]
    InvalidConfiguration(String),

    /// The named interface does not exist on this host.
    #[error("interface not found: {0}")]
    InterfaceNotFound(String),

    /// The caller lacks permission to open or operate the socket
    /// (typically: missing `CAP_NET_RAW`).
    #[error("permission denied opening CAN socket")]
    PermissionDenied,

    /// Generic IO error from the underlying syscall.
    #[error("transport IO error: {0}")]
    Io(#[from] io::Error),

    /// The kernel's send buffer is full and remained full after the bounded
    /// retry budget was exhausted.
    #[error("send buffer full after retries")]
    SendBufferFull,

    /// A frame the codec produced was rejected before any IO was issued.
    #[error("frame error: {0}")]
    FrameError(#[from] FrameError),

    /// A CAN-FD frame was passed to a classical-only bus.
    #[error("CAN-FD frame on non-FD bus")]
    FdFrameOnNonFdBus,

    /// The frame's payload length exceeds the bus's max payload length.
    #[error("payload length {len} exceeds bus max {max}")]
    PayloadExceedsBusCapacity {
        /// Payload length the caller attempted to send.
        len: u8,
        /// Bus's max payload length.
        max: u8,
    },

    /// The transport cannot send 29-bit extended IDs.
    #[error("extended (29-bit) CAN IDs not supported by this transport")]
    ExtendedIdNotSupported,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trait_is_object_safe() {
        // Just needs to compile.
        fn _accept(_b: Box<dyn CanBus>) {}
    }
}
