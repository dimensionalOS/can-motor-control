//! Damiao USB-CAN serial bridge transport.
//!
//! The Damiao USB2CAN bridge (USB ID 2e88:4603, sold with Seeed reBot arms and
//! other Damiao kits) enumerates as a CDC-ACM serial port instead of a
//! SocketCAN or gs_usb device. It wraps each classical CAN frame in a fixed
//! serial packet:
//!
//! - host to bridge, 30 bytes: `55 AA 1E 03`, send count (u32 LE), send
//!   interval (u32 LE), ID type, CAN ID (u32 LE), frame type, DLC, two
//!   reserved bytes, eight data bytes, one unused check byte.
//! - bridge to host, 16 bytes: `AA 11 08`, CAN ID (u32 LE), eight data bytes,
//!   `55`.
//!
//! The bridge carries eight data bytes and standard 11-bit IDs only, so this
//! transport rejects extended IDs and shorter or FD payloads before any IO.
//! The CAN bitrate is configured on the bridge itself, not over this protocol.

use std::ffi::CString;
use std::io;
use std::mem::MaybeUninit;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd, RawFd};
use std::os::unix::ffi::OsStrExt;
use std::path::Path;
use std::thread;
use std::time::{Duration, Instant};

use motor_codec::{BusCapabilities, CanFrame, FrameFlags};

use super::{CanBus, TransportError};

const TX_PACKET_LEN: usize = 30;
const TX_HEADER: [u8; 4] = [0x55, 0xAA, 0x1E, 0x03];
const RX_PACKET_LEN: usize = 16;
const RX_HEADER: [u8; 3] = [0xAA, 0x11, 0x08];
const RX_TRAILER: u8 = 0x55;
const PAYLOAD_LEN: usize = 8;
const MAX_STANDARD_ID: u32 = 0x7FF;
const SEND_RETRY_BUDGET: u32 = 64;
const SEND_RETRY_SLEEP: Duration = Duration::from_micros(100);

/// Default minimum gap between two packets written to the bridge.
///
/// The bridge does not acknowledge writes, and back-to-back packets can
/// overrun its CAN transmit queue and drop frames silently.
pub const DEFAULT_DM_SERIAL_FRAME_GAP: Duration = Duration::from_micros(500);

/// Encode one classical CAN frame as a bridge transmit packet.
pub fn encode_dm_serial_packet(frame: &CanFrame) -> Result<[u8; TX_PACKET_LEN], TransportError> {
    if frame.is_fd() {
        return Err(TransportError::FdFrameOnNonFdBus);
    }
    if frame.is_extended() {
        return Err(TransportError::ExtendedIdNotSupported);
    }
    if frame.flags.contains(FrameFlags::REMOTE_REQUEST) {
        return Err(TransportError::InvalidConfiguration(
            "the Damiao serial bridge does not send remote frames".to_string(),
        ));
    }
    if frame.id > MAX_STANDARD_ID {
        return Err(TransportError::InvalidConfiguration(format!(
            "CAN ID {:#x} is not an 11-bit standard ID",
            frame.id
        )));
    }
    if frame.len as usize != PAYLOAD_LEN {
        return Err(TransportError::InvalidConfiguration(format!(
            "the Damiao serial bridge sends eight data bytes, got {}",
            frame.len
        )));
    }
    let mut packet = [0u8; TX_PACKET_LEN];
    packet[..4].copy_from_slice(&TX_HEADER);
    // Send once; the interval field only matters for repeated sends.
    packet[4..8].copy_from_slice(&1u32.to_le_bytes());
    packet[8..12].copy_from_slice(&10u32.to_le_bytes());
    // packet[12] = 0: standard ID. packet[17] = 0: data frame.
    packet[13..17].copy_from_slice(&frame.id.to_le_bytes());
    packet[18] = PAYLOAD_LEN as u8;
    packet[21..29].copy_from_slice(frame.payload());
    Ok(packet)
}

/// Incremental parser for bridge receive packets.
///
/// Serial reads may split or merge packets, and the bridge may emit bytes that
/// are not CAN frames. The parser keeps a partial packet across reads and
/// resynchronizes one byte at a time after anything it does not recognize.
#[derive(Default)]
pub struct DmSerialParser {
    buffer: Vec<u8>,
}

impl DmSerialParser {
    /// Append received bytes and return every complete CAN frame they finish.
    pub fn feed(&mut self, data: &[u8]) -> Vec<CanFrame> {
        self.buffer.extend_from_slice(data);
        let mut frames = Vec::new();
        let mut start = 0;
        while self.buffer.len() - start >= RX_PACKET_LEN {
            let packet = &self.buffer[start..start + RX_PACKET_LEN];
            if packet[..3] != RX_HEADER || packet[RX_PACKET_LEN - 1] != RX_TRAILER {
                start += 1;
                continue;
            }
            let id = u32::from_le_bytes([packet[3], packet[4], packet[5], packet[6]]);
            if id <= MAX_STANDARD_ID {
                frames.push(CanFrame::classical(id, &packet[7..15]).expect("eight-byte payload"));
            }
            start += RX_PACKET_LEN;
        }
        self.buffer.drain(..start);
        frames
    }

    /// Bytes held from an incomplete packet.
    pub fn pending_len(&self) -> usize {
        self.buffer.len()
    }
}

/// CAN bus behind a Damiao USB-CAN serial bridge.
pub struct DmSerialBus {
    name: String,
    fd: OwnedFd,
    parser: DmSerialParser,
    frame_gap: Duration,
    next_write_at: Instant,
}

impl DmSerialBus {
    /// Open the bridge's serial device, for example `/dev/ttyACM0`.
    ///
    /// The port is opened exclusively in raw mode. `frame_gap` is the minimum
    /// time between two packets written to the bridge.
    pub fn open(path: impl AsRef<Path>, frame_gap: Duration) -> Result<Self, TransportError> {
        let path = path.as_ref();
        let name = path.display().to_string();
        let cpath = CString::new(path.as_os_str().as_bytes())
            .map_err(|_| TransportError::InterfaceNotFound(name.clone()))?;
        // SAFETY: open() with a valid NUL-terminated path; returns -1 on error.
        let raw_fd = unsafe {
            libc::open(
                cpath.as_ptr(),
                libc::O_RDWR | libc::O_NOCTTY | libc::O_NONBLOCK | libc::O_CLOEXEC,
            )
        };
        if raw_fd < 0 {
            return Err(map_open_error(io::Error::last_os_error(), &name));
        }
        // SAFETY: raw_fd is a freshly opened fd; OwnedFd takes exclusive ownership.
        let fd = unsafe { OwnedFd::from_raw_fd(raw_fd) };

        // A second writer would interleave packets and corrupt both streams.
        // SAFETY: flock on an owned fd.
        if unsafe { libc::flock(raw_fd, libc::LOCK_EX | libc::LOCK_NB) } < 0 {
            let err = io::Error::last_os_error();
            if err.raw_os_error() == Some(libc::EWOULDBLOCK) {
                return Err(TransportError::InvalidConfiguration(format!(
                    "{name} is already open in another process"
                )));
            }
            return Err(TransportError::Io(err));
        }
        configure_raw(raw_fd)?;
        // Drop bytes queued before this process owned the port.
        // SAFETY: tcflush on an owned tty fd.
        if unsafe { libc::tcflush(raw_fd, libc::TCIOFLUSH) } < 0 {
            return Err(TransportError::Io(io::Error::last_os_error()));
        }
        Ok(Self {
            name,
            fd,
            parser: DmSerialParser::default(),
            frame_gap,
            next_write_at: Instant::now(),
        })
    }

    fn write_packet(&mut self, packet: &[u8]) -> Result<(), TransportError> {
        let raw = self.fd.as_raw_fd();
        let mut written = 0;
        let mut retries = 0u32;
        while written < packet.len() {
            let rest = &packet[written..];
            // SAFETY: write() to an owned fd with a valid byte slice.
            let n = unsafe { libc::write(raw, rest.as_ptr() as *const libc::c_void, rest.len()) };
            if n >= 0 {
                written += n as usize;
                continue;
            }
            let err = io::Error::last_os_error();
            match err.raw_os_error() {
                Some(libc::EAGAIN) => {
                    retries += 1;
                    if retries >= SEND_RETRY_BUDGET {
                        return Err(TransportError::SendBufferFull);
                    }
                    thread::sleep(SEND_RETRY_SLEEP);
                }
                Some(libc::EINTR) => {}
                _ => return Err(TransportError::Io(err)),
            }
        }
        Ok(())
    }
}

fn configure_raw(fd: RawFd) -> Result<(), TransportError> {
    let mut termios = MaybeUninit::<libc::termios>::uninit();
    // SAFETY: tcgetattr fills the termios struct on success.
    if unsafe { libc::tcgetattr(fd, termios.as_mut_ptr()) } < 0 {
        return Err(TransportError::Io(io::Error::last_os_error()));
    }
    // SAFETY: initialized by the successful tcgetattr above.
    let mut termios = unsafe { termios.assume_init() };
    // SAFETY: cfmakeraw only edits the struct.
    unsafe { libc::cfmakeraw(&mut termios) };
    termios.c_cflag |= libc::CLOCAL | libc::CREAD;
    // CDC-ACM ignores the line rate; set the vendor's 921600 where the
    // platform defines it so a UART-attached bridge also works.
    #[cfg(target_os = "linux")]
    // SAFETY: cfsetspeed only edits the struct.
    unsafe {
        libc::cfsetspeed(&mut termios, libc::B921600);
    }
    // SAFETY: tcsetattr with a valid termios struct.
    if unsafe { libc::tcsetattr(fd, libc::TCSANOW, &termios) } < 0 {
        return Err(TransportError::Io(io::Error::last_os_error()));
    }
    Ok(())
}

fn map_open_error(e: io::Error, name: &str) -> TransportError {
    match e.raw_os_error() {
        Some(libc::EACCES) | Some(libc::EPERM) => TransportError::PermissionDenied,
        Some(libc::ENOENT) | Some(libc::ENODEV) | Some(libc::ENXIO) => {
            TransportError::InterfaceNotFound(name.to_string())
        }
        _ => TransportError::Io(e),
    }
}

impl CanBus for DmSerialBus {
    fn name(&self) -> &str {
        &self.name
    }

    fn capabilities(&self) -> BusCapabilities {
        BusCapabilities::classical()
    }

    fn send(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        let packet = encode_dm_serial_packet(frame)?;
        let now = Instant::now();
        if now < self.next_write_at {
            thread::sleep(self.next_write_at - now);
        }
        self.write_packet(&packet)?;
        self.next_write_at = Instant::now() + self.frame_gap;
        Ok(())
    }

    fn drain_inbound_nonblocking(&mut self) -> Result<Vec<CanFrame>, TransportError> {
        let raw = self.fd.as_raw_fd();
        let mut frames = Vec::new();
        let mut buf = [0u8; 1024];
        loop {
            // SAFETY: read() into an owned buffer of the given length.
            let n = unsafe { libc::read(raw, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
            if n > 0 {
                frames.extend(self.parser.feed(&buf[..n as usize]));
                continue;
            }
            if n == 0 {
                // A zero-byte read on a non-blocking tty means the device hung up.
                return Err(TransportError::Io(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    format!("{} closed", self.name),
                )));
            }
            let err = io::Error::last_os_error();
            match err.raw_os_error() {
                Some(libc::EAGAIN) => return Ok(frames),
                Some(libc::EINTR) => continue,
                _ => return Err(TransportError::Io(err)),
            }
        }
    }

    fn raw_fd(&self) -> Option<RawFd> {
        Some(self.fd.as_raw_fd())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rx_packet(id: u32, data: [u8; 8]) -> Vec<u8> {
        let mut packet = RX_HEADER.to_vec();
        packet.extend_from_slice(&id.to_le_bytes());
        packet.extend_from_slice(&data);
        packet.push(RX_TRAILER);
        packet
    }

    #[test]
    fn encodes_standard_frame() {
        let frame = CanFrame::classical(0x101, &[1, 2, 3, 4, 5, 6, 7, 8]).unwrap();
        let packet = encode_dm_serial_packet(&frame).unwrap();
        assert_eq!(&packet[..4], &TX_HEADER);
        assert_eq!(&packet[4..8], &1u32.to_le_bytes());
        assert_eq!(&packet[8..12], &10u32.to_le_bytes());
        assert_eq!(packet[12], 0);
        assert_eq!(&packet[13..17], &0x101u32.to_le_bytes());
        assert_eq!(packet[17], 0);
        assert_eq!(packet[18], 8);
        assert_eq!(&packet[21..29], &[1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(packet[29], 0);
    }

    #[test]
    fn rejects_frames_the_bridge_cannot_carry() {
        let short = CanFrame::classical(0x01, &[0xFF; 4]).unwrap();
        assert!(matches!(
            encode_dm_serial_packet(&short),
            Err(TransportError::InvalidConfiguration(_))
        ));
        let extended = CanFrame::classical_extended(0x01, &[0; 8]).unwrap();
        assert!(matches!(
            encode_dm_serial_packet(&extended),
            Err(TransportError::ExtendedIdNotSupported)
        ));
        let fd = CanFrame::fd(0x01, &[0; 8]).unwrap();
        assert!(matches!(
            encode_dm_serial_packet(&fd),
            Err(TransportError::FdFrameOnNonFdBus)
        ));
    }

    #[test]
    fn parses_split_and_merged_packets() {
        let mut parser = DmSerialParser::default();
        let mut bytes = rx_packet(0x11, [1; 8]);
        bytes.extend(rx_packet(0x12, [2; 8]));
        assert!(parser.feed(&bytes[..10]).is_empty());
        let frames = parser.feed(&bytes[10..]);
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0].id, 0x11);
        assert_eq!(frames[0].payload(), &[1; 8]);
        assert_eq!(frames[1].id, 0x12);
        assert_eq!(parser.pending_len(), 0);
    }

    #[test]
    fn resynchronizes_after_noise() {
        let mut parser = DmSerialParser::default();
        let mut bytes = vec![0x00, 0xAA, 0x55, 0xAA, 0x11];
        bytes.extend(rx_packet(0x17, [7; 8]));
        let frames = parser.feed(&bytes);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].id, 0x17);
        assert_eq!(parser.pending_len(), 0);
    }

    #[test]
    fn drops_ids_outside_the_standard_range() {
        let mut parser = DmSerialParser::default();
        assert!(parser.feed(&rx_packet(0x800, [0; 8])).is_empty());
        assert_eq!(parser.pending_len(), 0);
    }

    #[test]
    fn exchanges_packets_over_a_pseudo_terminal() {
        // SAFETY: standard pseudo-terminal setup on a fresh master fd.
        let master = unsafe { libc::posix_openpt(libc::O_RDWR | libc::O_NOCTTY) };
        assert!(master >= 0);
        let master = unsafe { OwnedFd::from_raw_fd(master) };
        unsafe {
            assert_eq!(libc::grantpt(master.as_raw_fd()), 0);
            assert_eq!(libc::unlockpt(master.as_raw_fd()), 0);
        }
        let slave = unsafe { std::ffi::CStr::from_ptr(libc::ptsname(master.as_raw_fd())) }
            .to_str()
            .unwrap()
            .to_string();

        let mut bus = DmSerialBus::open(&slave, Duration::ZERO).unwrap();
        let frame = CanFrame::classical(0x7FF, &[1, 0, 0xCC, 0, 0, 0, 0, 0]).unwrap();
        bus.send(&frame).unwrap();
        let mut sent = [0u8; TX_PACKET_LEN];
        // SAFETY: read() into a local buffer from the owned master fd.
        let n = unsafe {
            libc::read(
                master.as_raw_fd(),
                sent.as_mut_ptr() as *mut libc::c_void,
                TX_PACKET_LEN,
            )
        };
        assert_eq!(n as usize, TX_PACKET_LEN);
        assert_eq!(sent, encode_dm_serial_packet(&frame).unwrap());

        let reply = rx_packet(0x11, [0x11, 0x80, 0x00, 0x80, 0x08, 0x00, 30, 31]);
        // SAFETY: write() of a local buffer to the owned master fd.
        let n = unsafe {
            libc::write(
                master.as_raw_fd(),
                reply.as_ptr() as *const libc::c_void,
                reply.len(),
            )
        };
        assert_eq!(n as usize, reply.len());
        let deadline = Instant::now() + Duration::from_secs(1);
        let mut frames = Vec::new();
        while frames.is_empty() && Instant::now() < deadline {
            frames = bus.drain_inbound_nonblocking().unwrap();
        }
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].id, 0x11);
        assert_eq!(frames[0].payload()[6], 30);
    }
}
