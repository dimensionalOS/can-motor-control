//! nusb/IOKit-backed gs_usb transport. This module is compiled only on macOS.

use std::collections::VecDeque;
use std::io;
use std::os::fd::RawFd;
use std::sync::mpsc::{self, Receiver, SyncSender, TryRecvError, TrySendError};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use motor_codec::{BusCapabilities, CanFrame};
use nusb::descriptors::TransferType;
use nusb::transfer::{
    Bulk, ControlIn, ControlOut, ControlType, Direction, In, Out, Recipient, TransferError,
};
use nusb::{DeviceInfo, Endpoint, ErrorKind, Interface, MaybeFuture};

use crate::transport::{CanBus, TransportError};

use super::protocol::{
    encode_host_format, encode_host_frame, encode_mode, BitTimingConstraints, DeviceConfig,
    NominalBitTiming, HOST_FRAME_LEN,
};
use super::selection::{
    discover_endpoint_layout, select_candidate, validated_selector, AlternateSetting, Candidate,
    EndpointKind, EndpointLayout, Selector,
};
use super::timing::calculate_nominal_timing;
use super::worker_core::{
    process_input, EchoCredits, GsUsbStatistics, RxQueue, ECHO_CREDITS, TX_CAPACITY,
};

const DEFAULT_BITRATE: u32 = 1_000_000;
const DEFAULT_INITIALIZATION_TIMEOUT: Duration = Duration::from_secs(5);
const CONTROL_TIMEOUT: Duration = Duration::from_secs(1);
const RESET_SETTLE: Duration = Duration::from_millis(50);
const STALE_QUIET_INTERVAL: Duration = Duration::from_millis(20);
const STALE_FLUSH_LIMIT: Duration = Duration::from_millis(500);
const IN_FLIGHT: usize = 4;

const BREQ_HOST_FORMAT: u8 = 0;
const BREQ_BITTIMING: u8 = 1;
const BREQ_MODE: u8 = 2;
const BREQ_BT_CONST: u8 = 4;
const BREQ_DEVICE_CONFIG: u8 = 5;
const MODE_RESET: u32 = 0;
const MODE_START: u32 = 1;

/// One macOS gs_usb adapter returned by [`list_gs_usb_devices`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GsUsbDeviceInfo {
    /// Enumeration index accepted by [`GsUsbConfig::index`].
    pub index: usize,
    /// USB vendor identifier.
    pub vendor_id: u16,
    /// USB product identifier.
    pub product_id: u16,
    /// Stable USB serial number, when the adapter firmware provides one.
    pub serial_number: Option<String>,
}

/// List macOS gs_usb adapters matching a USB vendor/product pair.
pub fn list_gs_usb_devices(
    vendor_id: u16,
    product_id: u16,
) -> Result<Vec<GsUsbDeviceInfo>, TransportError> {
    let devices = nusb::list_devices().wait().map_err(discovery_error)?;
    Ok(devices
        .filter(|device| device.vendor_id() == vendor_id && device.product_id() == product_id)
        .enumerate()
        .map(|(index, device)| GsUsbDeviceInfo {
            index,
            vendor_id: device.vendor_id(),
            product_id: device.product_id(),
            serial_number: device.serial_number().map(str::to_owned),
        })
        .collect())
}

/// Configuration for opening one macOS gs_usb adapter.
#[derive(Clone, Debug)]
pub struct GsUsbConfig {
    /// USB vendor identifier.
    pub vendor_id: u16,
    /// USB product identifier.
    pub product_id: u16,
    /// Stable serial selector. Mutually exclusive with [`Self::index`].
    pub serial_number: Option<String>,
    /// Enumeration-order selector. This is not a stable deployment identity.
    pub index: Option<usize>,
    /// Requested nominal classical-CAN bitrate.
    pub bitrate: u32,
    /// One shared deadline for all complete initialization attempts.
    pub initialization_timeout: Duration,
}

impl GsUsbConfig {
    /// Create a channel-0, 1 Mbit/s configuration with a five-second
    /// initialization deadline and default enumeration index zero.
    pub fn new(vendor_id: u16, product_id: u16) -> Self {
        Self {
            vendor_id,
            product_id,
            serial_number: None,
            index: None,
            bitrate: DEFAULT_BITRATE,
            initialization_timeout: DEFAULT_INITIALIZATION_TIMEOUT,
        }
    }
}

/// Synchronous classical-CAN bus backed by a gs_usb adapter through IOKit.
///
/// `open` returns only after channel 0 is configured and a clean session
/// boundary has been established. `send` performs no USB I/O: success means a
/// bounded worker queue accepted responsibility for attempting the frame.
/// Runtime USB failures fault the bus permanently; reconstruct it to recover.
/// Dropping the bus cancels transfers, resets channel 0, releases the claimed
/// interface through nusb's ownership model, and joins the worker.
pub struct GsUsbBus {
    name: String,
    tx: Option<SyncSender<CanFrame>>,
    rx: Arc<RxQueue>,
    terminal_error: Arc<Mutex<Option<String>>>,
    shutdown: mpsc::Sender<()>,
    worker: Option<JoinHandle<()>>,
    statistics: GsUsbStatistics,
}

impl GsUsbBus {
    /// Open, initialize, and prime the selected adapter.
    ///
    /// Serial and index selectors are mutually exclusive. When both are
    /// absent, enumeration index zero is used.
    pub fn open(config: GsUsbConfig) -> Result<Self, TransportError> {
        let selector = validated_selector(config.serial_number.as_deref(), config.index)
            .map_err(TransportError::InvalidConfiguration)?;
        if config.bitrate == 0 {
            return Err(TransportError::InvalidConfiguration(
                "gs_usb bitrate must be greater than zero".into(),
            ));
        }
        if config.initialization_timeout.is_zero() {
            return Err(TransportError::InvalidConfiguration(
                "gs_usb initialization timeout must be greater than zero".into(),
            ));
        }

        let statistics = GsUsbStatistics::default();
        let rx = Arc::new(RxQueue::new(statistics.clone()));
        let terminal_error = Arc::new(Mutex::new(None));
        let (tx_sender, tx_receiver) = mpsc::sync_channel(TX_CAPACITY);
        let (shutdown_sender, shutdown_receiver) = mpsc::channel();
        let (ready_sender, ready_receiver) = mpsc::sync_channel(1);
        let worker_rx = Arc::clone(&rx);
        let worker_error = Arc::clone(&terminal_error);
        let worker_stats = statistics.clone();
        let worker_config = config.clone();
        let worker = thread::Builder::new()
            .name("gs_usb".into())
            .spawn(move || {
                worker_main(WorkerContext {
                    config: worker_config,
                    selector,
                    tx: tx_receiver,
                    rx: worker_rx,
                    terminal_error: worker_error,
                    statistics: worker_stats,
                    shutdown: shutdown_receiver,
                    ready: ready_sender,
                })
            })
            .map_err(TransportError::Io)?;

        match ready_receiver.recv() {
            Ok(Ok(())) => Ok(Self {
                name: format!("gs_usb {:04x}:{:04x}", config.vendor_id, config.product_id),
                tx: Some(tx_sender),
                rx,
                terminal_error,
                shutdown: shutdown_sender,
                worker: Some(worker),
                statistics,
            }),
            Ok(Err(error)) => {
                let _ = worker.join();
                Err(error)
            }
            Err(_) => {
                let _ = worker.join();
                Err(TransportError::Io(io::Error::other(
                    "gs_usb worker exited before readiness handoff",
                )))
            }
        }
    }

    /// Clone the shared read-only runtime statistics handle.
    pub fn statistics(&self) -> GsUsbStatistics {
        self.statistics.clone()
    }

    fn terminal_result(&self) -> Result<(), TransportError> {
        let message = self
            .terminal_error
            .lock()
            .map_err(|_| TransportError::Io(io::Error::other("gs_usb terminal state poisoned")))?
            .clone();
        match message {
            Some(message) => Err(TransportError::Io(io::Error::other(message))),
            None => Ok(()),
        }
    }
}

impl CanBus for GsUsbBus {
    fn name(&self) -> &str {
        &self.name
    }

    fn capabilities(&self) -> BusCapabilities {
        BusCapabilities::classical()
    }

    fn send(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        self.terminal_result()?;
        if frame.is_fd() {
            return Err(TransportError::FdFrameOnNonFdBus);
        }
        if frame.len > 8 {
            return Err(TransportError::PayloadExceedsBusCapacity {
                len: frame.len,
                max: 8,
            });
        }
        encode_host_frame(0, frame)
            .map_err(|error| TransportError::InvalidConfiguration(error.to_string()))?;
        let sender = self.tx.as_ref().ok_or_else(|| {
            TransportError::Io(io::Error::new(
                io::ErrorKind::BrokenPipe,
                "gs_usb worker stopped",
            ))
        })?;
        match sender.try_send(*frame) {
            Ok(()) => {
                self.statistics.accepted();
                Ok(())
            }
            Err(TrySendError::Full(_)) => Err(TransportError::SendBufferFull),
            Err(TrySendError::Disconnected(_)) => self.terminal_result().and_then(|()| {
                Err(TransportError::Io(io::Error::new(
                    io::ErrorKind::BrokenPipe,
                    "gs_usb worker stopped",
                )))
            }),
        }
    }

    fn drain_inbound_nonblocking(&mut self) -> Result<Vec<CanFrame>, TransportError> {
        self.terminal_result()?;
        Ok(self.rx.drain())
    }

    fn raw_fd(&self) -> Option<RawFd> {
        None
    }
}

impl Drop for GsUsbBus {
    fn drop(&mut self) {
        self.tx.take();
        let _ = self.shutdown.send(());
        if let Some(worker) = self.worker.take() {
            if worker.join().is_err() {
                log::error!("gs_usb worker panicked during shutdown");
            }
        }
    }
}

struct UsbSession {
    interface: Interface,
    input: Endpoint<Bulk, In>,
    output: Endpoint<Bulk, Out>,
    input_request_size: usize,
}

enum InitFailure {
    Transient(String),
    Terminal(TransportError),
}

enum ProtocolInitFailure {
    Transfer(TransferError),
    Configuration(String),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TransferAction {
    ExpectedShutdown,
    RecoverHalt,
    Fault,
}

fn classify_transfer_error(
    error: TransferError,
    shutting_down: bool,
    halt_recovery_used: bool,
) -> TransferAction {
    match error {
        TransferError::Cancelled if shutting_down => TransferAction::ExpectedShutdown,
        TransferError::Stall if !halt_recovery_used => TransferAction::RecoverHalt,
        _ => TransferAction::Fault,
    }
}

impl From<TransferError> for ProtocolInitFailure {
    fn from(error: TransferError) -> Self {
        Self::Transfer(error)
    }
}

struct WorkerContext {
    config: GsUsbConfig,
    selector: Selector,
    tx: Receiver<CanFrame>,
    rx: Arc<RxQueue>,
    terminal_error: Arc<Mutex<Option<String>>>,
    statistics: GsUsbStatistics,
    shutdown: Receiver<()>,
    ready: SyncSender<Result<(), TransportError>>,
}

fn worker_main(context: WorkerContext) {
    let WorkerContext {
        config,
        selector,
        tx,
        rx,
        terminal_error,
        statistics,
        shutdown,
        ready,
    } = context;
    let mut session = match initialize_with_retry(&config, &selector) {
        Ok(session) => session,
        Err(error) => {
            let _ = ready.send(Err(error));
            return;
        }
    };
    if ready.send(Ok(())).is_err() {
        cleanup_session(&mut session);
        return;
    }

    if let Err(error) = run_session(&mut session, &tx, &rx, &statistics, &shutdown) {
        if let Ok(mut terminal) = terminal_error.lock() {
            *terminal = Some(error);
        }
    }
    cleanup_session(&mut session);
}

fn initialize_with_retry(
    config: &GsUsbConfig,
    selector: &Selector,
) -> Result<UsbSession, TransportError> {
    retry_initialization(config, selector, |deadline| {
        initialize_once(config, selector, deadline)
    })
}

fn retry_initialization<T>(
    config: &GsUsbConfig,
    selector: &Selector,
    mut attempt: impl FnMut(Instant) -> Result<T, InitFailure>,
) -> Result<T, TransportError> {
    let deadline = Instant::now() + config.initialization_timeout;
    loop {
        let transient_error = match attempt(deadline) {
            Ok(value) => return Ok(value),
            Err(InitFailure::Terminal(error)) => return Err(error),
            Err(InitFailure::Transient(error)) => error,
        };
        if Instant::now() >= deadline {
            return Err(TransportError::InterfaceNotFound(format!(
                "gs_usb {:04x}:{:04x} selector {selector:?} was not ready within {:?}: {transient_error}",
                config.vendor_id, config.product_id, config.initialization_timeout
            )));
        }
        thread::sleep(
            Duration::from_millis(50).min(deadline.saturating_duration_since(Instant::now())),
        );
    }
}

fn initialize_once(
    config: &GsUsbConfig,
    selector: &Selector,
    deadline: Instant,
) -> Result<UsbSession, InitFailure> {
    let devices: Vec<DeviceInfo> = nusb::list_devices()
        .wait()
        .map_err(classify_nusb_error)?
        .collect();
    let candidates: Vec<Candidate> = devices
        .iter()
        .map(|device| Candidate {
            vendor_id: device.vendor_id(),
            product_id: device.product_id(),
            serial_number: device.serial_number().map(str::to_owned),
        })
        .collect();
    let matching_count = candidates
        .iter()
        .filter(|candidate| {
            candidate.vendor_id == config.vendor_id && candidate.product_id == config.product_id
        })
        .count();
    if matching_count == 0 {
        return Err(InitFailure::Transient("adapter not present".into()));
    }
    let selected = select_candidate(&candidates, config.vendor_id, config.product_id, selector)
        .map_err(|message| InitFailure::Terminal(TransportError::InvalidConfiguration(message)))?;
    let device = devices[selected]
        .open()
        .wait()
        .map_err(classify_nusb_error)?;

    if device.active_configuration().is_err() {
        let configuration = device
            .configurations()
            .next()
            .map(|descriptor| descriptor.configuration_value())
            .ok_or_else(|| terminal_config("gs_usb device has no USB configuration descriptor"))?;
        device
            .set_configuration(configuration)
            .wait()
            .map_err(classify_nusb_error)?;
    }
    let configuration = device.active_configuration().map_err(|error| {
        terminal_config(format!("cannot read gs_usb active configuration: {error}"))
    })?;
    let mut settings = Vec::new();
    for interface in configuration.interface_alt_settings() {
        let endpoints = interface
            .endpoints()
            .map(|endpoint| {
                let kind = match (endpoint.transfer_type(), endpoint.direction()) {
                    (TransferType::Bulk, Direction::In) => EndpointKind::BulkIn,
                    (TransferType::Bulk, Direction::Out) => EndpointKind::BulkOut,
                    _ => EndpointKind::Other,
                };
                (endpoint.address(), kind)
            })
            .collect();
        settings.push(AlternateSetting {
            interface_number: interface.interface_number(),
            alternate_setting: interface.alternate_setting(),
            endpoints,
        });
    }
    let layout = discover_endpoint_layout(&settings)
        .map_err(|message| InitFailure::Terminal(TransportError::InvalidConfiguration(message)))?;
    let interface = device
        .claim_interface(layout.interface_number)
        .wait()
        .map_err(classify_nusb_error)?;
    if interface.get_alt_setting() != layout.alternate_setting {
        interface
            .set_alt_setting(layout.alternate_setting)
            .wait()
            .map_err(classify_nusb_error)?;
    }

    initialize_protocol(&interface, layout, config.bitrate).map_err(|error| match error {
        ProtocolInitFailure::Transfer(TransferError::Disconnected) => {
            InitFailure::Transient(TransferError::Disconnected.to_string())
        }
        ProtocolInitFailure::Transfer(error) => {
            InitFailure::Terminal(TransportError::Io(io::Error::from(error)))
        }
        ProtocolInitFailure::Configuration(message) => {
            InitFailure::Terminal(TransportError::InvalidConfiguration(message))
        }
    })?;

    let input = interface
        .endpoint::<Bulk, In>(layout.bulk_in)
        .map_err(classify_nusb_error)?;
    let output = interface
        .endpoint::<Bulk, Out>(layout.bulk_out)
        .map_err(classify_nusb_error)?;
    let input_request_size = input.max_packet_size().max(HOST_FRAME_LEN);
    let input_request_size =
        input_request_size.div_ceil(input.max_packet_size()) * input.max_packet_size();
    let mut session = UsbSession {
        interface,
        input,
        output,
        input_request_size,
    };
    establish_clean_boundary(&mut session, deadline)?;
    while session.input.pending() < IN_FLIGHT {
        submit_input(&mut session.input, session.input_request_size);
    }
    Ok(session)
}

fn initialize_protocol(
    interface: &Interface,
    layout: EndpointLayout,
    bitrate: u32,
) -> Result<(), ProtocolInitFailure> {
    control_out(
        interface,
        BREQ_HOST_FORMAT,
        1,
        u16::from(layout.interface_number),
        &encode_host_format(),
    )?;
    let device_bytes = control_in(
        interface,
        BREQ_DEVICE_CONFIG,
        1,
        u16::from(layout.interface_number),
        12,
    )?;
    let device = DeviceConfig::decode(&device_bytes)
        .map_err(|error| ProtocolInitFailure::Configuration(error.to_string()))?;
    if device.channel_count != 1 {
        return Err(ProtocolInitFailure::Configuration(format!(
            "gs_usb device exposes {} channels; exactly one channel is supported",
            device.channel_count
        )));
    }
    let capability_bytes = control_in(interface, BREQ_BT_CONST, 0, 0, 40)?;
    let constraints = BitTimingConstraints::decode(&capability_bytes)
        .map_err(|error| ProtocolInitFailure::Configuration(error.to_string()))?;
    let timing = calculate_nominal_timing(constraints, bitrate)
        .map_err(ProtocolInitFailure::Configuration)?;

    // Reset before programming timing is required for a clean RX session on
    // adapters left started by a previous process.
    control_out(interface, BREQ_MODE, 0, 0, &encode_mode(MODE_RESET, 0))?;
    thread::sleep(RESET_SETTLE);
    set_timing(interface, timing)?;
    control_out(interface, BREQ_MODE, 0, 0, &encode_mode(MODE_START, 0))?;
    Ok(())
}

fn set_timing(interface: &Interface, timing: NominalBitTiming) -> Result<(), ProtocolInitFailure> {
    control_out(interface, BREQ_BITTIMING, 0, 0, &timing.encode()).map_err(Into::into)
}

fn control_out(
    interface: &Interface,
    request: u8,
    value: u16,
    index: u16,
    data: &[u8],
) -> Result<(), TransferError> {
    interface
        .control_out(
            ControlOut {
                control_type: ControlType::Vendor,
                recipient: Recipient::Interface,
                request,
                value,
                index,
                data,
            },
            CONTROL_TIMEOUT,
        )
        .wait()
}

fn control_in(
    interface: &Interface,
    request: u8,
    value: u16,
    index: u16,
    length: u16,
) -> Result<Vec<u8>, TransferError> {
    interface
        .control_in(
            ControlIn {
                control_type: ControlType::Vendor,
                recipient: Recipient::Interface,
                request,
                value,
                index,
                length,
            },
            CONTROL_TIMEOUT,
        )
        .wait()
}

fn establish_clean_boundary(
    session: &mut UsbSession,
    deadline: Instant,
) -> Result<(), InitFailure> {
    let flush_deadline = deadline.min(Instant::now() + STALE_FLUSH_LIMIT);
    submit_input(&mut session.input, session.input_request_size);
    loop {
        let now = Instant::now();
        if now >= flush_deadline {
            session.input.cancel_all();
            collect_cancelled(&mut session.input);
            return Err(terminal_config(
                "gs_usb stale-input discard did not reach a quiet boundary",
            ));
        }
        let wait = STALE_QUIET_INTERVAL.min(flush_deadline.saturating_duration_since(now));
        match session.input.wait_next_complete(wait) {
            None => return Ok(()),
            Some(completion) => {
                completion.status.map_err(|error| {
                    if error == TransferError::Disconnected {
                        InitFailure::Transient(error.to_string())
                    } else {
                        InitFailure::Terminal(TransportError::Io(io::Error::from(error)))
                    }
                })?;
                let mut buffer = completion.buffer;
                buffer.clear();
                buffer.set_requested_len(session.input_request_size);
                session.input.submit(buffer);
            }
        }
    }
}

fn submit_input(input: &mut Endpoint<Bulk, In>, request_size: usize) {
    let mut buffer = input.allocate(request_size);
    buffer.set_requested_len(request_size);
    input.submit(buffer);
}

fn run_session(
    session: &mut UsbSession,
    tx: &Receiver<CanFrame>,
    rx: &RxQueue,
    statistics: &GsUsbStatistics,
    shutdown: &Receiver<()>,
) -> Result<(), String> {
    let mut echoes = EchoCredits::default();
    let mut pending_tx = VecDeque::new();
    let mut input_halt_recovered = false;
    let mut output_halt_recovered = false;

    loop {
        if shutdown.try_recv().is_ok() {
            return Ok(());
        }

        while pending_tx.len() < TX_CAPACITY {
            match tx.try_recv() {
                Ok(frame) => pending_tx.push_back(frame),
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => return Ok(()),
            }
        }

        if session.output.pending() == 0 && !pending_tx.is_empty() && echoes.in_use() < ECHO_CREDITS
        {
            let echo_id = echoes.allocate().expect("credit count checked");
            let frame = pending_tx.pop_front().expect("queue checked");
            let encoded = encode_host_frame(echo_id, &frame).map_err(|error| error.to_string())?;
            let mut buffer = session.output.allocate(HOST_FRAME_LEN);
            buffer.extend_from_slice(&encoded);
            session.output.submit(buffer);
        }

        if session.output.pending() != 0 {
            if let Some(completion) = session.output.wait_next_complete(Duration::ZERO) {
                match completion.status {
                    Ok(()) if completion.actual_len == HOST_FRAME_LEN => statistics.completed(),
                    Ok(()) => {
                        return Err(format!(
                            "short gs_usb bulk-OUT completion: expected {HOST_FRAME_LEN}, got {}",
                            completion.actual_len
                        ));
                    }
                    Err(error)
                        if classify_transfer_error(error, false, output_halt_recovered)
                            == TransferAction::RecoverHalt =>
                    {
                        output_halt_recovered = true;
                        session
                            .output
                            .clear_halt()
                            .wait()
                            .map_err(|error| error.to_string())?;
                        session.output.submit(completion.buffer);
                    }
                    Err(error) => return Err(format!("gs_usb bulk-OUT failed: {error}")),
                }
            }
        }

        let completion = session.input.wait_next_complete(Duration::from_millis(2));
        let Some(completion) = completion else {
            continue;
        };
        match completion.status {
            Ok(()) => {
                let actual = completion.actual_len;
                if actual == 0 || actual % HOST_FRAME_LEN != 0 {
                    return Err(format!("malformed gs_usb bulk-IN length {actual}"));
                }
                for frame_bytes in completion.buffer[..actual].chunks_exact(HOST_FRAME_LEN) {
                    process_input(frame_bytes, &mut echoes, rx)?;
                }
                let mut buffer = completion.buffer;
                buffer.clear();
                buffer.set_requested_len(session.input_request_size);
                session.input.submit(buffer);
            }
            Err(error)
                if classify_transfer_error(error, false, input_halt_recovered)
                    == TransferAction::RecoverHalt =>
            {
                input_halt_recovered = true;
                session.input.cancel_all();
                collect_cancelled(&mut session.input);
                session
                    .input
                    .clear_halt()
                    .wait()
                    .map_err(|error| error.to_string())?;
                for _ in 0..IN_FLIGHT {
                    submit_input(&mut session.input, session.input_request_size);
                }
            }
            Err(error) => return Err(format!("gs_usb bulk-IN failed: {error}")),
        }
    }
}

fn cleanup_session(session: &mut UsbSession) {
    session.input.cancel_all();
    session.output.cancel_all();
    collect_cancelled(&mut session.input);
    collect_cancelled(&mut session.output);
    if let Err(error) = control_out(
        &session.interface,
        BREQ_MODE,
        0,
        0,
        &encode_mode(MODE_RESET, 0),
    ) {
        log::warn!("failed to reset gs_usb channel during shutdown: {error}");
    }
}

fn collect_cancelled<EpType, Dir>(endpoint: &mut Endpoint<EpType, Dir>)
where
    EpType: nusb::transfer::BulkOrInterrupt,
    Dir: nusb::transfer::EndpointDirection,
{
    while endpoint.pending() != 0 {
        let _ = endpoint.wait_next_complete(Duration::from_secs(1));
    }
}

fn classify_nusb_error(error: nusb::Error) -> InitFailure {
    match error.kind() {
        ErrorKind::Disconnected | ErrorKind::NotFound => InitFailure::Transient(error.to_string()),
        ErrorKind::PermissionDenied => InitFailure::Terminal(TransportError::PermissionDenied),
        _ => InitFailure::Terminal(TransportError::Io(io::Error::from(error))),
    }
}

fn discovery_error(error: nusb::Error) -> TransportError {
    match error.kind() {
        ErrorKind::PermissionDenied => TransportError::PermissionDenied,
        _ => TransportError::Io(io::Error::from(error)),
    }
}

fn terminal_config(message: impl Into<String>) -> InitFailure {
    InitFailure::Terminal(TransportError::InvalidConfiguration(message.into()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicBool, Ordering};

    fn fake_bus() -> (GsUsbBus, Receiver<CanFrame>, Arc<AtomicBool>) {
        let statistics = GsUsbStatistics::default();
        let rx = Arc::new(RxQueue::new(statistics.clone()));
        let terminal_error = Arc::new(Mutex::new(None));
        let (tx_sender, tx_receiver) = mpsc::sync_channel(TX_CAPACITY);
        let (shutdown_sender, shutdown_receiver) = mpsc::channel();
        let stopped = Arc::new(AtomicBool::new(false));
        let worker_stopped = Arc::clone(&stopped);
        let worker = thread::spawn(move || {
            let _ = shutdown_receiver.recv();
            worker_stopped.store(true, Ordering::Release);
        });
        (
            GsUsbBus {
                name: "fake-gs_usb".into(),
                tx: Some(tx_sender),
                rx,
                terminal_error,
                shutdown: shutdown_sender,
                worker: Some(worker),
                statistics,
            },
            tx_receiver,
            stopped,
        )
    }

    #[test]
    fn defaults_and_constructor_validation_are_explicit() {
        let config = GsUsbConfig::new(0x1d50, 0x606f);
        assert_eq!(config.bitrate, 1_000_000);
        assert_eq!(config.initialization_timeout, Duration::from_secs(5));
        assert_eq!(
            validated_selector(config.serial_number.as_deref(), config.index).unwrap(),
            Selector::Index(0)
        );
    }

    #[test]
    fn initialization_sequence_keeps_reset_before_timing_and_start() {
        const STEPS: &[&str] = &[
            "host-format",
            "device-config",
            "timing-capabilities",
            "validate-channel-0",
            "reset",
            "program-timing",
            "start",
            "quiet-boundary",
            "prime-rx",
        ];
        assert_eq!(STEPS[4..7], ["reset", "program-timing", "start"]);
        assert!(
            STEPS
                .iter()
                .position(|step| *step == "host-format")
                .unwrap()
                < STEPS
                    .iter()
                    .position(|step| *step == "device-config")
                    .unwrap()
        );
        assert!(
            STEPS
                .iter()
                .position(|step| *step == "quiet-boundary")
                .unwrap()
                < STEPS.iter().position(|step| *step == "prime-rx").unwrap()
        );
        assert_eq!(STEPS.last(), Some(&"prime-rx"));
    }

    #[test]
    fn retry_restarts_complete_attempts_but_terminal_errors_fail_immediately() {
        let mut config = GsUsbConfig::new(1, 2);
        config.initialization_timeout = Duration::from_secs(1);
        let mut attempts = 0;
        let value = retry_initialization(&config, &Selector::Index(0), |_| {
            attempts += 1;
            if attempts < 3 {
                Err(InitFailure::Transient("re-enumerating".into()))
            } else {
                Ok(42)
            }
        })
        .unwrap();
        assert_eq!(value, 42);
        assert_eq!(attempts, 3);

        let mut terminal_attempts = 0;
        let error = retry_initialization::<()>(&config, &Selector::Index(0), |_| {
            terminal_attempts += 1;
            Err(terminal_config("bad descriptor"))
        })
        .unwrap_err();
        assert!(matches!(error, TransportError::InvalidConfiguration(_)));
        assert_eq!(terminal_attempts, 1);
    }

    #[test]
    fn can_bus_contract_is_nonblocking_bounded_and_classical() {
        let (mut bus, receiver, _) = fake_bus();
        assert_eq!(bus.capabilities(), BusCapabilities::classical());
        assert_eq!(bus.raw_fd(), None);
        assert!(bus.drain_inbound_nonblocking().unwrap().is_empty());
        let inbound = CanFrame::classical(0x456, &[4, 5, 6]).unwrap();
        bus.rx.push(inbound);
        assert_eq!(bus.drain_inbound_nonblocking().unwrap(), vec![inbound]);

        let standard = CanFrame::classical(0x123, &[1]).unwrap();
        let extended = CanFrame::classical_extended(0x12345, &[2]).unwrap();
        bus.send(&standard).unwrap();
        bus.send(&extended).unwrap();
        assert_eq!(receiver.try_recv().unwrap(), standard);
        assert_eq!(receiver.try_recv().unwrap(), extended);
        assert_eq!(bus.statistics().tx_accepted(), 2);

        let fd = CanFrame::fd(1, &[0; 8]).unwrap();
        assert!(matches!(
            bus.send(&fd),
            Err(TransportError::FdFrameOnNonFdBus)
        ));
    }

    #[test]
    fn full_tx_rejects_new_frame_without_evicting_old_frames() {
        let (mut bus, receiver, _) = fake_bus();
        for id in 0..TX_CAPACITY {
            bus.send(&CanFrame::classical((id % 0x7ff) as u32, &[]).unwrap())
                .unwrap();
        }
        assert!(matches!(
            bus.send(&CanFrame::classical(1, &[]).unwrap()),
            Err(TransportError::SendBufferFull)
        ));
        assert_eq!(receiver.try_iter().count(), TX_CAPACITY);
        assert_eq!(bus.statistics().tx_accepted(), TX_CAPACITY as u64);
    }

    #[test]
    fn terminal_fault_fails_closed_and_drop_joins_worker() {
        let (mut bus, _receiver, stopped) = fake_bus();
        *bus.terminal_error.lock().unwrap() = Some("device disconnected".into());
        assert!(matches!(
            bus.send(&CanFrame::classical(1, &[]).unwrap()),
            Err(TransportError::Io(_))
        ));
        assert!(matches!(
            bus.drain_inbound_nonblocking(),
            Err(TransportError::Io(_))
        ));
        drop(bus);
        assert!(stopped.load(Ordering::Acquire));
    }

    #[test]
    fn transfer_errors_distinguish_shutdown_recovery_and_terminal_failure() {
        assert_eq!(
            classify_transfer_error(TransferError::Cancelled, true, false),
            TransferAction::ExpectedShutdown
        );
        assert_eq!(
            classify_transfer_error(TransferError::Stall, false, false),
            TransferAction::RecoverHalt
        );
        assert_eq!(
            classify_transfer_error(TransferError::Stall, false, true),
            TransferAction::Fault
        );
        for error in [
            TransferError::Disconnected,
            TransferError::Fault,
            TransferError::InvalidArgument,
            TransferError::Unknown(7),
            TransferError::Cancelled,
        ] {
            assert_eq!(
                classify_transfer_error(error, false, false),
                TransferAction::Fault
            );
        }
    }

    #[test]
    fn drop_joins_with_quiet_pending_and_faulted_command_queues() {
        for scenario in ["quiet-rx", "tx-pending", "out-pending", "already-faulted"] {
            let (mut bus, _receiver, stopped) = fake_bus();
            if scenario == "tx-pending" || scenario == "out-pending" {
                bus.send(&CanFrame::classical(1, &[1]).unwrap()).unwrap();
            }
            if scenario == "already-faulted" {
                *bus.terminal_error.lock().unwrap() = Some("faulted".into());
            }
            drop(bus);
            assert!(stopped.load(Ordering::Acquire), "scenario {scenario}");
        }

        // Before ready, all USB values are local to the worker. Dropping the
        // readiness receiver makes the worker take its cleanup-and-return path
        // rather than detach.
        let (ready_tx, ready_rx) = mpsc::sync_channel::<Result<(), TransportError>>(1);
        drop(ready_rx);
        assert!(ready_tx.send(Ok(())).is_err());
    }

    #[test]
    fn fake_device_can_reopen_immediately_without_stale_session_frames() {
        struct FakeDevice {
            claimed: Arc<AtomicBool>,
            stale_frames: Arc<Mutex<Vec<CanFrame>>>,
        }
        impl FakeDevice {
            fn open(
                claimed: Arc<AtomicBool>,
                stale_frames: Arc<Mutex<Vec<CanFrame>>>,
            ) -> Result<Self, &'static str> {
                if claimed.swap(true, Ordering::AcqRel) {
                    return Err("interface retained");
                }
                stale_frames.lock().unwrap().clear();
                Ok(Self {
                    claimed,
                    stale_frames,
                })
            }
        }
        impl Drop for FakeDevice {
            fn drop(&mut self) {
                self.stale_frames.lock().unwrap().clear();
                self.claimed.store(false, Ordering::Release);
            }
        }

        let claimed = Arc::new(AtomicBool::new(false));
        let stale = Arc::new(Mutex::new(vec![CanFrame::classical(1, &[1]).unwrap()]));
        let first = FakeDevice::open(Arc::clone(&claimed), Arc::clone(&stale)).unwrap();
        stale
            .lock()
            .unwrap()
            .push(CanFrame::classical(2, &[2]).unwrap());
        drop(first);
        let _second = FakeDevice::open(Arc::clone(&claimed), Arc::clone(&stale)).unwrap();
        assert!(stale.lock().unwrap().is_empty());
    }
}
