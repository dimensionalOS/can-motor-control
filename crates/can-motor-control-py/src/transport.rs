//! Python wrappers for CAN transports.

use std::sync::{Arc, Mutex};

#[cfg(target_os = "linux")]
use can_motor_control::SocketCanBus;
#[cfg(target_os = "macos")]
use can_motor_control::{
    list_gs_usb_devices, GsUsbBus, GsUsbConfig, GsUsbDeviceInfo, GsUsbStatistics,
};
use can_motor_control::{CanBus, MockCanBus};
use pyo3::prelude::*;

use crate::errors::transport_to_pyerr;

/// Internal trait-object holder shared between Python wrappers and the
/// builder. Once the builder consumes a transport, the Arc has exactly one
/// strong reference inside the bus; the Python wrapper's Arc is dropped.
#[derive(Clone)]
pub struct TransportHandle {
    pub(crate) inner: Arc<Mutex<Option<Box<dyn CanBus>>>>,
}

impl TransportHandle {
    pub fn new<T: CanBus + 'static>(t: T) -> Self {
        Self {
            inner: Arc::new(Mutex::new(Some(Box::new(t)))),
        }
    }

    pub fn take(&self) -> Option<Box<dyn CanBus>> {
        self.inner.lock().ok()?.take()
    }
}

/// An in-memory CAN bus for tests and dry runs.
///
/// Drop-in replacement for `SocketCanBus` that talks to no hardware,
/// so robot code can be exercised without a real CAN interface. Pass it to
/// `RobotBuilder.add_bus`. A bus is consumed by the builder and may
/// be added to only one robot.
///
/// The default constructor is classical-CAN; use `MockCanBus.new_fd` for an
/// FD-capable mock that accepts and loops back CAN-FD frames.
#[pyclass(name = "MockCanBus", module = "can_motor_control")]
pub struct PyMockCanBus {
    pub(crate) handle: TransportHandle,
}

#[pymethods]
impl PyMockCanBus {
    /// Create a classical-CAN mock bus identified by ``name`` (used in error messages).
    #[new]
    fn new(name: &str) -> Self {
        Self {
            handle: TransportHandle::new(MockCanBus::new(name)),
        }
    }

    /// Create a CAN-FD-capable mock bus. It advertises FD capabilities and
    /// loops back FD frames, so the FD send/receive path is testable without an
    /// FD-capable interface.
    #[staticmethod]
    fn new_fd(name: &str) -> Self {
        Self {
            handle: TransportHandle::new(MockCanBus::new_fd(name)),
        }
    }
}

/// A CAN bus backed by a Linux SocketCAN interface.
///
/// Opens ``interface`` (e.g. ``"can0"``) on construction; a failure to open
/// raises `TransportError`. Pass it to
/// `RobotBuilder.add_bus`. A bus is consumed by the builder and may
/// be added to only one robot.
#[pyclass(name = "SocketCanBus", module = "can_motor_control")]
#[cfg(target_os = "linux")]
pub struct PySocketCanBus {
    pub(crate) handle: TransportHandle,
}

#[pymethods]
#[cfg(target_os = "linux")]
impl PySocketCanBus {
    /// Open SocketCAN ``interface``.
    ///
    /// Set ``fd=True`` to open the interface in CAN-FD mode (the interface
    /// itself must be FD-capable). Raises `TransportError` if the
    /// interface cannot be opened.
    #[new]
    #[pyo3(signature = (interface, *, fd=false))]
    fn new(py: Python<'_>, interface: &str, fd: bool) -> PyResult<Self> {
        // Release the GIL during the syscall.
        let bus = py.allow_threads(|| SocketCanBus::open(interface, fd));
        let bus = bus.map_err(transport_to_pyerr)?;
        Ok(Self {
            handle: TransportHandle::new(bus),
        })
    }
}

/// A discovered macOS gs_usb adapter.
#[cfg(target_os = "macos")]
#[pyclass(name = "GsUsbDeviceInfo", module = "can_motor_control", frozen)]
pub struct PyGsUsbDeviceInfo {
    #[pyo3(get)]
    pub index: usize,
    #[pyo3(get)]
    pub vendor_id: u16,
    #[pyo3(get)]
    pub product_id: u16,
    #[pyo3(get)]
    pub serial_number: Option<String>,
}

#[cfg(target_os = "macos")]
impl From<GsUsbDeviceInfo> for PyGsUsbDeviceInfo {
    fn from(device: GsUsbDeviceInfo) -> Self {
        Self {
            index: device.index,
            vendor_id: device.vendor_id,
            product_id: device.product_id,
            serial_number: device.serial_number,
        }
    }
}

/// List macOS gs_usb adapters matching a USB vendor/product pair.
#[cfg(target_os = "macos")]
#[pyfunction(name = "list_gs_usb_devices", signature = (*, vendor_id, product_id))]
pub fn py_list_gs_usb_devices(
    py: Python<'_>,
    vendor_id: u16,
    product_id: u16,
) -> PyResult<Vec<PyGsUsbDeviceInfo>> {
    py.allow_threads(|| list_gs_usb_devices(vendor_id, product_id))
        .map(|devices| devices.into_iter().map(Into::into).collect())
        .map_err(transport_to_pyerr)
}

/// A classical-CAN bus backed by a gs_usb adapter through native macOS IOKit.
#[cfg(target_os = "macos")]
#[pyclass(name = "GsUsbBus", module = "can_motor_control")]
pub struct PyGsUsbBus {
    pub(crate) handle: TransportHandle,
    statistics: GsUsbStatistics,
}

#[cfg(target_os = "macos")]
#[pymethods]
impl PyGsUsbBus {
    #[new]
    #[pyo3(signature = (*, vendor_id, product_id, serial_number=None, index=None, bitrate=1_000_000, initialization_timeout=5.0))]
    fn new(
        py: Python<'_>,
        vendor_id: u16,
        product_id: u16,
        serial_number: Option<String>,
        index: Option<usize>,
        bitrate: u32,
        initialization_timeout: f64,
    ) -> PyResult<Self> {
        if !initialization_timeout.is_finite() || initialization_timeout <= 0.0 {
            return Err(pyo3::exceptions::PyValueError::new_err(
                "initialization_timeout must be finite and greater than zero",
            ));
        }
        let mut config = GsUsbConfig::new(vendor_id, product_id);
        config.serial_number = serial_number;
        config.index = index;
        config.bitrate = bitrate;
        config.initialization_timeout = std::time::Duration::from_secs_f64(initialization_timeout);
        // USB discovery and the complete ready-or-error initialization wait do
        // not hold the Python GIL.
        let bus = py.allow_threads(|| GsUsbBus::open(config));
        let bus = bus.map_err(transport_to_pyerr)?;
        let statistics = bus.statistics();
        Ok(Self {
            handle: TransportHandle::new(bus),
            statistics,
        })
    }

    #[getter]
    fn rx_received(&self) -> u64 {
        self.statistics.rx_received()
    }
    #[getter]
    fn rx_dropped(&self) -> u64 {
        self.statistics.rx_dropped()
    }
    #[getter]
    fn tx_accepted(&self) -> u64 {
        self.statistics.tx_accepted()
    }
    #[getter]
    fn tx_completed(&self) -> u64 {
        self.statistics.tx_completed()
    }
}
