//! Python bindings for `can-motor-control`. Built as `can_motor_control._native` by maturin.

// pyo3 0.22's `create_exception!` macro emits a `cfg(gil-refs)` check that
// cargo flags as unexpected on stable. The feature exists upstream; the
// warning is benign and goes away in pyo3 0.23.
#![allow(unexpected_cfgs)]
// pyo3 macro-expanded code triggers `useless_conversion` on its own
// `.into()` wrappers in some 0.22 versions; that's noise, not user-fixable.
#![allow(clippy::useless_conversion)]

use pyo3::create_exception;
use pyo3::exceptions::PyException;
use pyo3::prelude::*;

mod codec;
mod errors;
mod frame;
mod robot;
mod spec;
mod transport;

use codec::{
    PyDamiaoCodec, PyMockFeedbackCodec, PyMotorType, PyRobstrideCodec, PyRobstrideMotorType,
};
use frame::PyCanFrame;
use robot::{PyArm, PyGripper, PyMotor, PyMotorGroup, PyRobot, PyRobotBuilder};
use spec::PyMotorSpec;
use transport::PyMockCanBus;
#[cfg(target_os = "linux")]
use transport::PySocketCanBus;
#[cfg(target_os = "macos")]
use transport::{py_list_gs_usb_devices, PyGsUsbBus, PyGsUsbDeviceInfo};

// The module argument sets each exception's `__module__`. Use `can_motor_control`
// (the public package the exceptions are re-exported from and imported as), not
// `_native`, so it matches the `module = "can_motor_control"` on the `#[pyclass]`
// types — a consistent `__module__` is what lets doc tooling (griffe) resolve
// the re-exports without a dangling bare-`_native` target.
create_exception!(
    can_motor_control,
    DmError,
    PyException,
    "Base class for every error raised by ``can_motor_control``.\n\nCatch this to handle any failure from the library regardless of cause."
);
create_exception!(
    can_motor_control,
    TransportError,
    DmError,
    "A CAN transport operation failed (e.g. the SocketCAN interface is down, \
a send/receive timed out, or a frame was malformed)."
);
create_exception!(
    can_motor_control,
    CodecError,
    DmError,
    "A motor frame could not be encoded or decoded by the vendor codec."
);
create_exception!(
    can_motor_control,
    ConfigError,
    DmError,
    "The robot configuration is invalid: bad config file, unknown vendor, \
unknown bus name, or an unsupported group/bus option."
);
create_exception!(
    can_motor_control,
    LifecycleError,
    DmError,
    "An operation was attempted in the wrong lifecycle state, such as ticking \
before `Robot.connect` or mutating topology after the robot is built."
);

#[pymodule]
fn _native(py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Classes.
    m.add_class::<PyCanFrame>()?;
    m.add_class::<PyMockCanBus>()?;
    #[cfg(target_os = "linux")]
    m.add_class::<PySocketCanBus>()?;
    #[cfg(target_os = "macos")]
    m.add_class::<PyGsUsbBus>()?;
    #[cfg(target_os = "macos")]
    m.add_class::<PyGsUsbDeviceInfo>()?;
    #[cfg(target_os = "macos")]
    m.add_function(wrap_pyfunction!(py_list_gs_usb_devices, m)?)?;
    m.add_class::<PyMotorSpec>()?;
    m.add_class::<PyMotor>()?;
    m.add_class::<PyArm>()?;
    m.add_class::<PyGripper>()?;
    m.add_class::<PyMotorGroup>()?;
    m.add_class::<PyRobot>()?;
    m.add_class::<PyRobotBuilder>()?;
    m.add_class::<PyDamiaoCodec>()?;
    m.add_class::<PyMockFeedbackCodec>()?;
    m.add_class::<PyMotorType>()?;
    m.add_class::<PyRobstrideCodec>()?;
    m.add_class::<PyRobstrideMotorType>()?;

    // Exceptions.
    m.add("DmError", py.get_type_bound::<DmError>())?;
    m.add("TransportError", py.get_type_bound::<TransportError>())?;
    m.add("CodecError", py.get_type_bound::<CodecError>())?;
    m.add("ConfigError", py.get_type_bound::<ConfigError>())?;
    m.add("LifecycleError", py.get_type_bound::<LifecycleError>())?;
    Ok(())
}
