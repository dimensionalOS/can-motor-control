//! Cross-platform robot control for CAN motors using the target's native
//! hardware transport.
//!
//! This crate is intentionally vendor-agnostic above the codec seam: it depends
//! only on the [`motor_codec`] trait crate, never on a specific vendor codec.

// Some crate-private items (Bus::routes, Motor::new, MotorGroup::attach_bus, ...)
// are wired up by the RobotBuilder and tick loop landing in subsequent task
// groups. Allow `dead_code` here until those callers land.
#![allow(dead_code)]

pub mod bus;
pub mod config;
pub mod error;
pub mod group;
pub mod motor;
pub mod robot;
pub mod spec;
pub mod transport;

pub use bus::{Bus, RouteKey};
pub use config::{CodecFactory, CodecRegistry, MotorTypeParser};
pub use error::Error;
pub use group::{
    Arm, Generic, Gripper, GroupKind, MitCmd, MotorGroup, PosForceCmd, PosVelCmd, VelCmd,
};
pub use motor::{FaultCode, Motor};
pub use robot::{Robot, RobotBuilder};
pub use spec::{GripperOpeningSpec, GroupSpecKind, MotorSpec, OpeningDirection};
#[cfg(target_os = "linux")]
pub use transport::SocketCanBus;
#[cfg(target_os = "macos")]
pub use transport::{list_gs_usb_devices, GsUsbBus, GsUsbConfig, GsUsbDeviceInfo, GsUsbStatistics};
pub use transport::{BusPoller, CanBus, MockCanBus, MockRecordedCall, TransportError};

// Re-export can-motor-codec public types so users have a single import surface.
pub use motor_codec::{
    BusCapabilities, CanFrame, CodecError, Command, CommandKind, Event, FrameError, FrameFlags,
    Limits, MotorCodec, MotorRef, MotorTypeId, ParamValue,
};
