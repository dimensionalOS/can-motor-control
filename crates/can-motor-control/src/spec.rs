//! Builder input types.

use motor_codec::MotorTypeId;

/// Whether increasing raw motor position increases or decreases gripper opening.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum OpeningDirection {
    /// Increasing raw motor position moves the gripper toward fully open.
    IncreasingPosition,
    /// Decreasing raw motor position moves the gripper toward fully open.
    DecreasingPosition,
}

impl OpeningDirection {
    /// Sign to apply to a positive opening span in raw motor position units.
    pub(crate) fn sign(self) -> f64 {
        match self {
            Self::IncreasingPosition => 1.0,
            Self::DecreasingPosition => -1.0,
        }
    }
}

/// Configuration required for normalized gripper opening commands.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct GripperOpeningSpec {
    /// Opening direction for this gripper mechanism.
    pub direction: OpeningDirection,
    /// Default per-unit current used when opening commands omit one.
    pub default_current: Option<f64>,
    /// Commanded travel in radians toward each stop during calibration.
    /// `None` uses the library default of 4.0 rad. Set it above the
    /// mechanism's full stroke so both stops are reached from any start.
    pub calibration_travel: Option<f64>,
}

impl GripperOpeningSpec {
    /// Constructor for normalized gripper opening configuration.
    pub fn new(direction: OpeningDirection, default_current: Option<f64>) -> Self {
        Self {
            direction,
            default_current,
            calibration_travel: None,
        }
    }

    /// Override the calibration travel toward each stop, in radians.
    pub fn with_calibration_travel(mut self, travel: f64) -> Self {
        self.calibration_travel = Some(travel);
        self
    }
}

/// User-supplied motor specification for the [`crate::RobotBuilder`].
#[derive(Clone, Debug, PartialEq)]
pub struct MotorSpec {
    /// Human-readable motor name unique within the group.
    pub name: String,
    /// Vendor type identifier (e.g. `MotorTypeId::Damiao(<DM4340 disc>)`).
    pub motor_type: MotorTypeId,
    /// CAN ID this motor accepts commands on.
    pub send_id: u32,
    /// CAN ID this motor emits state and replies from.
    pub recv_id: u32,
}

impl MotorSpec {
    /// Convenience constructor.
    pub fn new(
        name: impl Into<String>,
        motor_type: impl Into<MotorTypeId>,
        send_id: u32,
        recv_id: u32,
    ) -> Self {
        Self {
            name: name.into(),
            motor_type: motor_type.into(),
            send_id,
            recv_id,
        }
    }
}

/// Group kind discriminator the builder uses.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum GroupSpecKind {
    /// Articulated arm.
    Arm,
    /// One-motor gripper.
    Gripper,
    /// Generic catch-all.
    Generic,
}
