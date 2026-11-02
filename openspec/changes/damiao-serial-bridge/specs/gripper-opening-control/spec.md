## MODIFIED Requirements

### Requirement: Opening calibration runs during enable before ready
The system SHALL perform per-session opening calibration for normalized gripper control during the robot enable lifecycle, after connection and before the robot is ready for normal opening commands. Opening calibration SHALL discover the raw motor positions that correspond to fully open and fully closed openings. Calibration SHALL drive toward each stop by the gripper's configured calibration travel, 4.0 rad when none is configured. A configured travel that is not finite and positive SHALL fail enable before calibration commands the gripper.

#### Scenario: Enable calibrates gripper opening
- **WHEN** a robot with a gripper configured for opening control is connected and enabled
- **THEN** the enable lifecycle calibrates that gripper's open and closed endpoints before returning success

#### Scenario: Configured calibration travel
- **WHEN** a gripper is configured with a calibration travel longer than its stroke
- **THEN** calibration reaches both stops from any start position

#### Scenario: Invalid calibration travel
- **WHEN** a gripper's calibration travel is zero, negative or not finite
- **THEN** enable fails with an opening calibration error and the gripper receives no position command

#### Scenario: Context manager calibrates before user code
- **WHEN** Python code enters a `with Robot.from_config(...) as robot:` block for a robot with a gripper configured for opening control
- **THEN** the context-manager entry completes opening calibration before user code inside the block runs

#### Scenario: Opening command before calibration is rejected
- **WHEN** a caller invokes `set_opening` before opening calibration has completed
- **THEN** the system returns a lifecycle error and does not command normalized gripper motion
