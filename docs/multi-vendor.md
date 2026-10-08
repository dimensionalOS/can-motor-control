# Multi-vendor codec architecture

## Why a trait

Damiao is one of a family of low-cost CAN motors with broadly similar
control surfaces: Robostride, MyActuator, CubeMars all expose MIT-mode
plus optional position/velocity control over CAN, with vendor-specific
bit-packing and parameter sub-protocols. The library treats the codec
as a swappable trait so the rest of the stack — transport, motor cache,
groups, robot, control loops, Python bindings — stays vendor-agnostic.

## The trait

`motor_codec::MotorCodec` (in `crates/motor-codec`) defines:

- `vendor_name(&self) -> &'static str`
- `supports(&self, motor_type: MotorTypeId) -> bool`
- `limits(&self, motor_type: MotorTypeId) -> Result<Limits, CodecError>`
- `bind_to_bus(&mut self, caps: BusCapabilities)`
- `encode_enable / encode_disable / encode_set_zero / encode_command`
- `decode(&self, frame: &CanFrame) -> Result<Option<Event>, CodecError>`

It is object-safe (no associated types, no generics on methods), so the
rest of the library uses `Box<dyn MotorCodec>` exclusively. The
`can-motor-control` crate has **zero** dependency on any vendor codec — verified
by a CI grep job.

## Adding a vendor

Adding Robostride (or any other vendor) is a purely additive change:

1. **Add a variant** to `motor_codec::MotorTypeId`:
   ```rust
   MotorTypeId::Robostride(u16)  // the u16 is opaque to motor_codec
   ```
   The enum is `#[non_exhaustive]`, so existing downstream code keeps
   compiling.
2. **Create a new crate** `crates/robostride-codec` that:
   - Defines `RobostrideMotorType` (the SKU enum) with `From<RobostrideMotorType> for MotorTypeId`
   - Implements `impl MotorCodec for RobostrideCodec`
   - Exposes a `parse_motor_type(&str) -> Option<MotorTypeId>`
   - Exposes a `VENDOR_NAME: &str = "robostride"` constant
   - Optionally defines a `RobostrideCodecExt` trait for vendor-specific
     operations (mode-config registers, etc.)
3. **Register with the codec registry** at robot construction:
   ```rust
   let mut reg = CodecRegistry::new();
   reg.register("damiao", || Box::new(DamiaoCodec::new()), damiao::parse_motor_type);
   reg.register("robostride", || Box::new(RobostrideCodec::new()), robostride::parse_motor_type);
   let robot = Robot::from_config("config.toml", &reg)?;
   ```
4. **Mention the vendor in TOML**:
   ```toml
   [bus.left]
   kind = "socketcan"
   interface = "can0"
   vendor = "robostride"   # ← codec lives on the bus, not the group
   ```

No changes are required to `Robot`, `RobotBuilder`, `Arm`, `Gripper`,
`MotorGroup`, the Python bindings, or any existing codec.

## Codec lives on the bus, not the group

A bus's codec is shared across every group attached to it. This means:

- **Decode happens once per frame.** With arm + gripper on the same bus,
  only the bus's codec decodes; the routing table (`bus.routes`)
  dispatches the resulting event to the owning group.
- **CAN ID collisions are caught at `connect()`** because the routing
  table is built from all groups on the bus.
- **Mixed vendors on one CAN wire is not supported in v1.** A real
  hardware setup that mixes Damiao + Robostride on one bus would need
  to split into two buses, or wait for a future "multi-vendor bus"
  change.

## Vendor extensions

Vendor-specific operations (e.g. Damiao's `0x7FF` parameter sub-protocol)
don't belong on the common trait. They live on a per-vendor extension
trait in the vendor's own crate:

```rust
// in damiao_codec
pub trait DamiaoCodecExt {
    fn encode_read_param(&self, motor: MotorRef<'_>, rid: DamiaoRid) -> CanFrame;
    fn encode_write_param(&self, motor: MotorRef<'_>, rid: DamiaoRid, val: ParamValue) -> CanFrame;
    fn encode_save_to_flash(&self, motor: MotorRef<'_>) -> CanFrame;
    fn encode_refresh(&self, motor: MotorRef<'_>) -> CanFrame;
}
impl DamiaoCodecExt for DamiaoCodec { ... }
```

Callers reach the extension via downcast on the trait object. The
downcast returns `Some(...)` for matching vendor, `None` otherwise.

## RobStride model binding

The RobStride codec is implemented in `crates/robstride-codec`. The common trait now includes `bind_motor`, whose default does nothing for stateless codecs. Robot connection calls it for every configured motor before receiving frames. RobStride needs this table because its status packet identifies the device but does not identify the model. Python exposes the codec and model enum and registers the vendor in its configuration loader. See [the protocol guide](robstride.md) for routing and supported modes.
