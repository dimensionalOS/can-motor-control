# RobStride support specification

Add a separate no_std Rust codec and thin Python bindings for RS00 and RS06, the models used on B601 RS. Preserve the existing Robostride MotorTypeId spelling for compatibility. Use robstride as the public vendor name. Existing Damiao behavior remains unchanged.

Commands use extended classical CAN with communication type in bits 24 through 28, host data in bits 8 through 23 and destination motor in bits 0 through 7. MIT commands carry torque in the identifier and four unsigned big endian payload fields. Status has motor ID in bits 8 through 15 and host ID in bits 0 through 7. Routing keys use the motor ID because every motor can share one host ID. The codec is configured with the host ID and a model table keyed by motor ID. Unknown motors and wrong hosts cannot update state. Faulted status produces a fault event.

Implement enable, disable, explicit zero, MIT mode selection and readback, MIT commands, typed parameter extensions and status decoding. Unsupported command modes fail explicitly. No implicit enable, zero, flash save or motor motion occurs during construction or connection. Refresh remains unsupported because a position parameter reply is not a full state report. Rotor temperature is unavailable and represented by the minimum i16 sentinel.

Verify golden packets against the pinned official RobStride Python sample. Test malformed input, finite numeric bounds, model scaling, shared host routing, faults, lifecycle frames, Python construction and mock bus integration. Hardware timing and motion qualification remain pending.
