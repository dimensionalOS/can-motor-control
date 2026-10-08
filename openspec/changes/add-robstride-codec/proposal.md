# RobStride codec

The existing trait reserves a Robostride motor type but has no implementation. Add RS00 and RS06 protocol support through a new Rust codec, register it in Python and preserve Damiao behavior. All validation uses synthetic packets and MockCanBus.
