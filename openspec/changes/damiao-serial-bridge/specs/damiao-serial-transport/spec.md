## ADDED Requirements

### Requirement: DmSerialBus carries classical CAN over the Damiao serial bridge

`DmSerialBus` SHALL implement `CanBus` over the serial port of a Damiao USB-CAN bridge on Linux and macOS. It SHALL encode each frame as the bridge's 30-byte transmit packet and decode the bridge's 16-byte receive packets. It MUST reject FD frames, extended IDs, remote frames and payloads other than eight bytes before any IO.

#### Scenario: Frames round-trip through the bridge format
- **WHEN** a robot on `DmSerialBus` sends a standard eight-byte frame and the bridge returns a receive packet
- **THEN** the port receives one 30-byte packet with the frame's ID and payload, and the robot decodes the returned frame

#### Scenario: Receive packets split, merged or preceded by noise
- **WHEN** serial reads split a receive packet, merge several, or carry bytes that are not a packet
- **THEN** the parser keeps partial packets across reads, returns every complete frame in order, and resynchronizes one byte at a time

### Requirement: Each packet is one whole write

On the tested bridge, a tty write containing seven packets lost every reply, while seven separate writes lost none. This observation does not establish the USB-transfer boundaries or firmware parsing mechanism. As a workaround, `DmSerialBus` SHALL start each packet with its own write and MUST NOT batch packets. Partial writes SHALL be completed before starting another packet. A packet the driver refuses outright SHALL report `SendBufferFull` and leave the stream intact. Once part of a packet is written, any failure to write the rest SHALL leave the bus refusing every later send.

#### Scenario: Partial packet cannot complete
- **WHEN** a write accepts part of a packet and the rest cannot be written before the send timeout
- **THEN** the send fails and every later send on that bus fails

### Requirement: The port is owned exclusively

`DmSerialBus` SHALL take an advisory lock on the port and then set the tty exclusive. A second open SHALL fail with a configuration error stating that the port is already open, without changing the holder's port.

#### Scenario: Second process opens the port
- **WHEN** a port is held by one `DmSerialBus` and another `DmSerialBus` opens it
- **THEN** construction fails with an "already open" configuration error

### Requirement: Frame gap is bounded

`DmSerialBus` SHALL accept an optional minimum gap between packets, default zero, at most one second. Python SHALL reject a negative, non-finite or larger gap with `ValueError`.

#### Scenario: Invalid gap from Python
- **WHEN** Python constructs `DmSerialBus` with `frame_gap` of `-1`, NaN, infinity or more than one second
- **THEN** construction raises `ValueError`

### Requirement: Disconnection surfaces as a transport error

A bridge that disappears SHALL surface as `TransportError` from receive and send.

#### Scenario: Bridge unplugged
- **WHEN** the serial device hangs up
- **THEN** draining inbound frames and sending both return transport errors
