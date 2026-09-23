# rover_modbus_driver

Synchronous Modbus TCP client for the rover: the discrete contact/coil port, the retrying
client that speaks it, and the POSIX TCP transport behind it.

## Why this package has no node

Every other `rover_transport` package is a lifecycle bridge node that pumps raw bytes
between a device and a pair of topics. This one is **library only** - no
`*_node_main.cpp`, no `launch/`, no `config/`, and no `rover_io_context` dependency.

Two reasons:

1. **Modbus is not a byte stream.** `rover_io_context`'s `ByteStreamPort` is an async raw
   byte stream with a re-arming receive callback. `MB::TCP::Connection` is a blocking,
   transaction-ID-matched request/response client that owns its own `::poll()` loop. There
   is no honest adapter between the two - any consumer would have to reimplement framing
   and transaction matching on top of a raw byte topic.

2. **The consumer is on the safety path.** `RoverSafetyController`, inside a ros2_control
   `SystemInterface`, reads and writes e-stop coils. A topic round-trip there would add
   latency and a new failure mode for no benefit - and `rover_hardware_interface`'s
   `check_rt_path_purity.sh` already treats these calls as blocking.

## Layers

| Layer | Contents | Library |
|-------|----------|---------|
| `domain/` | `Contact`/`Coil` types, `ClientSettings`, and the `DiscreteIoPort`, `ModbusTransportPort` and `LoggerPort` seams | `rover_modbus_driver_core` |
| `application/` | `ModbusDiscreteIoClient` - contact/coil operations to Modbus frames, plus connection retry | `rover_modbus_driver_core` |
| `infrastructure/` | `ModbusTcpTransport` (sockets), `RclcppLogger`, and `makeModbusTcpDiscreteIoClient()` | `rover_modbus_driver_ros` |

`_core` links only `rover_modbus::Modbus_Core`, the OS-independent frame codec - **not**
`Modbus_Tcp`. So "no sockets in `_core`" is enforced by the linker, not by convention.
Concretely: `_core` must never include `<MB/connection.hpp>` or `<MB/server.hpp>`.

`_core` names `MB::ModbusRequest`/`MB::ModbusResponse` in its public headers. That is
deliberate - they are pure value types over bytes, with no OS behind them. Narrowing
`ModbusTransportPort` so it never mentions an `MB::` type (e.g.
`readDiscreteInputs(address, count) -> std::vector<bool>`) would let a second backend exist
without touching consumers, and is worth doing eventually; it was kept out of the
extraction to keep that diff mechanical.

## Usage

```cpp
#include "rover_modbus_driver/infrastructure/modbus_tcp_client_factory.hpp"

using namespace rover::transport::modbus;

ClientSettings settings;
settings.host                      = "192.168.88.11";
settings.port                      = 502;
settings.connection_retry_count    = 0;     // 0 = retry forever
settings.connection_retry_delay_ms = 1000;

// Logs to /rosout under "RoverModbus" unless you pass your own LoggerPort.
std::unique_ptr<DiscreteIoPort> client = makeModbusTcpDiscreteIoClient(settings);

const uint16_t pressed = client->readDiscreteContact(ContactInfo{Contact::CONTACT_0});
client->writeDiscreteCoil(CoilInfo{Coil::COIL_1, true, /*is_coil_engage_allowed=*/true}, true);
```

To read a range in one round-trip, use the batched reads. They return exactly `count` values
(element `i` is address `first + i`), with the reply's byte padding dropped:

```cpp
const std::vector<bool> coils = client->readDiscreteCoils(Coil::COIL_0, 20);        // FC1
const std::vector<bool> contacts = client->readDiscreteContacts(Contact::CONTACT_0, 1);  // FC2
```

On the Portenta PLC IDE, keep every batched read within one of its memory areas and at
most 8 coils:
- A range that crosses an area boundary (e.g. its Digital Outputs at 0..7 and Programmable
  DIO at 8..19) is served from the first area only, and the rest comes back `false` with no
  error.
- A read of more than 8 coils is answered with `byte_count = 2` but only one data byte.
  `MB::ModbusResponse` now rejects any reply shorter than its byte count (it used to decode
  bytes from past the end of the frame), so such a read throws `MB::ModbusException`.

The unit id is fixed at 255 (`ModbusDiscreteIoClient::kModbusDeviceId`), and each
`Contact`/`Coil` enum value **is** its Modbus address. `COIL_8..COIL_19` are the Portenta
Machine Control's programmable DIO00..DIO11 (PLC IDE "Modbus Coil 9..20").

Reads report failure by throwing `MB::ModbusException`, not by returning a sentinel - see
the note on `kDiscreteReadUnavailable` in `domain/discrete_io_port.hpp`.

## Tests

| Directory | Covers |
|-----------|--------|
| `test/unit/` | What the client puts on the wire - slave id, function codes, addresses, the read-only-coil guard, retry semantics - against a fake transport. No socket, no ROS. |
| `test/integration/` | The connection-failure path against a real, closed loopback port. |
| `test/e2e/` | A full request/response exchange against a real `MB::TCP::Server` on a pid-derived port. |

```bash
colcon test --packages-select rover_modbus_driver && colcon test-result --all
```

The e2e round trip is the first test anywhere in this workspace that proves request
encoding and response decoding agree. It caught one thing immediately: a coil read comes
back as a whole byte, so the decoded response carries 8 cells even when one was requested.

## Provenance

Extracted from `rover_hardware_interface`, where this lived as
`{include,src,test}/rover_modbus/` and was linked `PRIVATE` - reusable by nothing. The
vendored `rover_modbus/README.md` said to revisit its plain-CMake packaging "if a second
consumer inside this workspace ever needs `Modbus_Core`"; this extraction is that trigger,
and `rover_modbus` became an ament package as part of it.

Renamed during the move:

| Before | After |
|--------|-------|
| `RoverModbusInterface` | `DiscreteIoPort` |
| `ModbusConnection` | `ModbusTransportPort` |
| `ModbusTcpConnection` | `ModbusTcpTransport` |
| `RoverModbus` | `ModbusDiscreteIoClient` |
| `ModbusSettings` | `ClientSettings` |

`rover_hardware_interface`'s `rover_safety_controller_types.hpp` carries using-declarations
for the old spellings so its coil table kept compiling unchanged.

Settings are still parsed from the URDF `<ros2_control>` hardware parameters by
`RoverA1System` - that is a ros2_control plugin concern and stayed there. The URDF is
unchanged.
