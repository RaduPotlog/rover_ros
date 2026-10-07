# rover_transport

The rover's transport layer: a hard fork of
[ros-drivers/transport_drivers](https://github.com/ros-drivers/transport_drivers) v1.2.0,
relayouted into this workspace's Clean Architecture convention.

| Package | Role |
|---------|------|
| [`rover_asio_cmake_module`](rover_asio_cmake_module/) | `find_package(ASIO)` support. No code. |
| [`rover_io_context`](rover_io_context/) | Shared ports, byte bridges and the ASIO thread pool. No ROS dependency. |
| [`rover_udp_driver`](rover_udp_driver/) | UDP <-> `udp_read` / `udp_write`. |
| [`rover_modbus_driver`](rover_modbus_driver/) | Synchronous Modbus TCP client. **Library only - no node, no topics.** |

Upstream's serial driver was forked as `rover_serial_driver` and **removed in 2026-10**: its
only consumer, the ELRS receiver, moved onto the RUTX11 router, which forwards the UART over
UDP to `rover_udp_driver` (see `rover_crsf_teleop`). Recover it from git history if a UART
consumer ever returns.

Each package's README lists what changed from upstream. Several real defects were fixed
along the way - two use-after-frees in the async send paths, a receiver that stopped on a
zero-length datagram, and a `-O0` flag that leaked onto every consumer.

`rover_modbus_driver` is the odd one out and deliberately so: it exports libraries rather
than a bridge node, because Modbus TCP is request/response rather than a byte stream, and its
consumer (the ros2_control safety controller) needs in-process calls rather than a topic
round-trip on the e-stop path. Its README explains the reasoning.

## Provenance

`rover_modbus_driver` is **not** part of the transport_drivers fork described below. It was
extracted from `rover_hardware_interface` and builds on `rover_modbus`, a separate vendored
fork of [Mazurel/Modbus](https://github.com/Mazurel/Modbus) (MIT). Everything else here comes
from transport_drivers:

Forked from [ros-drivers/transport_drivers](https://github.com/ros-drivers/transport_drivers)
**v1.2.0** (Apache-2.0; LeoDrive, The Autoware Foundation, Apex.AI, Trimble, TierIV,
Evan Flynn). Relayouted into this workspace's Clean Architecture layout, renamed to the
`rover_` prefix and re-namespaced from `drivers::` to `rover::transport::`.

**Upstream is no longer merged - this is a hard fork.** Upstream has no release for ROS 2
`lyrical`, which is why it was vendored in the first place; port fixes by hand.

Topic names (`udp_read`, `udp_write`) and parameter names (`ip`, `port`) are
**unchanged from upstream** (`rover_udp_driver`'s `source_ip` is an addition), so upstream documentation still describes the wire interface.

> The exact upstream commit is not recoverable from this tree - it carried no VCS metadata
> and no vcs manifest entry; every `package.xml` read `1.2.0`. Resolve the sha of the
> `1.2.0` tag and record it here.
