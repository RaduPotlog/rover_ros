# rover_io_context

Shared transport primitives. **Carries no ROS dependency** - the drivers that link it do.

## Layers

| Layer | Contents |
|-------|----------|
| `domain/` | `ByteStreamPort` (a UART or a UDP socket) and `BytePublisherPort` (where received bytes go). No ROS, no ASIO, no OS. |
| `application/` | `InboundByteBridge` (bytes off the wire -> publisher) and `OutboundByteBridge` (message -> wire, **only while active**). |
| `infrastructure/` | `IoContext` - the ASIO `io_service` and its thread pool. `AsyncOpGuard` (header-only) - makes a stream's `close()` safe against its own handlers. |

`OutboundByteBridge` is where the "do not drive the hardware from a deactivated node"
rule lives. Upstream checked `PRIMARY_STATE_ACTIVE` inline in each of three subscriber
callbacks, so the single most safety-relevant rule in these drivers had no test; here it
is a plain flag driven from `on_activate` / `on_deactivate` and covered by
`test/unit/test_byte_bridges.cpp`.

`AsyncOpGuard` is why `AsioSerialPort::close()` and `AsioUdpSocket::close()` may be
followed straight away by destroying the receive callback's targets and the stream. ASIO's
close waits neither for a handler already running on another io thread nor for one
completed and still queued. The guard runs every handler it wraps on one strand and counts
it; `closeAndDrain()` runs the close on that strand and returns only once every wrapped
handler has finished. From inside a handler it closes directly. Once the `io_context` has
stopped - the path of a node destroyed while active, e.g. by its component container,
since the node's destructor stops the `IoContext` it owns first - queued handlers will
never run and are not waited for, but a handler still running on another io thread is:
`stop()` does not wait for it. `test/unit/test_async_op_guard.cpp` covers each path.

## Targets

`rover_io_context_core` (domain + application) and `rover_io_context_asio`
(infrastructure). The second is **not** called `_ros` - the house suffix - because this
package deliberately has no ROS dependency and the name would be a lie.

## Changes from upstream beyond the relayout

- The `msg_converters/` directory is gone: the two live `UInt8MultiArray` converters moved
  to `rover_serial_driver`, the two `UdpPacket` ones to `rover_udp_driver`. That is what
  removes the `std_msgs` / `udp_msgs` dependencies from this package.
- `src/msg_converters/std_msgs.cpp` was deleted outright. It was unreferenced, one
  definition matched no declaration, and every body did
  `*reinterpret_cast<int32_t *>(in[0])` - casting a `uint8_t` **value** to a pointer.
- `target_compile_options(io_context PUBLIC "-O0")` was dropped. It forced `-O0` on the
  library and on every consumer that linked it, i.e. both drivers.
- The single `RCLCPP_INFO_STREAM` in the `IoContext` constructor was removed, which is
  what lets the package build without `rclcpp`.
- `common.hpp` (a lone `#include <asio.hpp>`) was folded into `io_context.hpp`.

## Provenance

Forked from [ros-drivers/transport_drivers](https://github.com/ros-drivers/transport_drivers)
**v1.2.0** (Apache-2.0; LeoDrive, The Autoware Foundation, Apex.AI, Trimble, TierIV,
Evan Flynn). Relayouted into this workspace's Clean Architecture layout, renamed to the
`rover_` prefix and re-namespaced from `drivers::` to `rover::transport::`.

**Upstream is no longer merged - this is a hard fork.** Upstream has no release for ROS 2
`lyrical`, which is why it was vendored in the first place; port fixes by hand.

Topic names (`serial_read`, `serial_write`, `udp_read`, `udp_write`) and parameter names
(`device_name`, `baud_rate`, `flow_control`, `parity`, `stop_bits`, `ip`, `port`) are
**unchanged from upstream**, so upstream documentation still describes the wire interface.

> The exact upstream commit is not recoverable from this tree - it carried no VCS metadata
> and no vcs manifest entry; every `package.xml` read `1.2.0`. Resolve the sha of the
> `1.2.0` tag and record it here.
