# rover_udp_driver

Lifecycle nodes bridging UDP to ROS 2.

| Executable | Role |
|---|---|
| `rover_udp_receiver_node` | binds a socket, republishes every datagram on `udp_read` |
| `rover_udp_sender_node` | transmits packets received on `udp_write`, **while active** |
| `rover_udp_bridge_node` | both, sharing one `IoContext` |

| | Topic | Type |
|---|---|---|
| pub | `udp_read` | `udp_msgs/UdpPacket` (`QoS(100)`) |
| sub | `udp_write` | `udp_msgs/UdpPacket` (`KeepLast(32)`, best effort) |

Parameters: `ip`, `port`.

In-tree consumers: `rover_battery` (a receiver, `udp_read` -> `rover_battery_udp_data`)
and `rover_led` (two senders, `udp_write` -> `udp_write/led_channel_{1,2}`).

## Layers

| Layer | Contents |
|-------|----------|
| `domain/` | `UdpEndpoint` - a validated ip/port pair. No ROS, no ASIO. |
| `infrastructure/` | `AsioUdpSocket` (implements `ByteStreamPort`), the `UdpPacket` conversions, `Ros2UdpPacketPublisher` (which also does the header stamping), and the two lifecycle nodes. |

## Changes from upstream beyond the relayout

- **Use-after-free fixed in `asyncSend()`**, same defect as the serial driver: upstream
  held an `asio::buffer` over a vector that died with the subscriber callback.
- **The receiver no longer stops on a zero-length datagram.** Upstream only re-armed the
  read inside `if (bytes_transferred > 0 && m_func)`, so a legal zero-length UDP packet
  silently ended reception for good.
- **The receive buffer is no longer resized under the read.** Upstream shrank it to the
  datagram length, handed it to the callback, grew it back, then re-armed with a lambda
  that shrank it *again before dispatching* - so a following datagram could land in a
  two-byte buffer. Length is now passed alongside the buffer instead.
- `bind()` is no longer a separate public step a caller must remember: a `RECEIVER` socket
  binds inside `open()`.
- `send()` / `receive()` return 0 on error. Upstream returned `-1` from a `std::size_t`
  function, which wraps to `SIZE_MAX` and reads as an enormous successful transfer.
- `port` is validated (1..65535). Upstream defaulted it to 0 and never checked.
- `UdpDriver` was dropped for `makeUdpReceiver()` / `makeUdpSender()` factories;
  `visibility_control.hpp` was dropped as unused.
- The three upstream socket tests, which all hardcoded `127.0.0.1:8000` and so could not
  run concurrently, are merged into one file using per-process ports.
- `rclcpp_components` registration was dropped in favour of explicit `main()`s.

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
