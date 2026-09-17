# rover_serial_driver

A lifecycle node that owns a UART and bridges it to two topics.

| | Topic | Type |
|---|---|---|
| pub | `serial_read` | `std_msgs/UInt8MultiArray` (`QoS{100}`, reliable) |
| sub | `serial_write` | `std_msgs/UInt8MultiArray` (`KeepLast(32)`, best effort) |

Parameters: `device_name`, `baud_rate`, `flow_control` (`none`|`hardware`|`software`),
`parity` (`none`|`odd`|`even`), `stop_bits` (`"1"`|`"1.0"`|`"1.5"`|`"2"`|`"2.0"`).

Executable: `rover_serial_bridge_node`.

```bash
ros2 run rover_serial_driver rover_serial_bridge_node --ros-args \
  -p device_name:=/dev/ttyUSB0 -p baud_rate:=460800 \
  -p flow_control:=none -p parity:=none -p stop_bits:='"1"'
```

Its only in-tree consumer is `rover_crsf_teleop`, whose launch file starts it as
`rover_crsf_serial_bridge` with `serial_read` remapped to `rc/raw`.

## Layers

| Layer | Contents |
|-------|----------|
| `domain/` | `SerialPortConfig` + the `FlowControl` / `Parity` / `StopBits` enums, and the parsing of the five ROS parameters into them. No ROS, no ASIO. |
| `infrastructure/` | `AsioSerialPort` (implements `ByteStreamPort`), the ASIO option translation, the `UInt8MultiArray` conversions, `Ros2BytePublisher`, and `SerialBridgeNode`. |

The bridging behaviour itself lives in `rover_io_context`'"'"'s application layer.

## Changes from upstream beyond the relayout

- `SerialPortConfig` no longer leaks ASIO. The four `get_*_asio()` members became free
  functions in `infrastructure/asio_serial_options.hpp`, so the domain header compiles
  with no ASIO include.
- The parameter parsing moved out of `SerialBridgeNode::get_params()` into the domain
  layer. Upstream threw `std::invalid_argument` from inside a `catch` that only handled
  `rclcpp::ParameterTypeException`, so a bad `flow_control` string escaped the constructor
  unhandled; now bad values fail `on_configure` cleanly and are unit-tested.
- `baud_rate` is validated. Upstream defaulted it to 0 and never checked, so a missing
  parameter surfaced later as an opaque ASIO error at open time.
- **Use-after-free fixed in `asyncSend()`.** Upstream held an `asio::buffer` over a vector
  owned by the subscriber callback, which went out of scope as soon as the callback
  returned, leaving the async write reading freed memory. The payload is now owned for the
  lifetime of the operation.
- The re-arm lambda in `asyncReceiveHandler` no longer shadows its enclosing parameters
  (`-Wshadow`).
- `toMsg` clamps to the buffer size instead of `memcpy`-ing an unchecked length.
- `SerialDriver` (a one-member `shared_ptr` holder) was dropped for a `makeSerialPort()`
  factory; `visibility_control.hpp` was dropped as unused.
- `rclcpp_components` registration was dropped - nothing in this workspace composes these
  nodes - in favour of an explicit `main()`.
- The port is still closed and never reopened on a read error, matching upstream.
  Auto-reconnect is the obvious follow-up the `ByteStreamPort` split unlocks, and was
  deliberately left out of the relayout.

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
