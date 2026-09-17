# rover_asio_cmake_module

CMake glue for the standalone [ASIO](https://think-async.com/Asio/) library: it puts
`cmake/Modules` on `CMAKE_MODULE_PATH` so dependents can `find_package(ASIO REQUIRED)`,
which sets `ASIO_INCLUDE_DIRS` and `ASIO_DEFINITIONS` (`ASIO_STANDALONE`).

No C++ code, so it has no `domain` / `application` / `infrastructure` layering - there is
nothing to layer. Used by `rover_io_context`, `rover_serial_driver` and
`rover_udp_driver`.

On this machine ASIO comes from `libasio-dev` (`/usr/include/asio.hpp`). The whole package
is a `find_path` plus one compile definition, so it could reasonably be inlined into the
three dependents and deleted; it is kept as a package to stay close to upstream.

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
