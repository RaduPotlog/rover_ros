# Vendored `rs_driver`

`rs_driver` **1.5.20** — RoboSense's header-only LiDAR driver library, the decoding and UDP
input core of [`rslidar_sdk`](https://github.com/RoboSense-LiDAR/rslidar_sdk).

## What is here

Only `rs_driver/` — the headers. Copied verbatim from
`rover_ros/rover_rslidar_sdk/src/rs_driver/src/rs_driver/`, minus `macro/version.hpp.in`
(the configured `macro/version.hpp` is already present).

Deliberately **not** copied: the upstream `CMakeLists.txt`, `cmake/`, `demo/`, `tool/`,
`test/`, `win/` and `doc/`. That CMake project hardcodes `set(CMAKE_INSTALL_PREFIX /usr/local)`,
forces `-std=c++14`, and ships a generated `cmake/rs_driverConfig.cmake` with absolute paths
baked into one developer's checkout. `rover_rs16_lidar/CMakeLists.txt` wires the headers up
itself as a small `INTERFACE` target instead — see the `rs_driver_vendor` target there for the
compile definitions (`POINT_TYPE_XYZI`, `ENABLE_MODIFY_RECVBUF`, `ENABLE_DIFOP_PARSE`) and the
`pthread` / `pcap` link requirements.

The headers are added with `target_include_directories(... SYSTEM ...)`, so the vendor's own
warnings stay out of our `-Wall -Wextra -Wpedantic` build.

## Re-vendoring

```bash
git clone --branch v1.5.20 https://github.com/RoboSense-LiDAR/rs_driver.git /tmp/rs_driver
rm -rf 3rdparty/rs_driver/rs_driver
cp -r /tmp/rs_driver/src/rs_driver 3rdparty/rs_driver/rs_driver
rm -f 3rdparty/rs_driver/rs_driver/macro/version.hpp.in
```

Then re-check the compile-time feature flags in `CMakeLists.txt` against upstream's
`CMakeLists.txt` options, and rebuild. Note the exact upstream commit of the copy inside
`rover_rslidar_sdk` is **not recoverable** — that repository flattened the `rs_driver`
submodule before the rover fork, so no gitlink SHA survives. The version above comes from
`macro/version.hpp` and `CMakeLists.txt` (`project(rs_driver VERSION 1.5.20)`).

## License

BSD 3-Clause, RoboSense. The per-file headers are intact; see any header in `rs_driver/`.
