# rover_utils

Small helpers shared by the rover packages: header-only C++ utilities, and Python helpers for
launch files. It builds no nodes.

## C++ headers

Header-only. Include them as `rover_utils/<header>.hpp`. They come in two halves, and the
namespace says which.

### ROS-free: `namespace rover_utils`

Standard library, POSIX and yaml-cpp only, so a domain or application layer may include them.

| Header | Function | Behaviour |
|--------|----------|-----------|
| `yaml_utils.hpp` | `getYAMLKeyValue<T>(node, key)` | Returns `node[key].as<T>()`; throws `MissingYAMLKeyError` (a `std::runtime_error`) if the key is missing, `std::runtime_error` if it cannot convert. |
| | `getYAMLKeyValue<T>(node, key, default)` | Same, but returns `default` when the key is missing (a failed conversion still throws). |
| `networking_utils.hpp` | `isPortAvailable(port)` | `true` if a TCP socket can bind `port` on all interfaces. |

Used by `rover_led` (`yaml_utils`, including its domain layer), `rover_safety` (`yaml_utils`, and
`networking_utils` to pick a free Groot2 port) and, outside this repo, `rover_mission_manager`
(`networking_utils`).

### ROS-coupled: `namespace rover_utils::ros`

Need rclcpp, std_msgs or rcl_interfaces; for infrastructure code only.

| Header | Function | Behaviour |
|--------|----------|-----------|
| `ros_utils.hpp` | `addNamespaceToFrameID(frame_id, ns)` | `("imu_link", "/rover")` → `"rover/imu_link"`; the root namespace leaves the frame unchanged. |
| | `mergeHeaders(h1, h2)` | Header with the shared `frame_id` and the older stamp; throws if the frame ids differ. |
| | `verifyTimestampGap(h1, h2, max_gap)` | Throws if either stamp is unset or they differ by more than `max_gap` (whole seconds). |
| `shutdown_gate.hpp` | `ShutdownGate(context, on_close)`, `isOpen()` | Closes as soon as shutdown of `context` starts, before the middleware goes away; `on_close` runs first, on the thread calling `rclcpp::shutdown()`. |
| `parameter_utils.hpp` | `describe(description)` | `ParameterDescriptor` with `description` and `read_only = true`. |
| | `describePositive(description, max_value)` | Same, plus one `floating_point_range` from `1e-6` to `max_value`. |

Used by the infrastructure of `rover_led` (`ros_utils` for LED frame ids, `shutdown_gate`),
`rover_diag_manager` and `rover_twist_mux` (`shutdown_gate`), and `rover_battery` /
`rover_gps_heading` (`parameter_utils`).

### Keeping the halves apart

`test_header_layers` (`test/header_layers.cmake`) fails if a header not on its `ROS_HEADERS` list
includes anything but the C++ standard library, POSIX, yaml-cpp or another ROS-free rover_utils
header, or is not in `namespace rover_utils`; and if a listed header is not in
`rover_utils::ros`. A new header is ROS-free until it is added to that list.

```cmake
find_package(rover_utils REQUIRED)
target_include_directories(my_target PUBLIC ${rover_utils_INCLUDE_DIRS})
```

`rover_utils_INCLUDE_DIRS` is this package's include directory and nothing else: rover_utils
exports no dependencies and no targets. A domain target that uses it and links no ROS library
cannot compile a `rover_utils::ros` header. A target that uses those headers links rclcpp,
std_msgs or rcl_interfaces itself.

## Python launch helpers

The helpers install as the `rover_utils` Python package, one module per concern. All but
`version_check` build `launch` actions or substitutions, so they are for launch files only;
`version_check` is plain Python.

| Concern | Module | Helper | Use |
|---------|--------|--------|-----|
| Console messages | `rover_utils.messages` | `welcome_msg(robot_model, serial_number, robot_hw_version, additional_stats={})` | `LogInfo` action printing the rover banner, serial number, versions and links (used by `rover_bringup`). |
| | | `error_msg(text)`, `warning_msg(text)` | Bold red / yellow `LogInfo` actions. |
| | | `ErrorMessages` | Canned texts: `INCORRECT_HW_CONFIG`, `INCORRECT_OS_VERSION`, … |
| Log levels | `rover_utils.logging` | `limit_log_level_to_info(unit, log_level)` | `--log-level` value that follows `log_level` but never goes below `INFO`, and turns `WARNING` into `WARN` (used by `rover_battery`, `rover_controller`, `rover_led`, `rover_safety`). |
| | | `quiet_rmw_zenoh(log_level)` | `--log-level` value capping `rmw_zenoh_cpp` at `ERROR` unless the launch runs at `DEBUG`, `ERROR` or `FATAL` (used by `rover_diag_manager`, `rover_localization`, `rover_twist_mux`). |
| Start-up order | `rover_utils.events` | `ControllersActive`, `start_once_on(event_type, timeout, actions, fallback_msg)` | `rover_controller` emits `ControllersActive`; `rover_bringup` starts the rest of the stack on it, or after `timeout` seconds with a warning. |
| Shutdown | `rover_utils.shutdown` | `shutdown_unless_shutting_down(process_name)` | `on_exit` handler that shuts launch down when a required process exits on its own, but not during a Ctrl-C teardown (used by `rover_controller`, `rover_led`). |
| Version checks | `rover_utils.version_check` | `check_version_compatibility(version, min_required)` | `True` if the first `vX.Y.Z` in `version` is ≥ the one in `min_required` (numbers, major → minor → patch). A string without `vX.Y.Z` (e.g. `1.2.3`) counts as `v0.0.0`; a suffix such as `-rc1` is ignored (used by `rover_bringup`). |

```python
from rover_utils.logging import limit_log_level_to_info

arguments=["--ros-args", "--log-level", log_level,
           "--log-level", limit_log_level_to_info("rcl", log_level)]
```

## Tests

- `test_header_layers`: the ROS-free / ROS-coupled split above.
- `test_yaml_utils`, `test_shutdown_gate`, `test_parameter_utils` (gtest). `ros_utils.hpp` and
  `networking_utils.hpp` have none.
- `test_logging`, `test_events`, `test_messages`, `test_shutdown`, `test_version_check` (pytest).
  `messages.welcome_msg` has none.
