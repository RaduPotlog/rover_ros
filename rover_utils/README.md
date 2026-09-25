# rover_utils

Small helpers shared by the rover packages: header-only C++ utilities, and Python helpers for
launch files. It builds no nodes.

## C++ headers

The headers install to `include/rover_utils/`. Include them as `rover_utils/<header>.hpp`.

| Header | Namespace | Function | Behaviour |
|--------|-----------|----------|-----------|
| `yaml_utils.hpp` | `rover_utils` | `getYAMLKeyValue<T>(node, key)` | Returns `node[key].as<T>()`; throws `std::runtime_error` if the key is missing or cannot convert. |
| | | `getYAMLKeyValue<T>(node, key, default)` | Same, but returns `default` when the key is missing (a failed conversion still throws). |
| `ros_utils.hpp` | `rover_utils::ros` | `addNamespaceToFrameID(frame_id, ns)` | `("imu_link", "/rover")` → `"rover/imu_link"`; the root namespace leaves the frame unchanged. |
| | | `mergeHeaders(h1, h2)` | Header with the shared `frame_id` and the older stamp; throws if the frame ids differ. |
| | | `verifyTimestampGap(h1, h2, max_gap)` | Throws if either stamp is unset or they differ by more than `max_gap` (whole seconds). |
| `networking_utils.hpp` | `rover_utils` | `isPortAvailable(port)` | `true` if a TCP socket can bind `port` on all interfaces. |

Used by `rover_led` (`yaml_utils` for the animation config, `ros_utils` for LED frame ids) and
`rover_safety` (`networking_utils` to pick a free Groot2 port).

```cmake
find_package(rover_utils REQUIRED)
target_include_directories(my_target PUBLIC ${rover_utils_INCLUDE_DIRS})
```

## Python launch helpers

The helpers install as the `rover_utils` Python package.

| Module | Helper | Use |
|--------|--------|-----|
| `rover_utils.messages` | `welcome_msg(robot_model, serial_number, robot_hw_version, additional_stats={})` | `LogInfo` action printing the rover banner, serial number, versions and links (used by `rover_bringup`). |
| | `error_msg(text)`, `warning_msg(text)` | Bold red / yellow `LogInfo` actions. |
| | `ErrorMessages` | Canned texts: `INCORRECT_HW_CONFIG`, `INCORRECT_OS_VERSION`, … |
| `rover_utils.logging` | `limit_log_level_to_info(unit, log_level)` | `--log-level` value that caps a noisy logger (`rcl`, a controller) at `INFO` when the launch runs at `DEBUG` (used by `rover_battery`, `rover_controller`, `rover_led`, `rover_safety`). |
| `rover_utils.version_check` | `check_version_compatibility(version, min_required)` | `True` if the first `vX.Y.Z` in `version` is ≥ the one in `min_required` (numbers, major → minor → patch). A string without `vX.Y.Z` (e.g. `1.2.3`) counts as `v0.0.0`; a suffix such as `-rc1` is ignored. |

```python
from rover_utils.logging import limit_log_level_to_info

arguments=["--ros-args", "--log-level", log_level,
           "--log-level", limit_log_level_to_info("rcl", log_level)]
```

## Known limitations

- The functions in `ros_utils.hpp` are defined in the header without `inline`. Including it in
  more than one translation unit of the same target causes multiple-definition link errors.
- `ros_utils.hpp`, `networking_utils.hpp` and `messages.welcome_msg` have no tests.
