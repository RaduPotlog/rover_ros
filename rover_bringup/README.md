# rover_bringup

Top-level launch files that start the whole Rover A1 stack on real hardware, plus the host
setup files the rover computer needs (udev rules, network configuration). For simulation use
`rover_gazebo` instead.

## Launch Files

### `rover_bringup.launch.py`

Starts the rover driver stack on hardware. Run it inside `rovera1-app` (`rover_docker`), or
directly:

```bash
ros2 launch rover_bringup rover_bringup.launch.py
```

It prints the rover banner (`rover_utils.messages.welcome_msg`) and then:

1. Checks `ROBOT_HW_CONFIG_CORRECT` (default `true`). If it is not `true`, it logs
   `INCORRECT_HW_CONFIG` and starts nothing. It keeps the launch alive unless
   `exit_on_wrong_hw:=true`.
2. Warns when `SYSTEM_BUILD_VERSION` is older than the minimum OS version (`v1.0.0`).
3. Starts immediately:
   - `rover_controller` (robot description, `controller_manager`, controllers)
   - `rover_diag_manager` (system diagnostics and the diagnostic aggregator)
4. After a fixed 10 s delay (giving the hardware interface time to come up), starts:
   - `rover_battery`
   - `rover_led`
   - `rover_safety` (skipped with `disable_manager:=True`)
   - `rover_localization` (with `use_ekf:=True` and `fuse_gps:=<use_gps>`)
   - `rover_gps` (RUTX11 NMEA driver, GPS diagnostics, GNSS heading alignment)
   - `rover_crfs_teleop`
   - `rover_twist_mux`

Every included launch file receives `namespace`, `log_level` and, where supported,
`common_dir_path`.

| Argument | Default | Description |
|----------|---------|-------------|
| `namespace` | `$ROVER_NAMESPACE`, else empty | Namespace of every node (the rover uses `rover`). |
| `log_level` | `INFO` | Logging level passed to every package. |
| `common_dir_path` | empty | Directory with per-package config overrides (`<dir>/<package>/config/...`). |
| `disable_manager` | `False` | `True` skips `rover_safety`. |
| `exit_on_wrong_hw` | `false` | Exit instead of idling when the hardware configuration is incorrect. |
| `use_gps` | `$EKF_USE_GPS`, else `false` | `true`: localization fuses wheels + IMU + GPS (dual EKF, `map → odom`). `false`: wheels + IMU only. |

| Environment variable | Default | Effect |
|----------------------|---------|--------|
| `ROVER_NAMESPACE` | empty | Default for `namespace`. |
| `EKF_USE_GPS` | `false` | Default for `use_gps` (set as a balenaCloud variable; `start.sh` normalizes it to `true`/`false`). |
| `ROBOT_MODEL_NAME` / `ROBOT_SERIAL_NO` / `ROBOT_VERSION` | `rover_a1` / `A1-2026-01` / `1.0` | Shown in the banner. |
| `ROBOT_HW_CONFIG_CORRECT` | `true` | Gate for starting the driver stack. |
| `SYSTEM_BUILD_VERSION` | `v1.0.0` | OS version compared against the minimum. |

### `rover_web_bridges.launch.py`

Starts the web bridges under rover-prefixed node names. `rover_docker/rovera1_app/start.sh`
launches it next to the bringup.

| Node | Package | Notes |
|------|---------|-------|
| `rover_foxglove_bridge` | `foxglove_bridge` | Upstream `foxglove_bridge_launch.xml` defaults, websocket port 8765. Used by Foxglove, the network monitor LED page and Cockpit diagnostics. |
| `rover_rosbridge_websocket` | `rosbridge_server` | Port 9090, for ros-mcp-server. |
| `rosapi` | `rosapi` | Kept as `/rosapi`: rosbridge clients call `/rosapi/*`. |

The bridges are not namespaced, so they see the whole graph (`/rover/...` topics included).

```bash
ros2 launch rover_bringup rover_web_bridges.launch.py
```

## Host setup files (`scripts/`)

These files are not installed by CMake. Copy them onto the rover host OS.

| File | Destination | Purpose |
|------|-------------|---------|
| `99-elrs.rules` | `/etc/udev/rules.d/` | Creates the `/dev/ttyELRS` symlink for the ELRS receiver's CP210x USB-UART (`10c4:ea60`). |
| `99-libphidget22.rules` | `/etc/udev/rules.d/` | Grants access to Phidgets USB devices (the IMU, vendor `06c2`). |
| `60-plc-init.yaml` | `/etc/netplan/` | Static `192.168.88.10/24` on the USB Ethernet adapter to the safety controller. |
| `60-switch-init.yaml` | `/etc/netplan/` | Static `192.168.99.10/24` on `eth0`. |
| `60-wifi-init.yaml` | `/etc/netplan/` | `wlan0` Wi-Fi client with DHCP. |
| `balena-eth0` | NetworkManager system connection (balenaOS) | `safety-controller` profile: static `192.168.88.10/24` on `eth0`. |
| `rutx11_gps_nmea_forwarding.sh` | run from any host on the rover LAN | Shows (`show`) or configures (`apply`) the RUTX11 GNSS and its NMEA forwarding over UDP to `rover_gps`. See `rover_gps/README.md`. |

After adding udev rules, run `sudo udevadm control --reload-rules && sudo udevadm trigger`.
After adding netplan files, run `sudo netplan apply`.

## Known limitations

- `60-wifi-init.yaml` stores the Wi-Fi access point credentials in plain text in the repository.
