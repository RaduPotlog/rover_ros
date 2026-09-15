# rover_world

Gazebo Sim worlds for the rover simulation. `rover_gazebo` includes this package's launch file;
it can also run on its own to open a world without a robot.

## World Files

- `world/rover_world.sdf` - empty world (default).

## Config Files

- `config/teleop.config` - default Gazebo GUI layout used when `rover_world.launch.py` runs on
  its own. `rover_gazebo` overrides it with its own `teleop.config`, which publishes to the
  namespaced `cmd_vel`.

## Launch Files

- `rover_world.launch.py` - starts Gazebo through `ros_gz_sim`'s `gz_sim.launch.py` with
  `-r -v <gz_log_level> <gz_world>`. Closing Gazebo shuts the launch down.

| Argument | Default | Description |
|----------|---------|-------------|
| `gz_world` | `world/rover_world.sdf` | Absolute path to the SDF world. |
| `gz_gui` | `config/teleop.config` | GUI layout file (empty string: Gazebo default). |
| `gz_headless_mode` | `False` | Server only with headless rendering. |
| `gz_log_level` | `2` | Gazebo console verbosity, `0`–`4`. |

```bash
ros2 launch rover_world rover_world.launch.py
```

The package hook adds `share/rover_world/models` to `GZ_SIM_RESOURCE_PATH` (and the legacy
`IGN_GAZEBO_RESOURCE_PATH` / `GAZEBO_MODEL_PATH`); the package ships no models yet.
