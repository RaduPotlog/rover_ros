# rover_world

Gazebo Sim worlds for the rover simulation. `rover_gazebo` includes this package's launch file;
it can also run on its own to open a world without a robot.

## World Files

- `world/rover_world.sdf` - the default world: a 25 × 25 m ground plane enclosed by four 1 m
  walls (a 22 × 22 m square), with four boxes (0.8–1.2 m tall) and three 1.5 m pillars as
  static obstacles, so the lidar, costmaps and SLAM have structure to work with. It also
  sets `<spherical_coordinates>` for the simulated GNSS.
- `world/follow_me_world.sdf` - the same world plus a walking person (Fuel's walking actor) for
  follow-me: it stands 25 s at (1.5, -2), 1.5 m ahead of the spawned rover, then walks a loop
  round the east half. Needs the depth camera (`ROVER_USE_CAMERA=true`). The first start
  downloads the actor's mesh from Fuel (cached in `~/.gz/fuel` afterwards).

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
