# rover_scripts

Helper scripts for a development PC working with Rover A1. This folder has no `package.xml`, so
colcon ignores it.

## `setup_rover_pc.sh`

Writes the **rover-pc setup** block into `~/.bashrc`. The block sets up:
- ROS 2 (`/opt/ros/<distro>/setup.bash`);
- the rover_a1 workspace overlay (sourced only once it is built);
- `ROVER_ROS_BUILD_TYPE` and `ROVER_NAMESPACE`;
- the Zenoh settings that put this PC on the rover's ROS graph;
- `~/.local/bin` and `~/balena/bin` on PATH, and `QT_QPA_PLATFORM=xcb`.

```bash
~/ros2_ws/rover_a1/src/rover_ros/rover_scripts/setup_rover_pc.sh              # defaults below
~/ros2_ws/rover_a1/src/rover_ros/rover_scripts/setup_rover_pc.sh --rover-ip 10.0.0.5 --dry-run
~/ros2_ws/rover_a1/src/rover_ros/rover_scripts/setup_rover_pc.sh --remove
source ~/.bashrc   # or open a new terminal
```

| Option | Default |
|--------|---------|
| `--workspace PATH` | the workspace this script lives in (`<ws>/src/rover_ros/rover_scripts`) |
| `--rover-ip IP` | `192.168.1.201` |
| `--namespace NAME` | `rover` |
| `--distro NAME` | `lyrical` (or the only distro under `/opt/ros`) |
| `--build-type hardware\|simulation` | `hardware` |
| `--domain-id N` | `0` (as on the rover) |
| `--bashrc FILE` | `~/.bashrc` |
| `--dry-run` | print the block and the diff, change nothing |
| `--remove` | delete the block |

- **Idempotent.** The block between `# >>> rover-pc setup >>>` and `# <<< rover-pc setup <<<` is
  replaced in place, and the rest of the file is left byte for byte. If nothing changed, nothing
  is written.
- **Backups.** A backup (`~/.bashrc.bak-YYYYmmdd-HHMMSS`) is made before every change.
- **Broken markers.** Unbalanced or duplicated markers are refused rather than guessed at.
- **Edits.** Don't edit inside the block; re-run the script with other options instead.

### How the PC joins the rover's graph

Every ROS node on the PC is a **direct Zenoh client** of the router in `rover-a1-platform`, via
`ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/<rover-ip>:7447"]'`. No local
router runs. This is deliberate:
- **The simulation stays isolated.** `rover_gazebo/scripts/rover_sim.sh` clears the override and
  starts its own router on `127.0.0.1:7447`. A permanent local router that dialed the rover would
  sit on that same port, be reused by `rover_sim.sh`, and bridge the simulated `/rover` into the
  real rover's `/rover` graph.
- **The trade-off:** with the rover off or unreachable, nodes started from a plain shell cannot
  connect. For the simulation use `rover_sim.sh`; for off-rover `colcon test`, clear
  `ZENOH_CONFIG_OVERRIDE` and run a local router.
