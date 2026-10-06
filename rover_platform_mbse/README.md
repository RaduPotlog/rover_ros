# rover_platform_mbse

A model-based systems engineering (MBSE) project for the **rover_ros platform** of
Rover A1, built in MATLAB, Simulink, System Composer and Requirements Toolbox (R2026a).
It is not a ROS package (`COLCON_IGNORE`).

It combines two inputs:

- the system requirements `system/ROVER-A1-ROVER_ROS_SYS-SR-V-002.xlsx` (SYS-SR-001 … 029);
- reverse engineering of the rover_ros packages. Every value cites `path:line` in
  `src/rover_ros`.

From them it produces:

- a software architecture;
- one **software requirements specification (SWRS) per package**;
- executable behaviour models with tests;
- traceability from SYS-SR to SWR to architecture and tests;
- a compliance assessment of the as-built software against the SYS-SR.

**Boundary:** the rover_ros packages. The operator station, RC transmitter, orchestrator
(Nav 2), sensor payload, safety PLC, motor drivers, IMU, BMS and LED panels are
external actors.

## Layout

| Path | What | Edited by |
|------|------|-----------|
| `system/` | SYS-SR workbook: the system-level master | the requirement owner |
| `data/platform_architecture.json` | Components, actors, interfaces, connections (topics, QoS, rates, buses) | hand, with provenance |
| `data/platform_parameters.json` | Every numeric model value, with unit and source line | hand, with provenance |
| `data/swrs/<package>.json` | V-001 seed for each SWRS (schema in `data/swrs/README.md`) | first issue only |
| `data/sys_sr_compliance.json` | Allocation, verdict and assessment for each SYS-SR, plus open questions | hand |
| `architecture/` | `RoverPlatformArch.slx` (7 subsystems, 14 packages, 9 actors), `RoverPlatformInterfaces.sldd`, `RoverPlatformProfile.xml`, `RoverPlatformDeployment.slx`, `RoverPlatformAlloc.mldatx` | generated |
| `behaviour/` | `CommandArbitration`, `EStopLatch`, `SkidSteerKinematics` (as built) and `PlatformModeManager` (**proposed**, since no state machine exists in the code) | generated |
| `requirements/` | `ROVER_ROS_SYS-SR.slreqx` and `SWRS_<PFX>.slreqx` ×14, plus link sets (`*.slmx`) | see master policy |
| `tests/` | `t<Model>.m` simulation tests. Each method names what it verifies on a `% Verifies:` line. | hand |
| `scripts/` | `build_*`, `run_model_tests`, `check_traceability`, `export_*`, `format_exports.py`, and the `+mbse` helpers | hand |
| `export/` | xlsx for review: 14 SWRS, compliance, traceability. `rover_platform_export.sysml` | generated |
| `results/` | `model_test_results.json` (git-ignored) | generated |

SWRS prefixes:

| Prefix | Package | Prefix | Package |
|--------|---------|--------|---------|
| HWI | rover_hardware_interface | LED | rover_led |
| CTL | rover_controller | DIAG | rover_diag_manager |
| DESC | rover_description | LOC | rover_localization |
| MUX | rover_twist_mux | GPSH | rover_gps_heading |
| CRSF | rover_crsf_teleop | BRG | rover_bringup |
| SAF | rover_safety | TRN | rover_transport (+ rover_modbus, rover_utils) |
| BAT | rover_battery | MSG | rover_msgs |

## Master policy

- **SYS-SR**: the xlsx is the master. `build_requirements` updates
  `ROVER_ROS_SYS-SR.slreqx` in place from it on every run. Edit the xlsx, not the set.
- **SWRS**: the `.slreqx` sets are the master from V-001 on. Edit them in the
  Requirements Editor. `build_requirements` never overwrites an existing set unless you
  call `build_requirements('Rebuild', true)` (or `build_all('RebuildRequirements', true)`),
  which recreates the sets from the JSON seeds and discards editor changes.
- **Links**: `build_trace_links` owns the links it creates. They are marked in the
  link description. Links you add by hand in the editor survive a re-run.
- **Architecture and behaviour models** are generated. Change the JSON or the builder
  script, then rebuild. Do not edit the `.slx` files by hand.
- **Values**: never type a number into a model or script. Add it to
  `platform_parameters.json` with its `source` line. Unknowns stay `[TBD per SYS-SR-0xx]`.
  The one ASSUMPTION is that the mode manager uses the LED battery thresholds (40 % / 10 %)
  until SYS-SR-017 is set.

## Rebuild

With the MATLAB MCP server (workspace `.mcp.json`; its start folder is this directory):

```matlab
% evaluate_matlab_code, project_path = \\wsl.localhost\Ubuntu\home\rover-a1\ros2_ws\rover_a1\src\rover_ros\rover_platform_mbse
addpath scripts; build_all
```

Or run the steps one by one: `setup_project`, `build_architecture`,
`build_command_arbitration`, `build_estop_latch`, `build_mode_manager`,
`build_skid_steer`, `build_requirements`, `build_trace_links`, `run_model_tests`,
`export_requirements`, `export_sysml`.

Then, from WSL, style the spreadsheets and validate the SysML export:

```bash
uv run --no-project --with openpyxl python scripts/format_exports.py
# /sysml-validate src/rover_ros/rover_platform_mbse/export
```

Windows MATLAB reads this folder over `\\wsl.localhost`. That path gets no change
notifications, so run `rehash path` after editing files from WSL.

## Model tests and known deviations

Run `run_model_tests()`, or `runtests('tests')` with the project open.

Tests tagged `KnownDeviation` are expected to fail: each one measures a SYS-SR the
as-built design misses. `run_model_tests` errors if an untagged test fails, or if a
tagged test starts passing. In that case the deviation is fixed: remove the tag and
update `data/sys_sr_compliance.json`.

Results as of 2026-09-27: 25 passed, 7 known deviations confirmed.

| Known deviation | Requirement | Measured |
|-----------------|-------------|----------|
| SW E-stop to zero command, worst-case Modbus link | SYS-SR-007 (100 ms) | 240 ms (90 ms with an ideal link) |
| E-stop state publish latency (10 Hz poll → 20 Hz publish) | SYS-SR-023 (100 ms) | 130 ms |
| Command watchdog (0.5 s timeout checked at 50 Hz) | SYS-SR-011 (500 ms) | 520 ms |
| IMU publish rate | SYS-SR-019 (≥ 100 Hz) | 50 Hz |
| Encoder feedback rate | SYS-SR-002 (≥ 50 Hz) | 10 Hz |
| Latch reset while moving | SWR-HWI-010 (SAFETY_CHAIN.md §5) | accepted |
| Boot LED animation | SYS-SR-009, SWR-LED-020 | none defined |

The models verify the **design**, not the running code. The rover_ros gtests and
pytests are listed in each SWR's "Verified By" column.

## Compliance summary (SYS-SR V-002 against rover_ros master)

`export/ROVER-A1-ROVER_ROS_SYS-SR-COMPLIANCE-V-001.xlsx` has the detail, the findings
list and the open questions.

| Verdict | SYS-SR |
|---------|--------|
| Compliant | 004, 005, 008, 020, 021 |
| Partial | 002, 003, 006, 009, 011, 022, 024 |
| Not Met | 007, 016, 019, 023 |
| Gap | 010 (no state machine; proposal in `behaviour/PlatformModeManager.slx`), 017 |
| Blocked (TBD) | 012, 013, 014, 025 |
| Not SW | 001, 015, 018, 026, 027, 028, 029 |

## Traceability

`check_traceability()` requires every SYS-SR that is not "Not SW" to have at least one
derived SWR, every SWR to have a parent (or a derived rationale), and every SWR to have
an implementing architecture component. It also lists the As-Built SWRs verified by
Test for which no test exists anywhere (13 at V-001). Those are verification gaps in
rover_ros.
