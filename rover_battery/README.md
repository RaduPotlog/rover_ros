# rover_battery

Decodes BMS telemetry received over UDP and publishes the rover's battery state.

## Interfaces

| Direction | Name | Type |
|-----------|------|------|
| in  | `/rover_battery_udp_data` | `udp_msgs/UdpPacket` (from `udp_driver`, 392-byte BMS payload) |
| out | `rover_battery/battery_status` | `sensor_msgs/BatteryState` — used by `rover_safety` |
| out | `rover_battery/charging_status` | `rover_msgs/ChargingStatus` |
| out | `/diagnostics` | hardware id `RoverBattery`, tasks `Battery errors`, `Battery status` |

If no packet arrives within `watchdog_timeout_ms`, the node publishes a state with
`present: false` and `POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE`.

### Parameters (`rover_battery_node`, see `config/rover_battery.yaml`)

| Name | Default | Description |
|------|---------|-------------|
| `design_capacity` | `40.0` | Pack design capacity [Ah] |
| `serial_number` | `224KA141600043` | Reported serial number |
| `watchdog_timeout_ms` | `10000` | Max time between BMS packets before reporting a watchdog expiry |

## Layout

```
domain/          BmsFrame (wire layout), BatteryReport value types, pure classifier
                 functions, BatteryStatePublisherPort — no ROS dependencies
application/     MonitorBatteryUseCase — frame → report, watchdog → stale report
infrastructure/  RoverBatteryNode (composition root, UDP decode, watchdog timer),
                 Ros2BatteryStatePublisher (topics, diagnostics, logs), msg conversions
```

Unit tests (`test/unit/`) cover the domain, the use case, and the message mapping without a
running ROS graph. `test/integration/test_rover_battery_node.cpp` runs the real node in-process:
valid and wrong-size packets, the watchdog, and parameter range validation.
Run everything with `colcon test --packages-select rover_battery` (configure with `-DBUILD_TESTING=ON`).

## Known issues (not yet fixed — need BMS documentation / rover_safety review)

These are passed through unchanged from the original implementation, because fixing them
changes the values `rover_safety` reads:

- `cell_voltage` is published in mV; `BatteryState` specifies volts.
- `capacity` is the residual capacity in mAh, while `design_capacity` is in Ah.
- `charge` holds the SoC percentage (0–100), not a charge in Ah.
- `packVoltage` / `packCurrent` are documented by the BMS as 0.1 V / 0.1 A units but are
  published without scaling. Check this against real hardware before changing it.
- The watchdog-expired state reports `POWER_SUPPLY_STATUS_FULL`; `UNKNOWN` would be more
  accurate.
- The `Battery errors` diagnostic keeps showing the last error after the alarms clear.
