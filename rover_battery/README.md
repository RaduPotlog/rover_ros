# rover_battery

Decodes BMS telemetry received over UDP and publishes the rover's battery state.

## Interfaces

| Direction | Name | Type |
|-----------|------|------|
| in  | `rover_battery_udp_data` | `udp_msgs/UdpPacket` (from `udp_driver`, 392-byte BMS payload sent by the ESP32 `rover_led_bms_ble_controller`) |
| out | `rover_battery/battery_status` | `sensor_msgs/BatteryState` — used by `rover_safety` |
| out | `rover_battery/charging_status` | `rover_msgs/ChargingStatus` |
| out | `diagnostics` | hardware id `RoverBattery`, tasks `Battery errors`, `Battery status` (voltage, current, SoC, charge, design capacity, temperature, charge state, health, cell min / max) |

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

## Units

The payload comes from the ESP32 firmware `rover_led_bms_ble_controller`, which polls the Daly
BMS over BLE and already decodes the Daly raw units
(`docs/Part 4 - Daly RS485+UART Protocol.pdf`), so pack voltage, current, SoC and temperatures
arrive in V, A, % and °C. `BmsFrame` mirrors that firmware's `BmsData` / `Alarm` byte for byte.

Only a valid 392-byte frame resets the watchdog. The firmware sends an all-zero payload when it
has no BMS data (BLE down, or no BMS reply during a poll cycle); the node logs and ignores it, as
it does wrong-size packets, so a lost BMS ends in the watchdog state after `watchdog_timeout_ms`.

`rover_battery/battery_status` follows `sensor_msgs/BatteryState`:

- `temperature` is the hottest sensor (`tempMax`), so `rover_safety` thresholds see a single hot cell;

- `cell_voltage` in V (converted from the BMS mV);
- `charge` is the BMS residual capacity in Ah (converted from mAh);
- `capacity` is NaN — the BMS does not report the last full capacity; `design_capacity` comes
  from the parameter;
- the watchdog-expired state reports `POWER_SUPPLY_STATUS_UNKNOWN` with NaN `charge` / `capacity`.
