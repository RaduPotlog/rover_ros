# crsf_receiver_msg

Messages published by `crsf_receiver`.

## CRSFChannels16

`int32 ch1` … `int32 ch16` hold the raw 11-bit CRSF channel values, unscaled (typically
172–1811, centre 992). `rover_crfs_teleop` maps them to velocity and switch positions (see its
`config/rover_crfs.yaml`).

## CRSFLinkInfo

| Field | Type | Meaning |
|-------|------|---------|
| `uplink_rssi_ant1`, `uplink_rssi_ant2` | `std_msgs/UInt8` | Uplink RSSI per antenna (dBm × -1) |
| `uplink_status` | `std_msgs/UInt8` | Uplink link quality (packet success rate, %) |
| `uplink_snr` | `std_msgs/UInt8` | Uplink SNR (dB) |
| `active_antenna` | `std_msgs/UInt8` | Diversity antenna in use (0 = antenna 1, 1 = antenna 2) |
| `rf_mode` | `std_msgs/UInt8` | RF mode enum (packet rate) |
| `uplink_tx_power` | `std_msgs/UInt8` | TX power enum (0 = 0 mW, 10, 25, 100, 500, 1000, 2000 mW) |
| `downlink_rssi` | `std_msgs/UInt8` | Downlink RSSI (dBm × -1) |
| `downlink_status` | `std_msgs/UInt8` | Downlink link quality (%) |
| `downlink_snr` | `std_msgs/Int8` | Downlink SNR (dB) |

Fields are wrapped `std_msgs` types, so read them as e.g. `msg.uplink_status.data`.
