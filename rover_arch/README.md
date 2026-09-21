# rover_arch

Rover firmware architecture.

Not a ROS package — documentation and diagrams only.

| File | Contents |
|------|----------|
| `rover_a1_arch.drawio` | System architecture diagrams, including the PLC safety logic. |
| [`SAFETY_CHAIN.md`](SAFETY_CHAIN.md) | The E-Stop / safety chain in prose: the PLC latch, the Modbus object map, the watchdog heartbeat timing budget, the two safety topics and the rule for reading each, every point where motion is gated, and the known gaps. |
