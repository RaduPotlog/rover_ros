# rover_led

Drives the rover's two SK9822 LED panels (front and rear bumper). A controller renders layered
image animations into one RGBA frame per panel. A driver encodes those frames into SK9822
packets, and `rover_udp_driver` sends them to the LED board over UDP.

```
led/set_animation ──► rover_led_controller ──led/channel_<n>_frame──► rover_led_driver
                      (rover_led_container)                           (rover_led_container)
                                                                            │ udp_write/led_channel_<n>
                                                                            ▼
                               rover_udp_led_channel_<n>_sender_node ──UDP──► LED board
```

## Nodes and interfaces

### rover_led_controller (component `rover_led::LedControllerNode`)

| Direction | Name | Type |
|-----------|------|------|
| srv | `led/set_animation` | `rover_msgs/SetLedAnimation` |
| srv | `led/stop_animation` | `rover_msgs/StopLedAnimation`: clears an animation (and its queued copies) from its layer on every segment; fails if it isn't playing |
| pub | `led/channel_<n>_frame` | `sensor_msgs/Image` (`rgba8`, one row per panel) at `controller_frequency` |
| pub | `led/animations` | `rover_msgs/LedAnimationCatalog`, latched, once after loading |
| pub | `led/state` | `rover_msgs/LedState` (what every layer of every segment plays), latched, at `state_publish_rate` |
| pub | `diagnostics` | hardware id `Bumper Led`: `Led controller status` + render rate |

Parameters (`src/led_controller_parameters.yaml`): `animations_config_path` (required),
`controller_frequency` (50 Hz), `state_publish_rate` (5 Hz).

### rover_led_driver (lifecycle component `rover_led::LedDriverNode`)

| Direction | Name | Type |
|-----------|------|------|
| sub | `led/channel_<n>_frame` | `sensor_msgs/Image` |
| pub | `udp_write/led_channel_<n>` | `udp_msgs/UdpPacket` (SK9822 frame) |
| srv | `led/set_brightness` | `rover_msgs/SetLedBrightness` (0.0–1.0); also updates the `global_brightness` parameter, so it survives a reconfigure |
| pub | `led/brightness` | `std_msgs/Float32`, latched, the brightness in effect (on activation and after each change) |
| client | `hardware/led_control_enable` | `std_srvs/SetBool`, only with `led_control_handshake` |
| pub | `diagnostics` | hardware id `Bumper Led`: `Led driver status` |

Lifecycle transitions:
- **configure** reads parameters and creates the encoders, topics and services.
- **activate** obtains LED control, clears the LEDs and starts forwarding frames. With
  `led_control_handshake`, frames are ignored until `hardware/led_control_enable` grants
  control.
- **deactivate** clears the LEDs and releases control.

With `autostart` (default) the node activates itself.

Parameters (`src/led_driver_parameters.yaml`, values in `config/rover_a1_driver.yaml`):

| Name | Default | Description |
|------|---------|-------------|
| `frame_timeout` | `0.1` | Frames older than this [s] are dropped |
| `global_brightness` | `1.0` | Initial brightness, `[0.0, 1.0]` |
| `channel_1_num_led` / `channel_2_num_led` | `24` (config: `40`) | LEDs per panel |
| `led_control_handshake` | `false` | Request control from `hardware/led_control_enable` on activation |
| `autostart` | `true` | Configure and activate on startup |

### rover_udp_led_channel_{1,2}_sender_node (`rover_udp_driver`)

These are lifecycle `rover_udp_sender_node` instances. Each one subscribes to
`udp_write/led_channel_<n>` and sends to the address in
`config/rover_a1_udp_led_channel_<n>.yaml` (channel 1 → `192.168.77.202`, channel 2 → `192.168.77.201`, port `3333`).

## Animations

`config/rover_a1_animations.yaml` describes the hardware and the animation catalog. It is a port
of Husarion Panther's `panther_animations.yaml`: the front bumper is channel 1, the rear bumper is
channel 2, and each bumper is one full-width segment (the rear one reversed). Front and rear play
different images for the same state, and directional animations such as blinkers work.

```yaml
panels:            # physical strips: UDP channel + LED count
  - channel: 1
    number_of_leds: 40
segments:          # virtual strips on a panel; a reversed range runs backwards
  - name: front
    channel: 1
    led_range: 0-39
  - name: rear
    channel: 2
    led_range: 39-0
segments_map:      # named groups of segments
  all: [front, rear]
  front: [front]
  rear: [rear]
led_animations:
  - id: 0          # must match the rover_msgs/LedAnimation constant
    name: E_STOP
    priority: 3    # layer, see below
    animations:
      - type: rover_led::ImageAnimation
        segments: front
        animation:
          image: $(find rover_led)/animations/rover_a1/estop_front.png
          duration: 6
      - type: rover_led::ImageAnimation
        segments: rear
        animation:
          image: $(find rover_led)/animations/rover_a1/estop_rear.png
          duration: 6
```

Every segment has four layers, indexed by `priority`. They are composited with `ERROR` on top,
and transparent pixels show the layer underneath:

| priority | Layer | Behaviour |
|---------:|-------|-----------|
| 0 | `ERROR` | Single animation, optionally repeating |
| 1 | `ALERT` | FIFO queue, never repeats; requests are rejected while the queue is full |
| 2 | `INFO` | Single animation, optionally repeating |
| 3 | `STATE` | Single animation, optionally repeating |

Animation types (pluginlib, `plugins.xml`, base `rover_led::Animation`):

| Type | Keys |
|------|------|
| `rover_led::ImageAnimation` | `image`, `duration` [s], optional `repeat` (1), `color` (`0xRRGGBB`: the image is converted to greyscale and recoloured) |
| `rover_led::MovingImageAnimation` | the above, plus `center_offset`, `object_width`, `start_offset`, `splash_duration`, `image_mirrored`, `position_mirrored`, optional `default_image_position` (0–1). The request `param` (0–1) sets the image position, e.g. the battery level. |

Images live in `animations/rover_a1/`. An image is resized to the segment: each row is one frame
and each column is one LED. `$(find <pkg>)` is resolved at load time.

To add an animation, add an entry with a new `id` to the YAML. For callers that use named ids,
also add the constant to `rover_msgs/msg/LedAnimation.msg`. Then request it:

```bash
ros2 service call /rover/led/set_animation rover_msgs/srv/SetLedAnimation \
  "{animation: {id: 1, param: ''}, repeating: true}"
ros2 service call /rover/led/stop_animation rover_msgs/srv/StopLedAnimation "{id: 1}"
ros2 service call /rover/led/set_brightness rover_msgs/srv/SetLedBrightness "{data: 0.5}"
ros2 topic echo /rover/led/state
```

`rover_safety`'s `rover_led_safety_node` is the normal client of `led/set_animation`.

## Launch Files

- `rover_led.launch.py` - `rover_led_container` (controller + driver) and the two UDP senders.

| Argument | Default | Description |
|----------|---------|-------------|
| `namespace` | `$ROVER_NAMESPACE`, else empty | Namespace of all nodes. |
| `robot_model` | `$ROBOT_MODEL_NAME`, else `rover_a1` | Selects `config/<robot_model>_*.yaml`. |
| `animations_config_path` | `config/<robot_model>_animations.yaml` | Animation catalog. |
| `common_dir_path` | empty | If set, the default animations file is read from `<common_dir_path>/rover_led/config/`. |
| `use_sim` | `False` | `True` skips `rover_led_driver` (the controller still publishes frames). |
| `log_level` | `INFO` | Logging level. |

## Layout

```
domain/          Animation (+ Image/MovingImage), LedPanel, LedSegment and its layers,
                 SegmentConverter, Sk9822FrameEncoder, AnimationCatalog/Factory ports - no ROS
application/     SetAnimation, RenderTick, GetLedState, EncodeFrame, SetBrightness use cases
infrastructure/  LedControllerNode, LedDriverNode, controller diagnostics,
                 YAML config loader, pluginlib animation factory
```

## Tests

```bash
colcon test --packages-select rover_led && colcon test-result --all --verbose
```

- `test/unit/` covers the domain, the use cases, the YAML loader and the diagnostics.
- `test/integration/test_led_driver_node.cpp` runs the driver node: its lifecycle, frame
  forwarding and dropping malformed frames.
