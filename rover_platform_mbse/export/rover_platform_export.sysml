  package RoverPlatform {
    library package RoverPlatformProfile {
      metadata def RosTopic {
        attribute topic : $::ScalarValues::String default := "";
        attribute msgType : $::ScalarValues::String default := "";
        attribute qos : $::ScalarValues::String default := "";
        attribute rate : $::ScalarValues::String default := "";
        attribute source : $::ScalarValues::String default := "";
      }
      metadata def RosService {
        attribute service : $::ScalarValues::String default := "";
        attribute srvType : $::ScalarValues::String default := "";
        attribute rate : $::ScalarValues::String default := "";
        attribute source : $::ScalarValues::String default := "";
      }
      metadata def RosPackage {
        attribute 'package' : $::ScalarValues::String default := "";
        attribute swrsPrefix : $::ScalarValues::String default := "";
        attribute 'language' : $::ScalarValues::String default := "";
        attribute lifecycle : $::ScalarValues::String default := "";
        attribute executables : $::ScalarValues::String default := "";
        attribute behaviourModel : $::ScalarValues::String default := "";
      }
      metadata def Ros2Control {
        attribute interfaces : $::ScalarValues::String default := "";
        attribute rate : $::ScalarValues::String default := "";
        attribute source : $::ScalarValues::String default := "";
      }
      metadata def HwBus {
        attribute bus : $::ScalarValues::String default := "";
        attribute address : $::ScalarValues::String default := "";
        attribute rate : $::ScalarValues::String default := "";
        attribute source : $::ScalarValues::String default := "";
      }
      metadata def FunctionalSubsystem {
        attribute label : $::ScalarValues::String default := "";
      }
      metadata def ExternalActor {
        attribute description : $::ScalarValues::String default := "";
      }
      metadata def ExecutionNode {
        attribute description : $::ScalarValues::String default := "";
        attribute source : $::ScalarValues::String default := "";
      }
    }
    library package RoverPlatformInterfaces {
      port def BatteryState {
        out attribute voltage : $::ScalarValues::Real;
        out attribute current : $::ScalarValues::Real;
        out attribute percentage : $::ScalarValues::Real;
        out attribute temperature : $::ScalarValues::Real;
        out attribute power_supply_health : $::ScalarValues::Real;
      }
      port def BoolFlag {
        out attribute data : $::ScalarValues::Real;
      }
      port def ByteStream {
        out attribute length : $::ScalarValues::Real;
      }
      port def Diagnostics {
        out attribute level : $::ScalarValues::Real;
      }
      port def DriveMode {
        out attribute mode : $::ScalarValues::Real;
        out attribute guard : $::ScalarValues::Real;
        out attribute reason : $::ScalarValues::Real;
      }
      port def DriverState {
        out attribute fault : $::ScalarValues::Real;
        out attribute temperature : $::ScalarValues::Real;
        out attribute current : $::ScalarValues::Real;
      }
      port def Imu {
        out attribute orientation_yaw : $::ScalarValues::Real;
        out attribute angular_velocity_z : $::ScalarValues::Real;
        out attribute linear_acceleration_x : $::ScalarValues::Real;
      }
      port def LedAnimationRequest {
        out attribute animation_id : $::ScalarValues::Real;
        out attribute repeating : $::ScalarValues::Real;
      }
      port def ModbusFrame {
        out attribute coils : $::ScalarValues::Real;
        out attribute discrete_inputs : $::ScalarValues::Real;
      }
      port def NavSatFix {
        out attribute latitude : $::ScalarValues::Real;
        out attribute longitude : $::ScalarValues::Real;
        out attribute altitude : $::ScalarValues::Real;
      }
      port def Odometry {
        out attribute x : $::ScalarValues::Real;
        out attribute y : $::ScalarValues::Real;
        out attribute yaw : $::ScalarValues::Real;
        out attribute vx : $::ScalarValues::Real;
        out attribute wz : $::ScalarValues::Real;
      }
      port def SafetyCommandEcho {
        out attribute sw_e_stop_user_button : $::ScalarValues::Real;
        out attribute sw_e_stop_motor_driver_fault : $::ScalarValues::Real;
        out attribute sw_e_stop_latch_reset : $::ScalarValues::Real;
        out attribute cpu_wdg_heartbeat : $::ScalarValues::Real;
      }
      port def SafetyStatus {
        out attribute hw_e_stop_user_button : $::ScalarValues::Real;
        out attribute motor_contactor_engaged : $::ScalarValues::Real;
        out attribute latch_active : $::ScalarValues::Real;
        out attribute latch_cause : $::ScalarValues::Real;
        out attribute link_healthy : $::ScalarValues::Real;
      }
      port def SystemStatus {
        out attribute cpu_percent : $::ScalarValues::Real;
        out attribute cpu_temp : $::ScalarValues::Real;
        out attribute ram_percent : $::ScalarValues::Real;
        out attribute disk_percent : $::ScalarValues::Real;
      }
      port def Transform {
        out attribute x : $::ScalarValues::Real;
        out attribute y : $::ScalarValues::Real;
        out attribute yaw : $::ScalarValues::Real;
      }
      port def TriggerService {
        out attribute success : $::ScalarValues::Real;
      }
      port def TwistStamped {
        out attribute linear_x : $::ScalarValues::Real;
        out attribute angular_z : $::ScalarValues::Real;
        out attribute stamp : $::ScalarValues::Real;
      }
      port def WheelCommand {
        out attribute fl : $::ScalarValues::Real;
        out attribute fr : $::ScalarValues::Real;
        out attribute rl : $::ScalarValues::Real;
        out attribute rr : $::ScalarValues::Real;
      }
      port def WheelState {
        out attribute position : $::ScalarValues::Real;
        out attribute velocity : $::ScalarValues::Real;
        out attribute effort : $::ScalarValues::Real;
      }
    }
    package SWRS_SAF {
      requirement <'SWR-SAF-001'> 'Interface: The rover_safety shall, in rover_safety_node, subscribe rover_battery/battery_status (s...' {
        doc /* The rover_safety shall, in rover_safety_node, subscribe rover_battery/battery_status (sensor_msgs/BatteryState, KeepLast(1), reliable), hardware_interface/safety_command_echo (rover_msgs/SafetyCommandEcho, KeepLast(1), reliable, volatile), system_status (rover_msgs/SystemStatus, KeepLast(1), reliable) and hardware_interface/rover_driver_state (rover_msgs/RoverDriverState, depth 10), and act as a client of hardware_interface/sw_user_e_stop_set (std_srvs/Trigger). */
      }
      requirement <'SWR-SAF-002'> 'Safety: The rover_safety shall, in rover_safety_node, classify a battery reading with health WA...' {
        doc /* The rover_safety shall, in rover_safety_node, classify a battery reading with health WATCHDOG_TIMER_EXPIRE, DEAD or OVERVOLTAGE as verdict TRIP_E_STOP. */
      }
      requirement <'SWR-SAF-003'> 'Safety: The rover_safety shall, in rover_safety_node, for a battery reading with health OVERHEA...' {
        doc /* The rover_safety shall, in rover_safety_node, for a battery reading with health OVERHEAT, return verdict SHUTDOWN above battery.temp.fatal (60.0 degC) and TRIP_E_STOP above battery.temp.critical (50.0 degC), and no action otherwise; a high temperature without OVERHEAT health shall not trigger an action. */
      }
      requirement <'SWR-SAF-004'> 'Parameter: The rover_safety shall, in rover_safety_node, reject battery.temp.critical and battery....' {
        doc /* The rover_safety shall, in rover_safety_node, reject battery.temp.critical and battery.temp.fatal outside [-20.0, 100.0] degC and shall fail construction unless critical < fatal. */
      }
      requirement <'SWR-SAF-005'> 'Functional: The rover_safety shall, in rover_safety_node, tick the RoverSafety behavior tree at tim...' {
        doc /* The rover_safety shall, in rover_safety_node, tick the RoverSafety behavior tree at timer_frequency (10.0 Hz), calling hardware_interface/sw_user_e_stop_set when the battery verdict is TRIP_E_STOP and the software E-Stop is not already set, and raising SignalShutdown with the verdict reason when the verdict is SHUTDOWN. */
      }
      requirement <'SWR-SAF-006'> 'Safety: The rover_safety shall, in rover_safety_node, evaluate and act on the battery verdict a...' {
        doc /* The rover_safety shall, in rover_safety_node, evaluate and act on the battery verdict as soon as a battery reading and the software E-Stop state are available, independently of whether system_status (CPU temperature) has been received. */
      }
      requirement <'SWR-SAF-007'> 'Functional: The rover_safety shall, in rover_safety_node, once a shutdown is started (SignalShutdow...' {
        doc /* The rover_safety shall, in rover_safety_node, once a shutdown is started (SignalShutdown or ~/shutdown), halt the safety tree and tick the RoverShutdown tree at timer_frequency, which trips the software E-Stop (best effort), asks every host in shutdown_hosts_path to shut down (best effort, unreachable hosts skipped), and runs shutdown.command (default: the installed shutdown_ros_controller.sh --no-e-stop with the reason exported) within shutdown.command_timeout (15.0 s). */
      }
      requirement <'SWR-SAF-008'> 'Interface: The rover_safety shall, in rover_safety_node, provide ~/shutdown (std_srvs/Trigger) whe...' {
        doc /* The rover_safety shall, in rover_safety_node, provide ~/shutdown (std_srvs/Trigger) when shutdown.service_enabled is true (shipped: true), replying immediately with success only when a new shutdown was started, refusing while one is in progress or has succeeded, and refusing retries for shutdown.retry_backoff (30.0 s) after a failed attempt. */
      }
      requirement <'SWR-SAF-009'> 'Diagnostics: The rover_safety shall, in rover_safety_node, publish diagnostics under hardware ID Rov...' {
        doc /* The rover_safety shall, in rover_safety_node, publish diagnostics under hardware ID 'Rover Safety' with the tasks 'Safety inputs' (ERROR when an input is older than input_timeout = 5.0 s, WARN when one was never received), 'Battery safety verdict' (ERROR for any verdict other than none, STALE before the first reading), 'Safety behavior tree' and 'Shutdown' (ERROR after a failed power-off). */
      }
      requirement <'SWR-SAF-010'> 'Lifecycle: The rover_safety shall run rover_safety_node and rover_led_safety_node as nav2::Lifecyc...' {
        doc /* The rover_safety shall run rover_safety_node and rover_led_safety_node as nav2::LifecycleNodes that configure and activate themselves when autostart_node is true, and that retry a failed configure every configure_retry_period (5.0 s) until it succeeds. */
      }
      requirement <'SWR-SAF-011'> 'Build: The rover_safety shall, in rover_safety.launch.py, start rover_led_safety_node always a...' {
        doc /* The rover_safety shall, in rover_safety.launch.py, start rover_led_safety_node always and rover_safety_node only when use_sim is false. */
      }
      requirement <'SWR-SAF-012'> 'Safety: The rover_safety shall, in rover_safety_node, evaluate motor-driver temperature from ha...' {
        doc /* The rover_safety shall, in rover_safety_node, evaluate motor-driver temperature from hardware_interface/rover_driver_state and trip the software E-Stop above a driver temperature limit of [TBD] degC. */
      }
      requirement <'SWR-SAF-013'> 'Interface: The rover_safety shall, in rover_led_safety_node, subscribe rover_battery/battery_statu...' {
        doc /* The rover_safety shall, in rover_led_safety_node, subscribe rover_battery/battery_status (depth 10), hardware_interface/safety_status (KeepLast(1), reliable, volatile) and joy (sensor_msgs/Joy, depth 10), tick the RoverLedSafety tree at timer_frequency (10.0 Hz) once e_stop_state and battery_status have been received, and request animations via led/set_animation (rover_msgs/SetLedAnimation) only when the selected animation changes. */
      }
      requirement <'SWR-SAF-014'> 'Functional: The rover_safety shall, in rover_led_safety_node, select the state animation E_STOP whi...' {
        doc /* The rover_safety shall, in rover_led_safety_node, select the state animation E_STOP while SafetyStatus.hw_e_stop_user_button is true, otherwise MANUAL_ACTION while joy button 4 (dead-man) is held, otherwise READY. */
      }
      requirement <'SWR-SAF-015'> 'Functional: The rover_safety shall, in rover_led_safety_node, while discharging, request CRITICAL_B...' {
        doc /* The rover_safety shall, in rover_led_safety_node, while discharging, request CRITICAL_BATTERY below battery.percent.threshold.critical (0.1), LOW_BATTERY from 0.1 up to battery.percent.threshold.low (0.4) repeated every battery.anim_period.low (30.0 s), and BATTERY_NOMINAL at or above 0.4; while charging or full it shall request CHARGER_INSERTED then CHARGING_BATTERY with the percentage rounded to charging_anim_step (0.05), or BATTERY_CHARGED when that rounds to 1.0. */
      }
      requirement <'SWR-SAF-016'> 'Functional: The rover_safety shall, in rover_led_safety_node, request the ERROR animation while the...' {
        doc /* The rover_safety shall, in rover_led_safety_node, request the ERROR animation while the battery status is UNKNOWN (including the BMS watchdog-expired state) or while charging with health OVERHEAT, and NO_ERROR otherwise. */
      }
      requirement <'SWR-SAF-017'> 'Diagnostics: The rover_safety shall, in rover_led_safety_node, publish diagnostics under hardware ID...' {
        doc /* The rover_safety shall, in rover_led_safety_node, publish diagnostics under hardware ID 'Bumper Led' with the tasks 'LED safety inputs' (battery_status and safety_status graded against input_timeout = 5.0 s; joy reported but not graded) and 'LED safety behavior tree'. */
      }
      requirement <'SWR-SAF-018'> 'Functional: The rover_safety shall provide a platform mode manager that maintains exactly one curre...' {
        doc /* The rover_safety shall provide a platform mode manager that maintains exactly one current system state out of Boot, Idle, Teleoperation, Autonomous, Emergency Stop, Fault and Low Battery, and publishes it on a ROS topic with reliable, transient-local QoS (depth 1) on every change. */
      }
      requirement <'SWR-SAF-019'> 'Functional: The rover_safety shall, in its platform mode manager, implement these transitions: Boot...' {
        doc /* The rover_safety shall, in its platform mode manager, implement these transitions: Boot -> Idle once all platform nodes are active and the first fresh SafetyStatus has been received; Idle <-> Teleoperation / Autonomous according to the twist_mux input currently selected (teleop inputs vs nav); any state -> Emergency Stop while any stop condition of SafetyStatus/SafetyCommandEcho is active (HW button, SW button, motor-driver fault, latch); Emergency Stop -> Idle only after the latch has been reset; Boot -> Idle within [TBD per SYS-SR-024] s of power-up. */
      }
      requirement <'SWR-SAF-020'> 'Safety: The rover_safety shall, in its platform mode manager, enter Low Battery when the batter...' {
        doc /* The rover_safety shall, in its platform mode manager, enter Low Battery when the battery state of charge falls below [TBD per SYS-SR-017] % and shall perform a safe stop (trip the software E-Stop) when it falls below [TBD per SYS-SR-017] %, with both thresholds as validated parameters. */
      }
      requirement <'SWR-SAF-021'> 'Safety: The rover_safety shall, in its platform mode manager, enter Fault when the battery verd...' {
        doc /* The rover_safety shall, in its platform mode manager, enter Fault when the battery verdict is not none, when a safety input is older than input_timeout (5.0 s), or when SafetyStatus.link_healthy is false, and shall leave Fault only when all these conditions have cleared. */
      }
      requirement <'SWR-SAF-022'> 'Functional: The rover_safety shall, in rover_led_safety_node, display each SYS-SR-010 state with a ...' {
        doc /* The rover_safety shall, in rover_led_safety_node, display each SYS-SR-010 state with a dedicated rover_msgs/LedAnimation according to a documented state-to-LED mapping - at least Emergency Stop -> E_STOP (for hardware button, software button, motor-driver fault and held latch alike), Idle -> READY, Teleoperation -> MANUAL_ACTION, Autonomous -> AUTONOMOUS_READY/AUTONOMOUS_ACTION, Low Battery -> LOW_BATTERY, Fault -> ERROR - with colour and pattern per [TBD per SYS-SR-009]. */
      }
    }
    package SWRS_TRN {
      requirement <'SWR-TRN-004'> 'Lifecycle: The rover_transport shall implement UdpReceiverNode and UdpSenderNode as ROS 2 managed ...' {
        doc /* The rover_transport shall implement UdpReceiverNode and UdpSenderNode as ROS 2 managed lifecycle nodes. on_configure shall open the socket and return FAILURE if it cannot be opened. on_activate and on_deactivate shall enable and disable data transfer. on_cleanup and on_shutdown shall close the socket and release all publishers and subscriptions so the node can be configured again. */
      }
      requirement <'SWR-TRN-005'> 'Lifecycle: The rover_transport shall, when a transport nodes autostart parameter is true, trigger ...' {
        doc /* The rover_transport shall, when a transport node's autostart parameter is true, trigger configure and then activate once from the executor immediately after construction, without retrying if configure fails. */
      }
      requirement <'SWR-TRN-006'> 'Interface: The rover_transport shall, in UdpReceiverNode, bind a UDP/IPv4 socket to the configured...' {
        doc /* The rover_transport shall, in UdpReceiverNode, bind a UDP/IPv4 socket to the configured ip:port and publish every non-empty received datagram on udp_read as udp_msgs/UdpPacket with QoS depth 100, header.stamp set to the node clock at reception and header.frame_id set to the configured ip. */
      }
      requirement <'SWR-TRN-007'> 'Functional: The rover_transport shall keep the UDP receiver armed after a successful zero-length da...' {
        doc /* The rover_transport shall keep the UDP receiver armed after a successful zero-length datagram, so that later datagrams are still published. */
      }
      requirement <'SWR-TRN-008'> 'Functional: The rover_transport shall recover from a receive error on a UDP transport by re-arming ...' {
        doc /* The rover_transport shall recover from a receive error on a UDP transport by re-arming the receive, and resume publishing without operator action. */
      }
      requirement <'SWR-TRN-009'> 'Interface: The rover_transport shall, in UdpSenderNode, subscribe to udp_write (udp_msgs/UdpPacket...' {
        doc /* The rover_transport shall, in UdpSenderNode, subscribe to udp_write (udp_msgs/UdpPacket, KeepLast(32), best effort) and transmit each message's data as one UDP datagram to the configured ip:port while the node is ACTIVE. */
      }
      requirement <'SWR-TRN-010'> 'Parameter: The rover_transport shall declare the UDP node parameters ip (string), port (int) and a...' {
        doc /* The rover_transport shall declare the UDP node parameters ip (string), port (int) and autostart (bool, default false), and shall fail the configure transition with a logged error when port is outside 1..65535 or the socket cannot be opened or bound. */
      }
      requirement <'SWR-TRN-011'> 'Safety: The rover_transport shall not write any byte to a UDP endpoint while the owning node is...' {
        doc /* The rover_transport shall not write any byte to a UDP endpoint while the owning node is not in the ACTIVE state, and shall count the bytes it discards this way. */
      }
      requirement <'SWR-TRN-012'> 'Performance: The rover_transport shall use a 2048-byte receive buffer for each UDP socket, and shall...' {
        doc /* The rover_transport shall use a 2048-byte receive buffer for each UDP socket, and shall forward only the valid prefix of each read (never more bytes than were received or than the buffer holds). */
      }
      requirement <'SWR-TRN-013'> 'Lifecycle: The rover_transport shall make closing a UDP socket wait until no receive or send handl...' {
        doc /* The rover_transport shall make closing a UDP socket wait until no receive or send handler of that socket is running or queued, and shall drop received data without throwing once the ROS context is shut down or the publish fails. */
      }
      requirement <'SWR-TRN-014'> 'Interface: The rover_transport shall register UdpReceiverNode and UdpSenderNode as rclcpp_componen...' {
        doc /* The rover_transport shall register UdpReceiverNode and UdpSenderNode as rclcpp_components, and shall let several transport nodes share one externally owned IoContext thread pool. */
      }
      requirement <'SWR-TRN-015'> 'Interface: The rover_transport shall provide, in rover_modbus_driver, an in-process synchronous Mo...' {
        doc /* The rover_transport shall provide, in rover_modbus_driver, an in-process synchronous Modbus TCP discrete-I/O client (DiscreteIoPort) that reads discrete inputs (FC2), reads coils (FC1, single or a batched range returning exactly count values) and writes a single coil (FC5), addressed to unit id 255. */
      }
      requirement <'SWR-TRN-016'> 'Safety: The rover_transport shall refuse a coil write whose CoilInfo.is_coil_engage_allowed is ...' {
        doc /* The rover_transport shall refuse a coil write whose CoilInfo.is_coil_engage_allowed is false: no Modbus frame is sent, an error is logged, and this holds even when the link is down. */
      }
      requirement <'SWR-TRN-017'> 'Functional: The rover_transport shall establish the initial Modbus TCP connection with up to connec...' {
        doc /* The rover_transport shall establish the initial Modbus TCP connection with up to connection_retry_count attempts (0 = retry forever), waiting connection_retry_delay_ms between them, and shall throw std::invalid_argument for an empty host and std::runtime_error once the attempts are exhausted. */
      }
      requirement <'SWR-TRN-018'> 'Functional: The rover_transport shall, after any failed Modbus transaction, drop the TCP link, then...' {
        doc /* The rover_transport shall, after any failed Modbus transaction, drop the TCP link, then fail operations fast with std::runtime_error inside a reconnection backoff window and re-dial lazily on the first operation after it. The window starts at connection_retry_delay_ms (minimum 1 ms) and doubles on each failure up to 30 s. The backoff resets when a re-dial succeeds. */
      }
      requirement <'SWR-TRN-019'> 'Performance: The rover_transport shall abort a Modbus transaction whose reply does not arrive within...' {
        doc /* The rover_transport shall abort a Modbus transaction whose reply does not arrive within ClientSettings.response_timeout_ms, which defaults to 500 ms and is configurable by the caller. */
      }
      requirement <'SWR-TRN-020'> 'Performance: The rover_transport shall bound every Modbus TCP connection attempt, initial or re-dial...' {
        doc /* The rover_transport shall bound every Modbus TCP connection attempt, initial or re-dial, by a configurable connect timeout of [TBD per SYS-SR-023] ms, so that dialling a black-holed PLC host cannot block the caller for the kernel SYN-retry period. */
      }
      requirement <'SWR-TRN-021'> 'Diagnostics: The rover_transport shall publish diagnostic status on /diagnostics at 1 Hz or faster f...' {
        doc /* The rover_transport shall publish diagnostic status on /diagnostics at 1 Hz or faster for each UDP transport node (endpoint open/closed, bytes forwarded, bytes dropped while inactive, datagrams rejected by the source filter, last receive error) and for the Modbus link (connected/disconnected, current backoff), reporting ERROR when a transport has closed or the link is down. */
      }
      requirement <'SWR-TRN-022'> 'Build: The rover_transport shall build as ament_cmake packages on the rovers ROS 2 distributio...' {
        doc /* The rover_transport shall build as ament_cmake packages on the rover's ROS 2 distribution (lyrical, [TBD per SYS-SR-020]), keeping rover_io_context free of any ROS dependency and keeping MB:: frame types and sockets out of rover_modbus_driver's domain and application layers, enforced by the core_purity_check test. */
      }
      requirement <'SWR-TRN-023'> 'Safety: The rover_transport shall, in UdpReceiverNode, accept a source_ip parameter (string, de...' {
        doc /* The rover_transport shall, in UdpReceiverNode, accept a source_ip parameter (string, default empty = any source) and, when it is set, drop inside the socket every datagram whose sender address differs from source_ip before it is published on udp_read, count the dropped datagrams (rejectedDatagrams()), log them at the 1st, 2nd, 4th, 8th, ... rejection, keep receiving afterwards, warn at configure when source_ip is empty, and fail the configure transition when source_ip does not parse as an IPv4 address. The receiver shall record each datagram's sender in an endpoint separate from the bound one, so that receiving never overwrites the bound address. */
      }
    }
    package 'ROVER_ROS_SYS-SR' {
      requirement <'SYS-SR-001'> 'Physical: The rover platform shall have a total mass not exceeding 60 kg, including batteries and...' {
        doc /* The rover platform shall have a total mass not exceeding 60 kg, including batteries and excluding payload. */
      }
      requirement <'SYS-SR-002'> 'Drive: The rover platform shall control 4 wheel drive joints in velocity mode using encoder fe...' {
        doc /* The rover platform shall control 4 wheel drive joints in velocity mode using encoder feedback, at a control rate of at least 50 Hz. */
      }
      requirement <'SYS-SR-003'> 'Drive: The rover platform shall implement skid-steer (differential-drive) kinematics that conv...' {
        doc /* The rover platform shall implement skid-steer (differential-drive) kinematics that convert commanded body linear and angular velocity into wheel velocities, and shall publish wheel odometry. */
      }
      requirement <'SYS-SR-004'> 'Performance: The rover platform shall limit linear velocity to a maximum of 1.0 m/s and angular velo...' {
        doc /* The rover platform shall limit linear velocity to a maximum of 1.0 m/s and angular velocity to a maximum of 1.7 rad/s, for commands from any source. */
      }
      requirement <'SYS-SR-005'> 'Command: The rover platform shall accept velocity commands from 4 sources ([TBD] e.g. joystick t...' {
        doc /* The rover platform shall accept velocity commands from 4 sources ([TBD] e.g. joystick teleoperation, remote/web UI, navigation stack, autonomy/docking) and shall execute only the highest-priority active source. */
      }
      requirement <'SYS-SR-006'> 'Safety: The rover platform shall have at least [TBD] hardware emergency stop button(s) that, wh...' {
        doc /* The rover platform shall have at least [TBD] hardware emergency stop button(s) that, when pressed, remove drive power independently of software (stop category 0 or 1 per IEC 60204-1, ISO 13850). */
      }
      requirement <'SYS-SR-007'> 'Safety: The rover platform shall provide a software emergency stop, triggerable through the ROS...' {
        doc /* The rover platform shall provide a software emergency stop, triggerable through the ROS interface, that commands zero velocity to all drive joints within 100 ms. */
      }
      requirement <'SYS-SR-008'> 'Safety: After any emergency stop (hardware or software), the rover platform shall remain stoppe...' {
        doc /* After any emergency stop (hardware or software), the rover platform shall remain stopped until an operator performs an explicit reset; releasing the E-stop alone shall not resume motion. */
      }
      requirement <'SYS-SR-009'> 'HMI: The rover platform shall indicate its current system state on the LED panels according ...' {
        doc /* The rover platform shall indicate its current system state on the LED panels according to a defined state-to-LED mapping (colour and pattern for each state in SYS-SR-010). */
      }
      requirement <'SYS-SR-010'> 'States: The rover platform shall implement a defined system state machine with at least these s...' {
        doc /* The rover platform shall implement a defined system state machine with at least these states: Boot, Idle, Teleoperation, Autonomous, Emergency Stop, Fault, Low Battery. */
      }
      requirement <'SYS-SR-011'> 'Safety: The rover platform shall bring itself to a stop if no valid velocity command is receive...' {
        doc /* The rover platform shall bring itself to a stop if no valid velocity command is received from the active source within 500 ms. */
      }
      requirement <'SYS-SR-012'> 'Safety: The rover platform shall stop if communication with the operator station is lost for mo...' {
        doc /* The rover platform shall stop if communication with the operator station is lost for more than [TBD] s. */
      }
      requirement <'SYS-SR-013'> 'Performance: The rover platform shall limit linear acceleration and deceleration to [TBD] m/s^2 and ...' {
        doc /* The rover platform shall limit linear acceleration and deceleration to [TBD] m/s^2 and angular acceleration to [TBD] rad/s^2. */
      }
      requirement <'SYS-SR-014'> 'Safety: The rover platform shall stop from 1.0 m/s within [TBD] m on flat, dry, hard ground whe...' {
        doc /* The rover platform shall stop from 1.0 m/s within [TBD] m on flat, dry, hard ground when any emergency stop is triggered. */
      }
      requirement <'SYS-SR-015'> 'Power: The rover platform shall operate continuously for at least [TBD] h on a full battery ch...' {
        doc /* The rover platform shall operate continuously for at least [TBD] h on a full battery charge under a [TBD] duty cycle. */
      }
      requirement <'SYS-SR-016'> 'Power: The rover platform shall publish battery voltage, current and state of charge at 1 Hz o...' {
        doc /* The rover platform shall publish battery voltage, current and state of charge at 1 Hz or faster. */
      }
      requirement <'SYS-SR-017'> 'Power: The rover platform shall enter the Low Battery state at [TBD]% state of charge and shal...' {
        doc /* The rover platform shall enter the Low Battery state at [TBD]% state of charge and shall perform a safe stop at [TBD]% state of charge. */
      }
      requirement <'SYS-SR-018'> 'Power: The rover platform shall support battery charging via [TBD: external connector / dockin...' {
        doc /* The rover platform shall support battery charging via [TBD: external connector / docking station] without removing the battery. */
      }
      requirement <'SYS-SR-019'> 'Sensors: The rover platform shall include an IMU that publishes orientation, angular velocity an...' {
        doc /* The rover platform shall include an IMU that publishes orientation, angular velocity and linear acceleration at 100 Hz or faster. */
      }
      requirement <'SYS-SR-020'> 'Software: The rover platform software shall be based on ROS 2 [TBD distribution] and shall use ro...' {
        doc /* The rover platform software shall be based on ROS 2 [TBD distribution] and shall use ros2_control for the drive hardware interface. */
      }
      requirement <'SYS-SR-021'> 'Software: The rover platform shall follow ROS REP-103 (units and conventions) and REP-105 (coordi...' {
        doc /* The rover platform shall follow ROS REP-103 (units and conventions) and REP-105 (coordinate frames: map, odom, base_link). */
      }
      requirement <'SYS-SR-022'> 'Software: The rover platform shall publish diagnostic status for drive motors, battery, E-stop ci...' {
        doc /* The rover platform shall publish diagnostic status for drive motors, battery, E-stop circuit and sensors on /diagnostics at 1 Hz or faster. */
      }
      requirement <'SYS-SR-023'> 'Safety: The rover platform shall publish the state of the hardware and software emergency stops...' {
        doc /* The rover platform shall publish the state of the hardware and software emergency stops on a ROS topic within 100 ms of a change. */
      }
      requirement <'SYS-SR-024'> 'Software: The rover platform shall start all platform software automatically on power-up and reac...' {
        doc /* The rover platform shall start all platform software automatically on power-up and reach the Idle state within [TBD] s. */
      }
      requirement <'SYS-SR-025'> 'Communication: The rover platform shall provide a wireless link to the operator station with a line-of...' {
        doc /* The rover platform shall provide a wireless link to the operator station with a line-of-sight range of at least [TBD] m and a round-trip latency of at most [TBD] ms. */
      }
      requirement <'SYS-SR-026'> 'Physical: The rover platform shall not exceed [TBD] x [TBD] x [TBD] mm (L x W x H).' {
        doc /* The rover platform shall not exceed [TBD] x [TBD] x [TBD] mm (L x W x H). */
      }
      requirement <'SYS-SR-027'> 'Physical: The rover platform shall carry a payload of at least [TBD] kg while still meeting SYS-S...' {
        doc /* The rover platform shall carry a payload of at least [TBD] kg while still meeting SYS-SR-004 and SYS-SR-014. */
      }
      requirement <'SYS-SR-028'> 'Environment: The rover platform shall operate on slopes up to [TBD] degrees on [TBD] terrain types.' {
        doc /* The rover platform shall operate on slopes up to [TBD] degrees on [TBD] terrain types. */
      }
      requirement <'SYS-SR-029'> 'Environment: The rover platform shall operate at ambient temperatures from [TBD] to [TBD] degC and s...' {
        doc /* The rover platform shall operate at ambient temperatures from [TBD] to [TBD] degC and shall meet ingress protection IP[TBD]. */
      }
    }
    package SWRS_BAT {
      requirement <'SWR-BAT-001'> 'Interface: The rover_battery shall subscribe udp_msgs/UdpPacket on rover_battery_udp_data (depth 1...' {
        doc /* The rover_battery shall subscribe udp_msgs/UdpPacket on rover_battery_udp_data (depth 100), fed by the rover_udp_battery_receiver_node bound to ip 192.168.1.201, port 4444. */
      }
      requirement <'SWR-BAT-002'> 'Functional: The rover_battery shall decode only payloads of exactly 392 bytes (385-byte BmsData + 7...' {
        doc /* The rover_battery shall decode only payloads of exactly 392 bytes (385-byte BmsData + 7-byte BmsAlarms) and shall ignore, without publishing and without resetting the watchdog, packets of any other size and all-zero no-data payloads. */
      }
      requirement <'SWR-BAT-003'> 'Interface: The rover_battery shall publish sensor_msgs/BatteryState on rover_battery/battery_statu...' {
        doc /* The rover_battery shall publish sensor_msgs/BatteryState on rover_battery/battery_status (depth 5, reliable, volatile) once for every valid BMS frame received. */
      }
      requirement <'SWR-BAT-004'> 'Functional: The rover_battery shall fill BatteryState with voltage = pack voltage [V], current = pa...' {
        doc /* The rover_battery shall fill BatteryState with voltage = pack voltage [V], current = pack current [A], percentage = BMS SOC / 100, charge = residual capacity [Ah] (mAh / 1000), capacity = NaN, design_capacity = design_capacity parameter [Ah], temperature = hottest sensor [degC], cell_voltage [V] (mV / 1000) and cell_temperature for the reported (clamped) cell and sensor counts, technology LIFE, present = true and serial_number from the parameter. */
      }
      requirement <'SWR-BAT-005'> 'Functional: The rover_battery shall classify battery health from the BMS alarm bits as DEAD on a le...' {
        doc /* The rover_battery shall classify battery health from the BMS alarm bits as DEAD on a level-one pack-voltage-low, cell-voltage-low or SOC-low alarm, otherwise OVERVOLTAGE on a level-two pack-voltage-high or SOC-high alarm, and then override the result with OVERHEAT on a level-one charge/discharge temperature-high alarm or COLD on a level-one charge/discharge temperature-low alarm; GOOD when none apply. */
      }
      requirement <'SWR-BAT-006'> 'Functional: The rover_battery shall report power_supply_status NOT_CHARGING for BMS status 0, CHARG...' {
        doc /* The rover_battery shall report power_supply_status NOT_CHARGING for BMS status 0, CHARGING for status 1 (FULL when SOC >= 100 %), DISCHARGING for status 2 and UNKNOWN otherwise. */
      }
      requirement <'SWR-BAT-007'> 'Interface: The rover_battery shall publish rover_msgs/ChargingStatus on rover_battery/charging_sta...' {
        doc /* The rover_battery shall publish rover_msgs/ChargingStatus on rover_battery/charging_status (depth 5) with every battery state, with charging = (BMS status 1), current and current_battery = pack current, and charger_type WIRED while charging, UNKNOWN otherwise. */
      }
      requirement <'SWR-BAT-008'> 'Safety: The rover_battery shall, when no valid BMS frame has been received for watchdog_timeout...' {
        doc /* The rover_battery shall, when no valid BMS frame has been received for watchdog_timeout_ms (10000 ms), publish a BatteryState with present = false, power_supply_health WATCHDOG_TIMER_EXPIRE, power_supply_status UNKNOWN, NaN percentage, charge and capacity, zeroed cell arrays of the last known size, and the error 'Battery watchdog expired', and repeat it every watchdog_timeout_ms while data stays absent. */
      }
      requirement <'SWR-BAT-009'> 'Parameter: The rover_battery shall declare design_capacity (default 40.0 Ah, range [0.1, 1000.0]),...' {
        doc /* The rover_battery shall declare design_capacity (default 40.0 Ah, range [0.1, 1000.0]), serial_number (default 224KA141600043) and watchdog_timeout_ms (default 10000, range [1, 600000]), and shall fail to start on an out-of-range override. */
      }
      requirement <'SWR-BAT-010'> 'Diagnostics: The rover_battery shall publish diagnostics under hardware ID RoverBattery with Battery...' {
        doc /* The rover_battery shall publish diagnostics under hardware ID 'RoverBattery' with 'Battery errors' (STALE before data, ERROR listing every active BMS alarm name or the watchdog error, OK otherwise) and 'Battery status' (present, charge state, health, voltage, current, SOC, charge, design capacity, temperature, cell count, min/max cell voltage and difference). */
      }
      requirement <'SWR-BAT-011'> 'Build: The rover_battery shall, in rover_battery.launch.py, load rover_battery_node and the UD...' {
        doc /* The rover_battery shall, in rover_battery.launch.py, load rover_battery_node and the UDP receiver as components of one component container (rover_battery_container) with intra-process communication, bringing the lifecycle-managed receiver to ACTIVE through its autostart parameter. */
      }
      requirement <'SWR-BAT-012'> 'Performance: The rover_battery shall publish rover_battery/battery_status at 1 Hz or faster regardle...' {
        doc /* The rover_battery shall publish rover_battery/battery_status at 1 Hz or faster regardless of the BMS packet timing, republishing the latest reading on a fixed-rate timer, and shall enter the watchdog state (SWR-BAT-008) when the latest reading is older than [TBD] s. */
      }
      requirement <'SWR-BAT-013'> 'Interface: The rover_battery shall set BatteryState.header.stamp to the time the BMS frame was rec...' {
        doc /* The rover_battery shall set BatteryState.header.stamp to the time the BMS frame was received (or the watchdog fired) and header.frame_id to a battery frame name. */
      }
      requirement <'SWR-BAT-014'> 'Performance: The rover_battery shall report pack voltage, current and state of charge within 5 % of ...' {
        doc /* The rover_battery shall report pack voltage, current and state of charge within 5 % of a reference meter. */
      }
      requirement <'SWR-BAT-015'> 'Lifecycle: The rover_battery shall implement rover_battery_node as a managed lifecycle node that c...' {
        doc /* The rover_battery shall implement rover_battery_node as a managed lifecycle node that creates its subscription, publishers and watchdog timer in configure/activate and is brought to ACTIVE by an autostart parameter. */
      }
    }
    package SWRS_BRG {
      requirement <'SWR-BRG-001'> 'Lifecycle: The rover_bringup shall start the platform stack only when the environment variable ROB...' {
        doc /* The rover_bringup shall start the platform stack only when the environment variable ROBOT_HW_CONFIG_CORRECT is true (default true); otherwise it shall log INCORRECT_HW_CONFIG, start no platform node, and keep the launch process alive unless exit_on_wrong_hw is true. */
      }
      requirement <'SWR-BRG-002'> 'Diagnostics: The rover_bringup shall log a warning when SYSTEM_BUILD_VERSION (default v1.0.0) is low...' {
        doc /* The rover_bringup shall log a warning when SYSTEM_BUILD_VERSION (default v1.0.0) is lower than the minimum OS version v1.0.0, and continue the start-up. */
      }
      requirement <'SWR-BRG-003'> 'Functional: The rover_bringup shall start rover_controller (robot description, controller_manager a...' {
        doc /* The rover_bringup shall start rover_controller (robot description, controller_manager and the ros2_control controllers) with use_sim False as the first stage of the platform stack. */
      }
      requirement <'SWR-BRG-004'> 'Functional: The rover_bringup shall start rover_diag_manager (system diagnostics and the diagnostic...' {
        doc /* The rover_bringup shall start rover_diag_manager (system diagnostics and the diagnostic aggregator) together with rover_controller in the first stage. */
      }
      requirement <'SWR-BRG-005'> 'Lifecycle: The rover_bringup shall start rover_battery, rover_led, rover_safety, rover_localizatio...' {
        doc /* The rover_bringup shall start rover_battery, rover_led, rover_safety, rover_localization, rover_crsf_teleop and rover_twist_mux exactly once, when rover_controller emits the ControllersActive event (after the rover_imu_broadcaster spawner succeeds) or after controllers_ready_timeout seconds (default 20.0) with a warning if the event never arrives. */
      }
      requirement <'SWR-BRG-006'> 'Safety: The rover_bringup shall start rover_safety in the second stage unless the launch argume...' {
        doc /* The rover_bringup shall start rover_safety in the second stage unless the launch argument disable_manager is True (default False). */
      }
      requirement <'SWR-BRG-007'> 'Functional: The rover_bringup shall start rover_localization with use_ekf True and fuse_gps set fro...' {
        doc /* The rover_bringup shall start rover_localization with use_ekf True and fuse_gps set from the use_gps argument (default $ROVER_USE_GPS, else false). */
      }
      requirement <'SWR-BRG-008'> 'Parameter: The rover_bringup shall pass namespace (default $ROVER_NAMESPACE, else empty), log_leve...' {
        doc /* The rover_bringup shall pass namespace (default $ROVER_NAMESPACE, else empty), log_level (DEBUG/INFO/WARN/ERROR/FATAL, default INFO) and common_dir_path (default empty) to every included launch file. */
      }
      requirement <'SWR-BRG-009'> 'Build: The rover_bringup shall declare run dependencies on rover_controller, rover_localizatio...' {
        doc /* The rover_bringup shall declare run dependencies on rover_controller, rover_localization, rover_diag_manager, rover_twist_mux, rover_crsf_teleop, rover_battery, rover_utils, rover_safety, rover_led, rover_udp_driver, foxglove_bridge, rosbridge_server and rmw_zenoh_cpp for the hardware build type (ROVER_ROS_BUILD_TYPE == hardware). */
      }
      requirement <'SWR-BRG-010'> 'Build: The rover_bringup shall run on the ROS 2 distribution fixed in SYS-SR-020 with rmw_zeno...' {
        doc /* The rover_bringup shall run on the ROS 2 distribution fixed in SYS-SR-020 with rmw_zenoh_cpp as the RMW implementation. */
      }
      requirement <'SWR-BRG-011'> 'Lifecycle: The rover_bringup shall be started automatically at power-up and bring the platform to ...' {
        doc /* The rover_bringup shall be started automatically at power-up and bring the platform to the Idle state within [TBD per SYS-SR-024] s. */
      }
      requirement <'SWR-BRG-012'> 'Interface: The rover_bringup shall start foxglove_bridge as node rover_foxglove_bridge (WebSocket ...' {
        doc /* The rover_bringup shall start foxglove_bridge as node rover_foxglove_bridge (WebSocket port 8765, upstream default) with sysinfo disabled and an asset URI allowlist accepting package:// URIs of the upstream mesh/image extensions in any letter case. */
      }
      requirement <'SWR-BRG-013'> 'Performance: The rover_bringup shall restrict foxglove_bridge to the whitelisted topics (/tf, /tf_st...' {
        doc /* The rover_bringup shall restrict foxglove_bridge to the whitelisted topics (/tf, /tf_static and the listed UI topics under any namespace) and to the whitelisted services, unless ROVER_FOXGLOVE_TOPIC_WHITELIST or ROVER_FOXGLOVE_SERVICE_WHITELIST is set non-empty. */
      }
      requirement <'SWR-BRG-014'> 'Interface: The rover_bringup shall start rosbridge_websocket as node rover_rosbridge_websocket (po...' {
        doc /* The rover_bringup shall start rosbridge_websocket as node rover_rosbridge_websocket (port 9090, upstream default) and rosapi_node as /rosapi. */
      }
      requirement <'SWR-BRG-015'> 'Lifecycle: The rover_bringup shall have an automated launch test that verifies the two-stage start...' {
        doc /* The rover_bringup shall have an automated launch test that verifies the two-stage start order and that all platform nodes are running after start-up. */
      }
    }
    package SWRS_CRSF {
      requirement <'SWR-CRSF-001'> 'Interface: The rover_crsf_teleop shall subscribe the raw CRSF byte stream as udp_msgs/UdpPacket da...' {
        doc /* The rover_crsf_teleop shall subscribe the raw CRSF byte stream as udp_msgs/UdpPacket datagrams on udp_topic (rc/raw_udp) with QoS depth 100, reliable, from the moment it is configured, and shall feed each datagram's data to the stream CRSF parser without assuming that datagram boundaries coincide with CRSF frame boundaries. */
      }
      requirement <'SWR-CRSF-002'> 'Functional: The rover_crsf_teleop shall decode CRSF RC_CHANNELS_PACKED (0x16, 16 x 11-bit channels)...' {
        doc /* The rover_crsf_teleop shall decode CRSF RC_CHANNELS_PACKED (0x16, 16 x 11-bit channels) and LINK_STATISTICS (0x14) frames addressed to the flight controller (0xC8), discard frames with a bad CRC8 or out-of-range length byte, and resynchronise on the next sync byte with a bounded buffer. */
      }
      requirement <'SWR-CRSF-003'> 'Interface: The rover_crsf_teleop shall publish velocity commands as geometry_msgs/TwistStamped wit...' {
        doc /* The rover_crsf_teleop shall publish velocity commands as geometry_msgs/TwistStamped with header.frame_id base_link on teleop_elrs_cmd_vel_stamped through a lifecycle publisher (depth 10) that is active only while the node is ACTIVE. */
      }
      requirement <'SWR-CRSF-004'> 'Performance: The rover_crsf_teleop shall run its control loop every 20 ms (50 Hz) while ACTIVE and p...' {
        doc /* The rover_crsf_teleop shall run its control loop every 20 ms (50 Hz) while ACTIVE and publish one command per tick while the sticks are deflected and the link is healthy. */
      }
      requirement <'SWR-CRSF-005'> 'Functional: The rover_crsf_teleop shall map linear_x_channel (3) to linear.x in [-0.95, 0.95] m/s a...' {
        doc /* The rover_crsf_teleop shall map linear_x_channel (3) to linear.x in [-0.95, 0.95] m/s and angular_z_channel (1, inverted) to angular.z in [-1.5, 1.5] rad/s, clamping raw values to the channel's calibrated range and mapping a stick within channel_deadband (30 counts) of its midpoint to exactly 0.0. */
      }
      requirement <'SWR-CRSF-006'> 'Functional: The rover_crsf_teleop shall shape each stick axis as (1 - e) * m + e * m^3 of the norma...' {
        doc /* The rover_crsf_teleop shall shape each stick axis as (1 - e) * m + e * m^3 of the normalised deflection m, with e = linear_x_expo (0.3) and angular_z_expo (0.5), clamping e to [0, 1] and keeping full deflection at the output limits. */
      }
      requirement <'SWR-CRSF-007'> 'Performance: The rover_crsf_teleop shall scale linear.x and angular.z by the same factor so that |v|...' {
        doc /* The rover_crsf_teleop shall scale linear.x and angular.z by the same factor so that |v| + |w| * effective_track_width / 2 does not exceed max_wheel_rim_speed (1.7 m/s with effective_track_width 1.0204 m). */
      }
      requirement <'SWR-CRSF-008'> 'Parameter: The rover_crsf_teleop shall declare linear_x_out_min/max and angular_z_out_min/max defa...' {
        doc /* The rover_crsf_teleop shall declare linear_x_out_min/max and angular_z_out_min/max defaults no larger in magnitude than 1.0 m/s and 1.7 rad/s and shall refuse to configure with values beyond those limits. */
      }
      requirement <'SWR-CRSF-009'> 'Safety: The rover_crsf_teleop shall, when the commanded velocity becomes zero (stick released, ...' {
        doc /* The rover_crsf_teleop shall, when the commanded velocity becomes zero (stick released, link lost or teleop inhibited), publish zero commands for zero_burst_duration_ms (300 ms) and then publish nothing until a non-zero command is produced. */
      }
      requirement <'SWR-CRSF-010'> 'Safety: The rover_crsf_teleop shall judge the RC link healthy only while an RC channels frame w...' {
        doc /* The rover_crsf_teleop shall judge the RC link healthy only while an RC channels frame was decoded within channel_timeout_ms (200 ms) and, with require_link_stats true, a LINK_STATISTICS frame was decoded within link_stats_timeout_ms (1000 ms) and uplink link quality has not fallen below link_quality_lost_below (30 %) since it was last at or above link_quality_recovered_at (50 %); the link shall be unhealthy before any input. */
      }
      requirement <'SWR-CRSF-011'> 'Safety: The rover_crsf_teleop shall, when the RC link is judged lost while ACTIVE, command zero...' {
        doc /* The rover_crsf_teleop shall, when the RC link is judged lost while ACTIVE, command zero velocity (SWR-CRSF-009 burst), stop publishing and ignore the switch channels until the link is healthy again, without triggering the E-Stop. */
      }
      requirement <'SWR-CRSF-012'> 'Safety: The rover_crsf_teleop shall, on a change of the e_stop_channel (5) switch position, cal...' {
        doc /* The rover_crsf_teleop shall, on a change of the e_stop_channel (5) switch position, call hardware_interface/sw_user_e_stop_set (std_srvs/Trigger) when it goes low and hardware_interface/sw_user_e_stop_reset when it goes high, and shall call hardware_interface/sw_e_stop_latch_reset when e_stop_latch_reset_channel (4) goes low, evaluated within one 20 ms control tick of the decoded frame. */
      }
      requirement <'SWR-CRSF-013'> 'Safety: The rover_crsf_teleop shall ignore switch positions for switch_settle_frames (100 ticks...' {
        doc /* The rover_crsf_teleop shall ignore switch positions for switch_settle_frames (100 ticks = 2 s) after start-up and after a calibration inhibit is released, learning the resting position without firing a service call, and shall classify a raw value below channel_switch_threshold (500) as low. */
      }
      requirement <'SWR-CRSF-014'> 'Safety: The rover_crsf_teleop shall keep requesting hardware_interface/sw_user_e_stop_set while...' {
        doc /* The rover_crsf_teleop shall keep requesting hardware_interface/sw_user_e_stop_set while the e_stop_channel switch is low and the request has not been acknowledged with success (service unavailable, pending or refused), instead of dropping the request. */
      }
      requirement <'SWR-CRSF-015'> 'Lifecycle: The rover_crsf_teleop shall be a managed lifecycle node that validates parameters and s...' {
        doc /* The rover_crsf_teleop shall be a managed lifecycle node that validates parameters and subscribes its inputs in configure, starts the 20 ms control loop in activate, publishes one zero command before its publisher is deactivated in deactivate and in shutdown, and self-transitions to ACTIVE when autostart is true. */
      }
      requirement <'SWR-CRSF-016'> 'Build: The rover_crsf_teleop shall, in rover_crsf_teleop.launch.py, load rover_udp_drivers Udp...' {
        doc /* The rover_crsf_teleop shall, in rover_crsf_teleop.launch.py, load rover_udp_driver's UdpReceiverNode as rover_crsf_udp_receiver (ip udp_bind_ip 192.168.1.201, port udp_port 10111, source_ip udp_source_ip 192.168.1.1, all read from the teleop config; udp_read remapped to rc/raw_udp; autostart) and the teleop node into one single-threaded component container (rover_crsf_container) with intra-process communication, and shall not start the UDP receiver when use_sim is true. */
      }
      requirement <'SWR-CRSF-017'> 'Parameter: The rover_crsf_teleop shall fail the configure transition when a channel role is outsid...' {
        doc /* The rover_crsf_teleop shall fail the configure transition when a channel role is outside 1-16, two roles share a channel, channel_switch_threshold is outside the 0-2047 wire range, zero_burst_duration_ms is negative, channel_timeout_ms or link_stats_timeout_ms is not positive, or the link-quality thresholds are inconsistent. */
      }
      requirement <'SWR-CRSF-018'> 'Safety: The rover_crsf_teleop shall start an RC calibration session only while the node is not ...' {
        doc /* The rover_crsf_teleop shall start an RC calibration session only while the node is not ACTIVE, the operator confirms the E-Stop, and a fresh (within e_stop_state_timeout_s = 1.0 s, link healthy) hardware_interface/safety_status shows the physical E-Stop pressed AND the latch active AND the motor contactor open; it shall refuse activation during a session, inhibit teleop for its duration, cancel it after the E-Stop is released for e_stop_grace_s (1.0 s) and after calibration_timeout_s (300 s). */
      }
      requirement <'SWR-CRSF-019'> 'Interface: The rover_crsf_teleop shall provide the calibration services rc/calibration/start (rove...' {
        doc /* The rover_crsf_teleop shall provide the calibration services rc/calibration/start (rover_msgs/StartRcCalibration), rc/calibration/sweep, finish and cancel (std_srvs/Trigger) and rc/calibration/apply (rover_msgs/SetRcCalibration), publish rc/calibration/state (rover_msgs/RcCalibrationState, reliable, transient-local, depth 1) at calibration_state_rate_hz (5.0 Hz) during a session and on every change, and persist an applied calibration to calibration_file. */
      }
      requirement <'SWR-CRSF-020'> 'Interface: The rover_crsf_teleop shall, when publish_rc_topics is true, echo decoded frames on rc/...' {
        doc /* The rover_crsf_teleop shall, when publish_rc_topics is true, echo decoded frames on rc/channels (rover_msgs/RcChannels) and rc/link (rover_msgs/RcLinkStatus) with QoS best-effort, volatile, depth 1, each capped at rc_topics_rate_hz (25 Hz; 0 = uncapped). */
      }
      requirement <'SWR-CRSF-021'> 'Diagnostics: The rover_crsf_teleop shall publish diagnostics under hardware ID RC Receiver with the ...' {
        doc /* The rover_crsf_teleop shall publish diagnostics under hardware ID 'RC Receiver' with the tasks 'RC link', 'E-Stop requests', 'RC channels rate', 'RC UDP link' and 'RC calibration', reporting at most WARN (never ERROR); 'RC channels rate' shall warn when the decoded frame rate deviates by more than rc_channels_rate_tolerance (0.2) from rc_channels_expected_hz (250.0 Hz), and 'RC UDP link' shall warn before the first datagram and when no datagram has arrived for more than 1000 ms. */
      }
      requirement <'SWR-CRSF-022'> 'Safety: The rover_crsf_teleop shall accept RC input only from the configured router address: ro...' {
        doc /* The rover_crsf_teleop shall accept RC input only from the configured router address: rover_crsf_teleop.launch.py shall pass udp_source_ip from the teleop config to the UDP receiver as source_ip, so that datagrams from any other sender are dropped before they are published on rc/raw_udp, and the shipped configuration shall set udp_source_ip to the RUTX11 address (192.168.1.1). A blank udp_source_ip shall keep the router address; only an explicit empty string shall accept any sender, and then the launch file and the teleop node shall each log a warning. */
      }
    }
    package SWRS_CTL {
      requirement <'SWR-CTL-001'> 'Performance: The rover_controller shall configure controller_manager with update_rate 100 Hz and run...' {
        doc /* The rover_controller shall configure controller_manager with update_rate 100 Hz and run the four wheel PID controllers at that rate. */
      }
      requirement <'SWR-CTL-002'> 'Functional: The rover_controller shall chain rover_drive_controller (diff_drive_controller/DiffDriv...' {
        doc /* The rover_controller shall chain rover_drive_controller (diff_drive_controller/DiffDriveController) into one rover_controller/SeededPidController per wheel joint (fl, fr, rl, rr), each commanding that joint's velocity interface and closing the loop on its velocity state, with rl/fl as left wheels and rr/fr as right wheels. */
      }
      requirement <'SWR-CTL-003'> 'Functional: The rover_controller shall provide SeededPidController, a pid_controller::PidController...' {
        doc /* The rover_controller shall provide SeededPidController, a pid_controller::PidController whose on_activate() copies each finite claimed hardware state value into the matching exported state interface, leaving NaN where the hardware has no value yet and skipping any pair whose names do not line up. */
      }
      requirement <'SWR-CTL-004'> 'Parameter: The rover_controller shall configure every wheel PID with p 0.05, i 1.0, d 0.04, feedfo...' {
        doc /* The rover_controller shall configure every wheel PID with p 0.05, i 1.0, d 0.04, feedforward_gain 1.0, u_clamp +/-12.58 rad/s, back_calculation anti-windup, save_i_term false, and i_clamp +/-0.25 on fl/fr and +/-0.33 on rl/rr. */
      }
      requirement <'SWR-CTL-005'> 'Parameter: The rover_controller shall configure rover_drive_controller with wheel_separation 0.617...' {
        doc /* The rover_controller shall configure rover_drive_controller with wheel_separation 0.617 m, wheel_radius 0.1651 m, wheels_per_side 2, wheel_separation_multiplier 1.659, left/right wheel_radius_multiplier 1.0 and update_rate 50 Hz. */
      }
      requirement <'SWR-CTL-006'> 'Interface: The rover_controller shall make rover_drive_controller subscribe to cmd_vel and publish...' {
        doc /* The rover_controller shall make rover_drive_controller subscribe to cmd_vel and publish nav_msgs/Odometry on odometry/wheels every drive update (50 Hz) with odom_frame_id odom, base_frame_id base_footprint, twist covariance diagonal [5.4e-5, 5.4e-5, 0, 0, 0, 1.9e-4] and enable_odom_tf false. */
      }
      requirement <'SWR-CTL-007'> 'Performance: The rover_controller shall produce wheel odometry whose travelled-distance error is <= ...' {
        doc /* The rover_controller shall produce wheel odometry whose travelled-distance error is <= 5 % [TBC per SYS-SR-003] when the rover drives 10 m straight at 0.5 m/s on flat ground. */
      }
      requirement <'SWR-CTL-008'> 'Safety: The rover_controller shall limit rover_drive_controller body velocity to linear.x withi...' {
        doc /* The rover_controller shall limit rover_drive_controller body velocity to linear.x within [-0.95, 0.95] m/s and angular.z within [-1.5, 1.5] rad/s for every command received on cmd_vel. */
      }
      requirement <'SWR-CTL-009'> 'Performance: The rover_controller shall limit rover_drive_controller linear acceleration and deceler...' {
        doc /* The rover_controller shall limit rover_drive_controller linear acceleration and deceleration to 2.7 m/s^2 and angular acceleration and deceleration to 3.74 rad/s^2 in both directions. */
      }
      requirement <'SWR-CTL-010'> 'Safety: The rover_controller shall configure rover_drive_controller with cmd_vel_timeout 0.5 s ...' {
        doc /* The rover_controller shall configure rover_drive_controller with cmd_vel_timeout 0.5 s so that the wheel references are set to zero when no velocity command has been received for 0.5 s. */
      }
      requirement <'SWR-CTL-011'> 'Safety: The rover_controller shall keep the outer-wheel reference at full linear plus full angu...' {
        doc /* The rover_controller shall keep the outer-wheel reference at full linear plus full angular command, plus the wheel PID i_clamp_max, at or below the URDF wheel joint velocity limit (10.958 rad/s). */
      }
      requirement <'SWR-CTL-012'> 'Build: The rover_controller shall keep wheel_radius and wheel_separation in config/wheel_01_co...' {
        doc /* The rover_controller shall keep wheel_radius and wheel_separation in config/wheel_01_controller.yaml equal to rover_description/config/wheel_01.yaml and fail its test suite otherwise. */
      }
      requirement <'SWR-CTL-013'> 'Interface: The rover_controller shall run rover_joint_state_broadcaster (joint_state_broadcaster) ...' {
        doc /* The rover_controller shall run rover_joint_state_broadcaster (joint_state_broadcaster) at update_rate 20 Hz, publishing sensor_msgs/JointState on joint_states for the four wheel joints. */
      }
      requirement <'SWR-CTL-014'> 'Interface: The rover_controller shall run rover_imu_broadcaster (imu_sensor_broadcaster) publishin...' {
        doc /* The rover_controller shall run rover_imu_broadcaster (imu_sensor_broadcaster) publishing sensor_msgs/Imu on imu/data with frame_id /imu_link from sensor /imu, with static diagonal covariances 1.8e-3 (orientation), 1.0e-4 (angular velocity) and 7.6e-4 (linear acceleration). */
      }
      requirement <'SWR-CTL-015'> 'Performance: The rover_controller shall publish imu/data at 100 Hz or faster.' {
        doc /* The rover_controller shall publish imu/data at 100 Hz or faster. */
      }
      requirement <'SWR-CTL-016'> 'Build: The rover_controller shall give every controller an update_rate that divides the contro...' {
        doc /* The rover_controller shall give every controller an update_rate that divides the controller_manager update_rate evenly and does not exceed it. */
      }
      requirement <'SWR-CTL-017'> 'Lifecycle: The rover_controller shall, in rover_controller.launch.py, activate rover_joint_state_b...' {
        doc /* The rover_controller shall, in rover_controller.launch.py, activate rover_joint_state_broadcaster, then rover_drive_controller together with its chained wheel PIDs in one --activate-as-group switch, then rover_imu_broadcaster, each only after the previous spawner exited with code 0, shut the launch down naming the controller and exit code when a spawner fails, and emit ControllersActive only after all three succeeded. */
      }
      requirement <'SWR-CTL-018'> 'Performance: The rover_controller shall, in rover_controller.launch.py, start ros2_control_node with...' {
        doc /* The rover_controller shall, in rover_controller.launch.py, start ros2_control_node with --disable-rosout-logs and a single /diagnostics -> diagnostics remap, and apply controller topic remaps only through each controller's node_options_args. */
      }
      requirement <'SWR-CTL-019'> 'Functional: The rover_controller shall, in rover_controller.launch.py, select the controller config...' {
        doc /* The rover_controller shall, in rover_controller.launch.py, select the controller configuration in the order explicit controller_config_path, then /rover_controller/config/_controller.yaml, then the bundled config/_controller.yaml, replace every "/" placeholder with the namespace prefix, and pass the same resolved file to controller_manager, every spawner and the simulation robot description. */
      }
      requirement <'SWR-CTL-020'> 'Safety: The rover_controller shall make its tuning tools (wheel_step_response, wheel_odom_calib...' {
        doc /* The rover_controller shall make its tuning tools (wheel_step_response, wheel_odom_calibration) publish velocity commands on the cmd_topic parameter (default: the twist_mux input teleop_foxglove_cmd_vel_stamped) and move the rover only when the parameter enable_motion is true (default false), printing the planned schedule otherwise. */
      }
    }
    package SWRS_DESC {
      requirement <'SWR-DESC-001'> 'Build: The rover_description shall generate the rover_a1 robot description from urdf/rover_a1....' {
        doc /* The rover_description shall generate the rover_a1 robot description from urdf/rover_a1.urdf.xacro with the xacro arguments use_sim, namespace, wheel_config_file, battery_config_file, controller_config_file, imu_xyz/imu_rpy, lidar_xyz/lidar_rpy and gps_xyz/gps_rpy. */
      }
      requirement <'SWR-DESC-002'> 'Interface: The rover_description shall define the frame tree base_footprint -> base_link -> body_l...' {
        doc /* The rover_description shall define the frame tree base_footprint -> base_link -> body_link with imu_link, lidar_link, gps_link and the four _wheel_base_link / _wheel_link frames as children of body_link, using REP-103 axes (+x forward, +y left, +z up) and prefixing every frame with "/" when a namespace is set. */
      }
      requirement <'SWR-DESC-003'> 'Functional: The rover_description shall place base_link above base_footprint by tyre_radius minus t...' {
        doc /* The rover_description shall place base_link above base_footprint by tyre_radius minus the wheel axis height above body_link (0.037363 m), with base_link coincident with body_link. */
      }
      requirement <'SWR-DESC-004'> 'Parameter: The rover_description shall take wheel geometry only from config/wheel_01.yaml (tyre_ra...' {
        doc /* The rover_description shall take wheel geometry only from config/wheel_01.yaml (tyre_radius 0.1699 m, wheel_width 0.108 m, wheelbase 0.503 m, wheel_separation 0.617 m, tyre_y_offset -0.00171 m) and mount the fl, fr, rl and rr wheels at (+/-wheelbase/2, +/-wheel_separation/2) in their REP-103 quadrants at a common height, with each tyre collision cylinder |tyre_y_offset| inboard of its joint. */
      }
      requirement <'SWR-DESC-005'> 'Parameter: The rover_description shall model each wheel joint as a continuous joint about +y with ...' {
        doc /* The rover_description shall model each wheel joint as a continuous joint about +y with limit effort 64.5 and velocity 10.958 rad/s, so that a positive joint velocity drives the rover forward. */
      }
      requirement <'SWR-DESC-006'> 'Interface: The rover_description shall declare, with use_sim false, a <ros2_control> system compon...' {
        doc /* The rover_description shall declare, with use_sim false, a system component named "/rover_system_node" using plugin rover_hardware_interface/RoverA1System with the four wheel joints each exposing a velocity command interface and position, velocity and effort state interfaces, and with use_sim true the plugin gz_ros2_control/GazeboSimSystem with the same joints plus the IMU state interfaces. */
      }
      requirement <'SWR-DESC-007'> 'Parameter: The rover_description shall set the RoverA1System safety-link parameters modbus_host 19...' {
        doc /* The rover_description shall set the RoverA1System safety-link parameters modbus_host 192.168.88.11, modbus_port 502, modbus_connection_retry_count 0, modbus_connection_retry_delay_ms 1000, modbus_response_timeout_ms 150, safety_wdg_kick_period_ms 200, safety_io_poll_period_ms 100 and safety_latch_reset_pulse_ms 100. */
      }
      requirement <'SWR-DESC-008'> 'Parameter: The rover_description shall set the RoverA1System drive-train parameters driver_states_...' {
        doc /* The rover_description shall set the RoverA1System drive-train parameters driver_states_update_frequency 20.0, max_rover_driver_initialization_attempts 3, max_rover_driver_activation_attempts 3, the three error-filter counts 2, driver_comm_timeout_ms 300, motor_failsafe_timeout_ms 500, motor_acceleration 4.0, motor_current_limit 15.0, motor_supply_voltage 24.0, motor_torque_constant 0.11, max_rpm_motor_speed 2800, gear_ratio 23.3, gearbox_efficiency 0.70, raw_current_to_amps_scale 0.001 and encoder_resolution 1024. */
      }
      requirement <'SWR-DESC-009'> 'Parameter: The rover_description shall set the E-Stop reset tolerances velocity_command_zero_toler...' {
        doc /* The rover_description shall set the E-Stop reset tolerances velocity_command_zero_tolerance 0.4 rad/s and velocity_state_zero_tolerance 0.05 rad/s. */
      }
      requirement <'SWR-DESC-010'> 'Parameter: The rover_description shall declare, with use_sim false, a <ros2_control> sensor compon...' {
        doc /* The rover_description shall declare, with use_sim false, a sensor component "/rover_imu" using plugin rover_hardware_interface/PhidgetImuSensor with data_interval_ms 8, callback_delta_epsilon_ms 1, Madgwick gain 0.00304 and zeta 0.00151, use_mag false, stateless false, remove_gravity_vector false and world_frame enu, exporting the 10 orientation / angular_velocity / linear_acceleration state interfaces. */
      }
      requirement <'SWR-DESC-011'> 'Functional: The rover_description shall, in rover_load_urdf.launch.py, take the IMU, lidar and GPS ...' {
        doc /* The rover_description shall, in rover_load_urdf.launch.py, take the IMU, lidar and GPS mount poses from the ROVER_{IMU,LIDAR,GPS}_LOCALIZATION_{X,Y,Z} and _ORIENTATION_{R,P,Y} environment variables, falling back to the defaults (IMU xyz (-0.09, 0.0, 0.2), rpy (3.14159, 0.0, 0.0); lidar and GPS all 0.0) for unset or non-finite values. */
      }
      requirement <'SWR-DESC-012'> 'Parameter: The rover_description shall declare body_link inertial mass 39.5373 kg and each wheel l...' {
        doc /* The rover_description shall declare body_link inertial mass 39.5373 kg and each wheel link mass 4.79607 kg (58.7216 kg in total). */
      }
      requirement <'SWR-DESC-013'> 'Interface: The rover_description shall start robot_state_publisher as node rover_state_publisher_n...' {
        doc /* The rover_description shall start robot_state_publisher as node rover_state_publisher_node in the requested namespace with the generated robot_description, frame_prefix "/" and use_sim_time equal to use_sim, when publish_robot_state is true (default). */
      }
      requirement <'SWR-DESC-014'> 'Functional: The rover_description shall, with use_sim true, provide Gazebo sensors on the IMU, lida...' {
        doc /* The rover_description shall, with use_sim true, provide Gazebo sensors on the IMU, lidar and GPS frames: IMU at 50 Hz on imu/data_raw with gaussian noise, a 16-ring gpu_lidar over +/-15 deg with 720 samples, 0.2-20 m range at 10 Hz on lidar, and a navsat sensor at 5 Hz on gps/fix. */
      }
      requirement <'SWR-DESC-015'> 'Performance: The rover_description shall configure the simulated IMU update_rate at 100 Hz or faster.' {
        doc /* The rover_description shall configure the simulated IMU update_rate at 100 Hz or faster. */
      }
      requirement <'SWR-DESC-016'> 'Parameter: The rover_description shall configure the simulated battery (Gazebo LinearBatteryPlugin...' {
        doc /* The rover_description shall configure the simulated battery (Gazebo LinearBatteryPlugin) with capacity 40.0 Ah, initial charge 0.7 of capacity, charging_time 8.0 h, power_load 160.0 W and discharging disabled by default. */
      }
    }
    package SWRS_DIAG {
      requirement <'SWR-DIAG-001'> 'Interface: The rover_diag_manager shall publish a rover_msgs/SystemStatus message on the relative ...' {
        doc /* The rover_diag_manager shall publish a rover_msgs/SystemStatus message on the relative topic system_status (queue depth 10) containing per-core CPU usage, mean CPU load, CPU temperature, RAM usage and disk usage. */
      }
      requirement <'SWR-DIAG-002'> 'Performance: The rover_diag_manager shall sample the system metrics and publish system_status at the...' {
        doc /* The rover_diag_manager shall sample the system metrics and publish system_status at the rate set by the publish_frequency parameter, 5.0 Hz in the shipped configuration. */
      }
      requirement <'SWR-DIAG-003'> 'Parameter: The rover_diag_manager shall declare the parameters cpu_usage_warn_threshold (default 9...' {
        doc /* The rover_diag_manager shall declare the parameters cpu_usage_warn_threshold (default 95.0 %, bounds 0-100), cpu_temperature_warn_threshold (default 80.0 degC, bounds 0-120), ram_usage_warn_threshold (default 90.0 %, bounds 0-100), disk_usage_warn_threshold (default 90.0 %, bounds 0-100) and publish_frequency (default 5.0 Hz, bounds 0.1-10.0) through generate_parameter_library. */
      }
      requirement <'SWR-DIAG-004'> 'Functional: The rover_diag_manager shall re-read the warning thresholds on every sampling tick so t...' {
        doc /* The rover_diag_manager shall re-read the warning thresholds on every sampling tick so that a threshold changed with `ros2 param set` takes effect at the next tick without restarting the node. */
      }
      requirement <'SWR-DIAG-005'> 'Diagnostics: The rover_diag_manager shall publish an OS status diagnostic (hardware ID Rover PC) on ...' {
        doc /* The rover_diag_manager shall publish an 'OS status' diagnostic (hardware ID 'Rover PC') on /diagnostics with one key/value per metric (CPU usage, CPU temperature, RAM usage, disk usage) at the diagnostic_updater period. */
      }
      requirement <'SWR-DIAG-006'> 'Diagnostics: The rover_diag_manager shall grade the OS status diagnostic WARN when any metric is str...' {
        doc /* The rover_diag_manager shall grade the 'OS status' diagnostic WARN when any metric is strictly above its warning threshold, ERROR when any metric cannot be read, and OK otherwise, using the most severe finding as the overall level. */
      }
      requirement <'SWR-DIAG-007'> 'Diagnostics: The rover_diag_manager shall report the OS status diagnostic as STALE with the message ...' {
        doc /* The rover_diag_manager shall report the 'OS status' diagnostic as STALE with the message 'No system sample yet.' until the first metrics sample has been taken. */
      }
      requirement <'SWR-DIAG-008'> 'Functional: The rover_diag_manager shall read CPU temperature from /sys/class/thermal/thermal_zone0...' {
        doc /* The rover_diag_manager shall read CPU temperature from /sys/class/thermal/thermal_zone0/temp (millidegrees, converted to degC with 2 decimals), RAM usage as (total - available)/total from /proc/meminfo, and disk usage as (capacity - available)/capacity of the root filesystem '/', and shall report a metric as unknown (NaN on system_status) when it cannot be read or its total is not positive. */
      }
      requirement <'SWR-DIAG-009'> 'Interface: The rover_diag_manager shall launch a diagnostic_aggregator aggregator_node named rover...' {
        doc /* The rover_diag_manager shall launch a diagnostic_aggregator aggregator_node named rover_diagnostic_aggregator that subscribes to the namespaced diagnostics topic and publishes diagnostics_agg and diagnostics_toplevel_state inside the launch namespace (ROVER_NAMESPACE). */
      }
      requirement <'SWR-DIAG-010'> 'Performance: The rover_diag_manager shall configure the diagnostic aggregator to publish diagnostics...' {
        doc /* The rover_diag_manager shall configure the diagnostic aggregator to publish diagnostics_agg at 1.0 Hz under the root path 'Rover'. */
      }
      requirement <'SWR-DIAG-011'> 'Diagnostics: The rover_diag_manager shall group diagnostic statuses by node-name prefix into the agg...' {
        doc /* The rover_diag_manager shall group diagnostic statuses by node-name prefix into the aggregator paths Computer (rover_diag_manager_node), Drive (rover_hardware_controller, controller_manager), Battery (rover_battery_node), Localization (rover_ekf_node, rover_ekf_global_node, rover_navsat_transform_node), GPS (rover_gps_node, rover_gps_heading_node), Lidar (rover_rs16_lidar_node), Lighting (rover_led_driver, rover_led_controller), Teleop (rover_crsf_teleop_node), Motion (rover_motion_lock_node, rover_twist_mux_node) and Safety (rover_safety_node, rover_led_safety_node). */
      }
      requirement <'SWR-DIAG-012'> 'Diagnostics: The rover_diag_manager shall configure the diagnostic aggregator so that the drive, bat...' {
        doc /* The rover_diag_manager shall configure the diagnostic aggregator so that the drive, battery, E-stop (safety) and sensor (GPS, lidar, IMU) groups report ERROR or STALE when an expected component's diagnostics are missing, rather than silently omitting them. */
      }
      requirement <'SWR-DIAG-013'> 'Parameter: The rover_diag_manager launch file shall accept the arguments log_level (DEBUG/INFO/WAR...' {
        doc /* The rover_diag_manager launch file shall accept the arguments log_level (DEBUG/INFO/WARN/ERROR/FATAL, default INFO), namespace (default $ROVER_NAMESPACE or empty), system_diag_config_path (default config/system_diag.yaml) and diagnostic_aggregator_config_path (default config/diagnostic_aggregator.yaml). */
      }
      requirement <'SWR-DIAG-014'> 'Lifecycle: The rover_diag_manager shall run rover_diag_manager_node as a plain (non-lifecycle) nod...' {
        doc /* The rover_diag_manager shall run rover_diag_manager_node as a plain (non-lifecycle) node that starts sampling on construction, exits with code 1 and a FATAL log if construction fails (for example on an out-of-range parameter), and stops its timer on context shutdown. */
      }
      requirement <'SWR-DIAG-015'> 'Build: The rover_diag_manager shall keep its domain and application layers in a separate stati...' {
        doc /* The rover_diag_manager shall keep its domain and application layers in a separate static library (rover_diag_manager_core) with no ROS or cppuprofile dependency. */
      }
    }
    package SWRS_GPSH {
      requirement <'SWR-GPSH-001'> 'Interface: The rover_gps_heading shall subscribe to sensor_msgs/NavSatFix on the relative topic gp...' {
        doc /* The rover_gps_heading shall subscribe to sensor_msgs/NavSatFix on the relative topic gps/fix with QoS reliable, volatile, keep-last depth 10. */
      }
      requirement <'SWR-GPSH-002'> 'Interface: The rover_gps_heading shall subscribe to nav_msgs/Odometry on the relative topic odom (...' {
        doc /* The rover_gps_heading shall subscribe to nav_msgs/Odometry on the relative topic odom (QoS depth 10) and use its orientation yaw, twist.linear.x and twist.angular.z. */
      }
      requirement <'SWR-GPSH-003'> 'Interface: The rover_gps_heading shall publish sensor_msgs/Imu on gps/heading_imu (QoS depth 10) w...' {
        doc /* The rover_gps_heading shall publish sensor_msgs/Imu on gps/heading_imu (QoS depth 10) with frame_id heading_frame_id, orientation containing only the ENU yaw (x = y = 0), orientation covariance [0] = [4] = 1.0e3 and [8] = yaw std squared, and angular-velocity and linear-acceleration covariance [0] = -1 (not provided). */
      }
      requirement <'SWR-GPSH-004'> 'Functional: The rover_gps_heading shall publish gps/heading_imu only after the alignment has reache...' {
        doc /* The rover_gps_heading shall publish gps/heading_imu only after the alignment has reached the Aligned state and only when publish_heading is true, and then once for every received odom message with yaw = wrap(odom yaw + latched offset). */
      }
      requirement <'SWR-GPSH-005'> 'Functional: The rover_gps_heading shall form one course measurement from each straight segment of a...' {
        doc /* The rover_gps_heading shall form one course measurement from each straight segment of at least alignment.min_segment_length_m (3.0 m) GNSS displacement, as offset = atan2(dNorth, dEast) - circular mean of odom yaw over the segment, adding pi when reversing (vx < 0). */
      }
      requirement <'SWR-GPSH-006'> 'Functional: The rover_gps_heading shall treat motion as straight only when |vx| >= alignment.min_sp...' {
        doc /* The rover_gps_heading shall treat motion as straight only when |vx| >= alignment.min_speed_m_s (0.3 m/s), |yaw rate| <= alignment.max_yaw_rate_rad_s (0.1 rad/s) and the yaw change since segment start is <= alignment.max_yaw_change_rad (0.1 rad), and shall abort the current segment otherwise or when the drive direction changes. */
      }
      requirement <'SWR-GPSH-007'> 'Functional: The rover_gps_heading shall ignore a GNSS fix for alignment (and abort the current segm...' {
        doc /* The rover_gps_heading shall ignore a GNSS fix for alignment (and abort the current segment) when it has no fix status, its horizontal 1-sigma error exceeds alignment.max_horizontal_std_m (5.0 m), the latest odometry is older than alignment.max_odometry_age_s (0.5 s), or the gap to the previous fix exceeds alignment.max_fix_gap_s (2.5 s). */
      }
      requirement <'SWR-GPSH-008'> 'Functional: The rover_gps_heading shall latch the circular mean of the most recent alignment.requir...' {
        doc /* The rover_gps_heading shall latch the circular mean of the most recent alignment.required_segments (3) offsets as the odom-to-ENU yaw offset when their circular standard deviation is <= alignment.max_offset_std_rad (0.1 rad), and keep it until reset. */
      }
      requirement <'SWR-GPSH-009'> 'Functional: The rover_gps_heading shall report the published heading standard deviation as max(min_...' {
        doc /* The rover_gps_heading shall report the published heading standard deviation as max(min_heading_std_rad, latched offset std), with min_heading_std_rad = 0.05 rad by default. */
      }
      requirement <'SWR-GPSH-010'> 'Interface: The rover_gps_heading shall provide the std_srvs/Trigger service gps/reset_heading_alig...' {
        doc /* The rover_gps_heading shall provide the std_srvs/Trigger service gps/reset_heading_alignment that discards the alignment, returns success true with the message 'Heading alignment reset; drive straight to realign.', and immediately republishes the (unaligned) alignment status. */
      }
      requirement <'SWR-GPSH-011'> 'Diagnostics: The rover_gps_heading shall publish a Heading alignment diagnostic (hardware ID RoverGp...' {
        doc /* The rover_gps_heading shall publish a 'Heading alignment' diagnostic (hardware ID 'RoverGpsHeading') with keys State, Accepted segments, Heading output and, once available, Yaw offset (deg) and Offset std (deg), updated by a 1 s timer and immediately at startup and on alignment. */
      }
      requirement <'SWR-GPSH-012'> 'Diagnostics: The rover_gps_heading shall grade the Heading alignment diagnostic OK when aligned, OK ...' {
        doc /* The rover_gps_heading shall grade the 'Heading alignment' diagnostic OK when aligned, OK when unaligned with publish_heading false, and WARN ('Drive straight to align the GNSS heading.') when unaligned with publish_heading true. */
      }
      requirement <'SWR-GPSH-013'> 'Parameter: The rover_gps_heading shall declare publish_heading (default true), min_heading_std_rad...' {
        doc /* The rover_gps_heading shall declare publish_heading (default true), min_heading_std_rad (default 0.05, positive, max 3.2), heading_frame_id (default base_link) and the alignment.* parameters with the defaults min_segment_length_m 3.0, min_speed_m_s 0.3, max_yaw_rate_rad_s 0.1, max_yaw_change_rad 0.1, max_horizontal_std_m 5.0, max_fix_gap_s 2.5, max_odometry_age_s 0.5, required_segments 3 (range 1-100) and max_offset_std_rad 0.1. */
      }
      requirement <'SWR-GPSH-014'> 'Lifecycle: The rover_gps_heading shall validate the alignment configuration at construction, rejec...' {
        doc /* The rover_gps_heading shall validate the alignment configuration at construction, rejecting non-positive limits or required_segments = 0, and shall then exit with code 1 and a FATAL log. */
      }
      requirement <'SWR-GPSH-015'> 'Build: The rover_gps_heading shall keep its domain and application code (geo math, HeadingAlig...' {
        doc /* The rover_gps_heading shall keep its domain and application code (geo math, HeadingAlignmentEstimator, AlignHeadingUseCase) in the rover_gps_heading_core static library without ROS dependencies. */
      }
    }
    package SWRS_HWI {
      requirement <'SWR-HWI-001'> 'Interface: The rover_hardware_interface shall export the pluginlib class rover_hardware_interface/...' {
        doc /* The rover_hardware_interface shall export the pluginlib class rover_hardware_interface/RoverA1System (hardware_interface::SystemInterface) that exposes, for each of exactly four wheel joints matched by the name sequences fl, fr, rl and rr, one velocity command interface and the state interfaces position, velocity and effort in that order, and fail on_init() with ERROR when the joint count, joint names or interface list differ. */
      }
      requirement <'SWR-HWI-002'> 'Functional: The rover_hardware_interface shall convert wheel velocity commands from rad/s at the wh...' {
        doc /* The rover_hardware_interface shall convert wheel velocity commands from rad/s at the wheel to a DCC1000 duty cycle using gear_ratio and max_rpm_motor_speed and clamp the result to [-1.0, 1.0], and report wheel position in rad, velocity in rad/s and effort in N·m derived from encoder_resolution, gear_ratio, motor_torque_constant and gearbox_efficiency. */
      }
      requirement <'SWR-HWI-003'> 'Parameter: The rover_hardware_interface shall configure each DCC1000 with the URDF motor_accelerat...' {
        doc /* The rover_hardware_interface shall configure each DCC1000 with the URDF motor_acceleration (4.0 duty/s) and motor_current_limit (15.0 A) and fail on_init() when motor_acceleration is outside [0.5, 10000] duty/s, motor_current_limit outside [2, 25] A, motor_supply_voltage outside [8, 30] V or the derived current regulator gain outside [1, 100]. */
      }
      requirement <'SWR-HWI-004'> 'Lifecycle: The rover_hardware_interface shall, in on_configure(), start the safety controller befo...' {
        doc /* The rover_hardware_interface shall, in on_configure(), start the safety controller before initialising the motor drivers (up to max_rover_driver_initialization_attempts = 3), arm each DCC1000 hardware failsafe and zero all velocity commands in on_activate() (up to max_rover_driver_activation_attempts = 3), and zero all velocity commands in on_deactivate(). */
      }
      requirement <'SWR-HWI-005'> 'Safety: The rover_hardware_interface shall, in every write() cycle, forward the controller velo...' {
        doc /* The rover_hardware_interface shall, in every write() cycle, forward the controller velocity commands to the drivers only when the component is ACTIVE, neither the software E-Stop nor the PLC latch is active and no hardware fault (motor failsafe trip or welded contactor) is latched, and otherwise discard the pending command and send zero velocity to all four drivers while ACTIVE or INACTIVE. */
      }
      requirement <'SWR-HWI-006'> 'Interface: The rover_hardware_interface shall provide the service hardware_interface/sw_user_e_sto...' {
        doc /* The rover_hardware_interface shall provide the service hardware_interface/sw_user_e_stop_set (std_srvs/Trigger) that writes the sw_e_stop_user_button coil (COIL_2) true on the Modbus link with priority over IO polling, returning success=false with the error text when the write fails. */
      }
      requirement <'SWR-HWI-007'> 'Performance: The rover_hardware_interface shall command zero velocity to all four drive joints withi...' {
        doc /* The rover_hardware_interface shall command zero velocity to all four drive joints within 100 ms of a hardware_interface/sw_user_e_stop_set request being received. */
      }
      requirement <'SWR-HWI-008'> 'Safety: The rover_hardware_interface shall refuse hardware_interface/sw_user_e_stop_reset (succ...' {
        doc /* The rover_hardware_interface shall refuse hardware_interface/sw_user_e_stop_reset (success=false) unless the component is ACTIVE, every wheel velocity command is finite and within velocity_command_zero_tolerance (0.4 rad/s) of zero and every measured wheel velocity is finite and within velocity_state_zero_tolerance (0.05 rad/s) of zero. */
      }
      requirement <'SWR-HWI-009'> 'Safety: The rover_hardware_interface shall provide hardware_interface/sw_e_stop_latch_reset (st...' {
        doc /* The rover_hardware_interface shall provide hardware_interface/sw_e_stop_latch_reset (std_srvs/Trigger) that, only while ACTIVE, pulses the latch-reset coil (COIL_4) true for safety_latch_reset_pulse_ms (100 ms) and then false, clears the latched error-filter state and the welded-contactor fault, and re-arms the DCC1000 failsafe. */
      }
      requirement <'SWR-HWI-010'> 'Safety: The rover_hardware_interface shall refuse hardware_interface/sw_e_stop_latch_reset (suc...' {
        doc /* The rover_hardware_interface shall refuse hardware_interface/sw_e_stop_latch_reset (success=false) while any wheel velocity command exceeds velocity_command_zero_tolerance or any measured wheel velocity exceeds velocity_state_zero_tolerance. */
      }
      requirement <'SWR-HWI-011'> 'Safety: The rover_hardware_interface shall, on every safety-controller start, drive the heartbe...' {
        doc /* The rover_hardware_interface shall, on every safety-controller start, drive the heartbeat, sw_e_stop_user_button and sw_e_stop_motor_driver_fault coils to true and the latch-reset and aux output coils to false, then release only the two software E-Stop inputs, so that the PLC latch stays asserted until an explicit hardware_interface/sw_e_stop_latch_reset. */
      }
      requirement <'SWR-HWI-012'> 'Safety: The rover_hardware_interface shall toggle the CPU watchdog heartbeat coil (COIL_1) ever...' {
        doc /* The rover_hardware_interface shall toggle the CPU watchdog heartbeat coil (COIL_1) every safety_wdg_kick_period_ms (200 ms) on a dedicated thread with absolute steady_clock deadlines and priority access to the Modbus link, counting late kicks and failed kicks without stopping the thread. */
      }
      requirement <'SWR-HWI-013'> 'Interface: The rover_hardware_interface shall connect by Modbus TCP to the safety PLC at modbus_ho...' {
        doc /* The rover_hardware_interface shall connect by Modbus TCP to the safety PLC at modbus_host:modbus_port (192.168.88.11:502) with a per-transaction response timeout of modbus_response_timeout_ms (150 ms), and refresh the cached safety and aux IO every safety_io_poll_period_ms (100 ms) using exactly four read transactions (one FC2, three FC1 blocks of at most 8 coils each within one PLC memory area). */
      }
      requirement <'SWR-HWI-014'> 'Interface: The rover_hardware_interface shall publish rover_msgs/SafetyStatus on hardware_interfac...' {
        doc /* The rover_hardware_interface shall publish rover_msgs/SafetyStatus on hardware_interface/safety_status and rover_msgs/SafetyCommandEcho on hardware_interface/safety_command_echo at driver_states_update_frequency (20 Hz) with QoS reliable, volatile, KeepLast(1), routing HW E-Stop button, contactor feedback and latch status to SafetyStatus and the read-backs of software-driven coils to SafetyCommandEcho. */
      }
      requirement <'SWR-HWI-015'> 'Safety: The rover_hardware_interface shall set SafetyStatus.link_healthy and AuxIoState.link_he...' {
        doc /* The rover_hardware_interface shall set SafetyStatus.link_healthy and AuxIoState.link_healthy false unless both safety-controller threads are running and the last successful IO poll is at most 500 ms old, and stamp io_sample_time with the time of that poll. */
      }
      requirement <'SWR-HWI-016'> 'Performance: The rover_hardware_interface shall publish a change of the hardware E-Stop button, soft...' {
        doc /* The rover_hardware_interface shall publish a change of the hardware E-Stop button, software E-Stop coil or PLC latch state on hardware_interface/safety_status or hardware_interface/safety_command_echo within 100 ms of the change at the PLC. */
      }
      requirement <'SWR-HWI-017'> 'Diagnostics: The rover_hardware_interface shall populate SafetyStatus.latch_cause with the trip sour...' {
        doc /* The rover_hardware_interface shall populate SafetyStatus.latch_cause with the trip source that set the PLC latch (SW user button, SW motor-driver fault, CPU watchdog or HW user button). */
      }
      requirement <'SWR-HWI-018'> 'Safety: The rover_hardware_interface shall assert the sw_e_stop_motor_driver_fault coil (COIL_3...' {
        doc /* The rover_hardware_interface shall assert the sw_e_stop_motor_driver_fault coil (COIL_3) when a motor-driver fault (latched MOTOR_FAILSAFE_TRIPPED or FAULT_FLAG error) is detected, so that the PLC latch opens the contactor. */
      }
      requirement <'SWR-HWI-019'> 'Safety: The rover_hardware_interface shall latch a welded-contactor fault and inhibit motion wh...' {
        doc /* The rover_hardware_interface shall latch a welded-contactor fault and inhibit motion when the PLC latch is active and the motor contactor auxiliary contact still reports engaged for at least 500 ms, keeping the fault latched until hardware_interface/sw_e_stop_latch_reset. */
      }
      requirement <'SWR-HWI-020'> 'Safety: The rover_hardware_interface shall arm each DCC1000 hardware failsafe with motor_failsa...' {
        doc /* The rover_hardware_interface shall arm each DCC1000 hardware failsafe with motor_failsafe_timeout_ms (500 ms), keep feeding it with zero commands while motion is inhibited, and latch a MOTOR_FAILSAFE_TRIPPED error on the first trip that inhibits motion until hardware_interface/sw_e_stop_latch_reset. */
      }
      requirement <'SWR-HWI-021'> 'Diagnostics: The rover_hardware_interface shall publish rover_msgs/RoverDriverState on hardware_inte...' {
        doc /* The rover_hardware_interface shall publish rover_msgs/RoverDriverState on hardware_interface/rover_driver_state at driver_states_update_frequency (20 Hz) with QoS reliable, volatile, KeepLast(5), reporting per driver current, temperature, fault flags, data time-outs and comm_error, where comm_error is set when no Phidget telemetry callback arrived within driver_comm_timeout_ms (300 ms). */
      }
      requirement <'SWR-HWI-022'> 'Diagnostics: The rover_hardware_interface shall publish on diagnostics, with hardware id Rover Syste...' {
        doc /* The rover_hardware_interface shall publish on diagnostics, with hardware id "Rover System", the tasks "system errors", "system status" and "safety plc link" at the diagnostic_updater period (default 1.0 s), reporting ERROR for any latched driver or error-filter fault, a stopped safety thread or a latched welded-contactor fault, and WARN for a late heartbeat or a contactor that is open while the latch is clear. */
      }
      requirement <'SWR-HWI-023'> 'Diagnostics: The rover_hardware_interface shall report the safety plc link diagnostic as ERROR when ...' {
        doc /* The rover_hardware_interface shall report the "safety plc link" diagnostic as ERROR when the last successful IO poll is older than 500 ms or when IO polls keep failing, even while both safety threads are running. */
      }
      requirement <'SWR-HWI-024'> 'Lifecycle: The rover_hardware_interface shall, in on_error(), assert the software E-Stop, broadcas...' {
        doc /* The rover_hardware_interface shall, in on_error(), assert the software E-Stop, broadcast an ERROR status on all diagnostic tasks and tear down the safety controller, drivers and ROS helper node. */
      }
      requirement <'SWR-HWI-025'> 'Interface: The rover_hardware_interface shall provide hardware_interface/aux_output_<0..5>/set (st...' {
        doc /* The rover_hardware_interface shall provide hardware_interface/aux_output_<0..5>/set (std_srvs/SetBool) that write PLC coils 8..13 without priority on the Modbus link, publish rover_msgs/AuxIoState on hardware_interface/aux_io_state at 20 Hz with QoS reliable, volatile, KeepLast(1), and drive all aux outputs off on every safety-controller start. */
      }
      requirement <'SWR-HWI-026'> 'Interface: The rover_hardware_interface shall export the pluginlib class rover_hardware_interface/...' {
        doc /* The rover_hardware_interface shall export the pluginlib class rover_hardware_interface/PhidgetImuSensor (hardware_interface::SensorInterface) that samples the Phidgets Spatial at data_interval_ms (8 ms, 125 Hz), fuses it with a Madgwick filter in the configured world_frame (enu) with use_mag false, exports 10 state interfaces (orientation x/y/z/w, angular_velocity x/y/z, linear_acceleration x/y/z) and returns ERROR from read() while the device is detached. */
      }
      requirement <'SWR-HWI-027'> 'Diagnostics: The rover_hardware_interface shall publish an IMU diagnostic status on diagnostics at >...' {
        doc /* The rover_hardware_interface shall publish an IMU diagnostic status on diagnostics at >= 1 Hz that reports ERROR while the Phidgets Spatial is detached or delivers no valid samples. */
      }
      requirement <'SWR-HWI-028'> 'Build: The rover_hardware_interface shall keep domain and application code free of ROS, hardwa...' {
        doc /* The rover_hardware_interface shall keep domain and application code free of ROS, hardware_interface and vendor SDK includes and keep read()/write() free of blocking Modbus or Phidget SDK calls, enforced by the CTest checks domain_purity_check, application_purity_check and rt_path_purity_check. */
      }
    }
    package SWRS_LED {
      requirement <'SWR-LED-001'> 'Functional: The rover_led shall give every LED segment four animation layers, ERROR (priority 0), A...' {
        doc /* The rover_led shall give every LED segment four animation layers, ERROR (priority 0), ALERT (1), INFO (2) and STATE (3), and shall composite them into one frame with ERROR on top, alpha-blending each layer over the layers below it. */
      }
      requirement <'SWR-LED-002'> 'Functional: The rover_led shall play ALERT-layer animations one after another in FIFO order from a ...' {
        doc /* The rover_led shall play ALERT-layer animations one after another in FIFO order from a queue of at most 10 entries per segment, and shall reject (not queue) a request on a segment whose queue is full while still answering the service call with success=true and a message naming the number of segments that dropped it. */
      }
      requirement <'SWR-LED-003'> 'Interface: The rover_led shall provide the service led/set_animation (rover_msgs/SetLedAnimation) ...' {
        doc /* The rover_led shall provide the service led/set_animation (rover_msgs/SetLedAnimation) that starts the catalog animation with the requested id, param and repeating flag on every segment the catalog entry names, at the entry's priority layer, and shall return success=false with a reason for an unknown id, an unknown segment, an unavailable animation type or an animation that fails to initialise, leaving the current animations unchanged in those cases. */
      }
      requirement <'SWR-LED-004'> 'Interface: The rover_led shall provide the service led/stop_animation (rover_msgs/StopLedAnimation...' {
        doc /* The rover_led shall provide the service led/stop_animation (rover_msgs/StopLedAnimation) that removes the animation with the requested id, including its queued ALERT copies, from its layer on every segment, and shall return success=false when that animation is not playing on any segment. */
      }
      requirement <'SWR-LED-005'> 'Performance: The rover_led shall publish one led/channel_<n>_frame (sensor_msgs/Image, encoding rgba...' {
        doc /* The rover_led shall publish one led/channel__frame (sensor_msgs/Image, encoding rgba8, height = panel rows, width = LEDs per row, data in wire order, KEEP_LAST depth 10) per configured panel at controller_frequency, default 50.0 Hz. */
      }
      requirement <'SWR-LED-006'> 'Interface: The rover_led shall publish led/state (rover_msgs/LedState, reliable, transient-local, ...' {
        doc /* The rover_led shall publish led/state (rover_msgs/LedState, reliable, transient-local, depth 1) at state_publish_rate, default 5.0 Hz, reporting for every segment and every layer whether it is active and, if so, the animation id, name, param, repeating flag, progress and number of queued animations. */
      }
      requirement <'SWR-LED-007'> 'Interface: The rover_led shall publish the loaded animation catalog once at start-up on led/animat...' {
        doc /* The rover_led shall publish the loaded animation catalog once at start-up on led/animations (rover_msgs/LedAnimationCatalog, reliable, transient-local, depth 1) with the id, name and priority of every entry, sorted by id. */
      }
      requirement <'SWR-LED-008'> 'Interface: The rover_led shall publish each panel frame additionally on led/channel_<n>_preview (s...' {
        doc /* The rover_led shall publish each panel frame additionally on led/channel__preview (sensor_msgs/Image, best effort, depth 1) at preview_publish_rate, default 5.0 Hz, and shall create no preview topics when preview_publish_rate is 0. */
      }
      requirement <'SWR-LED-009'> 'Parameter: The rover_led shall load the panel layout and animation catalog from the file given by ...' {
        doc /* The rover_led shall load the panel layout and animation catalog from the file given by animations_config_path (mandatory, non-empty) and, for rover_a1, shall configure two panels (channel 1 front, channel 2 rear) of 40 LEDs folded into 2 rows each and a catalog containing every rover_msgs/LedAnimation id from 0 (E_STOP) to 17 (FLOOD_LIGHT). */
      }
      requirement <'SWR-LED-010'> 'Functional: The rover_led shall provide a catalog animation for each of the SYS-SR-010 states Emerg...' {
        doc /* The rover_led shall provide a catalog animation for each of the SYS-SR-010 states Emergency Stop (E_STOP, id 0, STATE layer), Idle (READY, id 1, STATE), Teleoperation (MANUAL_ACTION, id 4, STATE), Autonomous (AUTONOMOUS_READY id 11 and AUTONOMOUS_ACTION id 12, STATE), Fault (ERROR, id 2, ERROR layer) and Low Battery (LOW_BATTERY id 5 and CRITICAL_BATTERY id 6, INFO layer). */
      }
      requirement <'SWR-LED-011'> 'Functional: The rover_led shall encode each RGBA panel frame as an SK9822 packet: a 4-byte start fr...' {
        doc /* The rover_led shall encode each RGBA panel frame as an SK9822 packet: a 4-byte start frame of 0x00, one 4-byte word per LED of 0xE0 | 5-bit brightness (pixel alpha x global brightness level 0-31 / 255) followed by blue, green and red after gamma 2.2 correction scaled to 240/255/245, and a 4-byte end frame of 0xFF, each word rotated left by one byte for the UDP LED bridge. */
      }
      requirement <'SWR-LED-012'> 'Functional: The rover_led shall drop, without sending, any received panel frame that is older than ...' {
        doc /* The rover_led shall drop, without sending, any received panel frame that is older than frame_timeout (default and rover_a1 value 0.1 s), older than the previously received frame, not rgba8-encoded, or whose height x width or data size does not match the channel's LED count, and shall report each dropped frame as a WARN diagnostic broadcast. */
      }
      requirement <'SWR-LED-013'> 'Interface: The rover_led shall subscribe to led/channel_1_frame and led/channel_2_frame (sensor_ms...' {
        doc /* The rover_led shall subscribe to led/channel_1_frame and led/channel_2_frame (sensor_msgs/Image, depth 5) and publish the encoded packets on udp_write/led_channel_1 and udp_write/led_channel_2 (udp_msgs/UdpPacket, depth 5), sized by channel_1_num_led and channel_2_num_led (40 each for rover_a1). */
      }
      requirement <'SWR-LED-014'> 'Interface: The rover_led shall provide the service led/set_brightness (rover_msgs/SetLedBrightness...' {
        doc /* The rover_led shall provide the service led/set_brightness (rover_msgs/SetLedBrightness) accepting a global brightness in [0.0, 1.0], shall reject values outside that range with success=false and unchanged brightness, shall store an accepted value in the global_brightness parameter so it survives a reconfigure, and shall publish the brightness in effect on led/brightness (std_msgs/Float32, reliable, transient-local, depth 1) on activation and after each change. */
      }
      requirement <'SWR-LED-015'> 'Lifecycle: The rover_led driver shall be a managed lifecycle node that, with autostart=true (defau...' {
        doc /* The rover_led driver shall be a managed lifecycle node that, with autostart=true (default), configures and activates itself once when the executor first spins, forwards frames only while Active, sends an all-off frame to both panels on activation and on deactivation, and on context shutdown finalizes itself (clearing the LEDs) within 1 s. */
      }
      requirement <'SWR-LED-016'> 'Functional: The rover_led driver shall, when led_control_handshake is true, request LED control fro...' {
        doc /* The rover_led driver shall, when led_control_handshake is true, request LED control from hardware/led_control_enable (std_srvs/SetBool) on activation, retrying from a 100 ms timer for at most 3 attempts with a 3 s response timeout each, ignore frames until control is granted, release control on deactivation, and hand back a grant that arrives for an earlier activation. */
      }
      requirement <'SWR-LED-017'> 'Diagnostics: The rover_led controller shall publish on diagnostics, hardware id Bumper Led, a Led co...' {
        doc /* The rover_led controller shall publish on diagnostics, hardware id 'Bumper Led', a 'Led controller status' that is ERROR when rendering fails, WARN when a segment fails to update or a catalog animation uses an unavailable type, and OK otherwise (reporting animations loaded, catalog warnings, last rejected request and what each segment shows), plus a 'Led render rate' frequency status expecting controller_frequency with 10 % tolerance over a 10-sample window. */
      }
      requirement <'SWR-LED-018'> 'Diagnostics: The rover_led driver shall publish on diagnostics, hardware id Bumper Led, a Led driver...' {
        doc /* The rover_led driver shall publish on diagnostics, hardware id 'Bumper Led', a 'Led driver status' that is ERROR 'Driver is not active!' when not Active, WARN 'Driver is not yet functional!' while LED control is pending, ERROR when obtaining control failed, and OK 'Driver is fully functional.' when control is granted, including the lifecycle state and LED control status. */
      }
      requirement <'SWR-LED-019'> 'Build: The rover_led launch file rover_led.launch.py shall load rover_led_driver, rover_led_co...' {
        doc /* The rover_led launch file rover_led.launch.py shall load rover_led_driver, rover_led_controller and two rover_udp_driver UdpSenderNode components (rover_udp_led_channel_1_sender_node to 192.168.77.202:3334 and rover_udp_led_channel_2_sender_node to 192.168.77.201:3333, autostart) into one container rover_led_container with intra-process communication, shall omit the driver and senders when use_sim is True, and shall shut down the launch when the container exits. */
      }
      requirement <'SWR-LED-020'> 'Functional: The rover_led shall provide a catalog animation (and a matching rover_msgs/LedAnimation...' {
        doc /* The rover_led shall provide a catalog animation (and a matching rover_msgs/LedAnimation constant) for the SYS-SR-010 Boot state, shown on both panels from the moment the LED driver becomes Active until a client selects another state animation. */
      }
      requirement <'SWR-LED-021'> 'Functional: The rover_led shall document, in the package, the state-to-LED mapping required by SYS-...' {
        doc /* The rover_led shall document, in the package, the state-to-LED mapping required by SYS-SR-009: for each SYS-SR-010 state the animation id, layer, colour and pattern (steady/blinking/moving, period) shown on the front and rear panels. */
      }
    }
    package SWRS_LOC {
      requirement <'SWR-LOC-001'> 'Functional: The rover_localization shall run a robot_localization EKF node named rover_ekf_node whe...' {
        doc /* The rover_localization shall run a robot_localization EKF node named rover_ekf_node whenever the launch argument use_ekf is true. */
      }
      requirement <'SWR-LOC-002'> 'Performance: The rover_localization shall configure rover_ekf_node to run at 50.0 Hz in 2D mode (two...' {
        doc /* The rover_localization shall configure rover_ekf_node to run at 50.0 Hz in 2D mode (two_d_mode true) with a sensor timeout of 0.1 s. */
      }
      requirement <'SWR-LOC-003'> 'Interface: The rover_localization shall fuse only the body linear velocities vx and vy from nav_ms...' {
        doc /* The rover_localization shall fuse only the body linear velocities vx and vy from nav_msgs/Odometry on odometry/wheels into rover_ekf_node, not the wheel-odometry pose. */
      }
      requirement <'SWR-LOC-004'> 'Interface: The rover_localization shall fuse only the yaw rate (vyaw) from sensor_msgs/Imu on imu/...' {
        doc /* The rover_localization shall fuse only the yaw rate (vyaw) from sensor_msgs/Imu on imu/data into rover_ekf_node, not the IMU absolute orientation or linear acceleration. */
      }
      requirement <'SWR-LOC-005'> 'Interface: The rover_localization shall publish the rover_ekf_node estimate as nav_msgs/Odometry o...' {
        doc /* The rover_localization shall publish the rover_ekf_node estimate as nav_msgs/Odometry on the relative topic odom (remapped from odometry/filtered). */
      }
      requirement <'SWR-LOC-006'> 'Interface: The rover_localization shall make rover_ekf_node the only broadcaster of the odom -> ba...' {
        doc /* The rover_localization shall make rover_ekf_node the only broadcaster of the odom -> base_footprint transform, using world_frame odom, odom_frame odom, map_frame map and base_link_frame base_footprint, with the namespace as tf_prefix. */
      }
      requirement <'SWR-LOC-007'> 'Interface: The rover_localization shall expose the robot_localization services of rover_ekf_node a...' {
        doc /* The rover_localization shall expose the robot_localization services of rover_ekf_node as localization/set_pose, localization/enable and localization/toggle. */
      }
      requirement <'SWR-LOC-008'> 'Diagnostics: The rover_localization shall enable print_diagnostics on every EKF and remap all locali...' {
        doc /* The rover_localization shall enable print_diagnostics on every EKF and remap all localization nodes' /diagnostics to the namespaced diagnostics topic. */
      }
      requirement <'SWR-LOC-009'> 'Functional: The rover_localization shall, when fuse_gps (default $ROVER_USE_GPS) is one of true/1/y...' {
        doc /* The rover_localization shall, when fuse_gps (default $ROVER_USE_GPS) is one of true/1/yes/on in any case and use_ekf is true, additionally start rover_ekf_global_node, rover_navsat_transform_node and rover_gps_heading_node and load the _with_gps configuration. */
      }
      requirement <'SWR-LOC-010'> 'Functional: The rover_localization shall configure rover_ekf_global_node at 50.0 Hz in 2D mode with...' {
        doc /* The rover_localization shall configure rover_ekf_global_node at 50.0 Hz in 2D mode with world_frame map, fusing the same wheel vx/vy and IMU yaw rate as rover_ekf_node plus X/Y only from odometry/gps, and publish its estimate on odometry/global. */
      }
      requirement <'SWR-LOC-011'> 'Parameter: The rover_localization shall broadcast map -> odom from rover_ekf_global_node only when...' {
        doc /* The rover_localization shall broadcast map -> odom from rover_ekf_global_node only when publish_global_tf (default $ROVER_GPS_PUBLISH_MAP_TF) is one of true/1/yes/on, overriding the config's publish_tf. */
      }
      requirement <'SWR-LOC-012'> 'Interface: The rover_localization shall run rover_navsat_transform_node at 10.0 Hz with a 3.0 s st...' {
        doc /* The rover_localization shall run rover_navsat_transform_node at 10.0 Hz with a 3.0 s start delay, magnetic_declination_radians 0.0, yaw_offset 0.0 and zero_altitude true, subscribing gps/fix, gps/heading_imu and odometry/global and publishing odometry/gps and gps/filtered, with the datum service at localization/datum. */
      }
      requirement <'SWR-LOC-013'> 'Parameter: The rover_localization shall pass to rover_gps_heading_node publish_heading true, headi...' {
        doc /* The rover_localization shall pass to rover_gps_heading_node publish_heading true, heading_frame_id /base_link, min_heading_std_rad 0.05 and the alignment parameters min_segment_length_m 3.0, min_speed_m_s 0.3, max_yaw_rate_rad_s 0.1, max_yaw_change_rad 0.1, max_horizontal_std_m 5.0, max_fix_gap_s 2.5, max_odometry_age_s 0.5, required_segments 3 and max_offset_std_rad 0.1. */
      }
      requirement <'SWR-LOC-014'> 'Parameter: The rover_localization launch file shall accept the arguments use_ekf (default False), ...' {
        doc /* The rover_localization launch file shall accept the arguments use_ekf (default False), fuse_gps, publish_global_tf, namespace (default $ROVER_NAMESPACE), localization_mode (rel|enu, default rel), localization_config_path, common_dir_path (default empty), use_sim (default False, sets use_sim_time) and log_level (default INFO). */
      }
      requirement <'SWR-LOC-015'> 'Parameter: The rover_localization shall ship a valid configuration file for every value accepted b...' {
        doc /* The rover_localization shall ship a valid configuration file for every value accepted by the localization_mode argument, including enu. */
      }
      requirement <'SWR-LOC-016'> 'Performance: The rover_localization shall have an automated test that verifies the EKF publishes odo...' {
        doc /* The rover_localization shall have an automated test that verifies the EKF publishes odom at >= 47.5 Hz and the odom -> base_footprint transform from recorded or simulated wheel-odometry and IMU input. */
      }
    }
    package SWRS_MSG {
      requirement <'SWR-MSG-001'> 'Interface: The rover_msgs shall define rover_msgs/SafetyStatus with fields std_msgs/Header header,...' {
        doc /* The rover_msgs shall define rover_msgs/SafetyStatus with fields std_msgs/Header header, builtin_interfaces/Time io_sample_time, bool hw_e_stop_user_button, bool motor_contactor_engaged, bool latch_active, uint8 latch_cause and bool link_healthy. */
      }
      requirement <'SWR-MSG-002'> 'Interface: The rover_msgs shall document in SafetyStatus that hw_e_stop_user_button true means pre...' {
        doc /* The rover_msgs shall document in SafetyStatus that hw_e_stop_user_button true means pressed, motor_contactor_engaged true means contacts closed (drive live), latch_active true means the E-Stop latch holds a stop, and link_healthy false means the other fields are last-known-good values that must not be trusted. */
      }
      requirement <'SWR-MSG-003'> 'Interface: The rover_msgs shall define SafetyStatus.io_sample_time as the time the safety PLC was ...' {
        doc /* The rover_msgs shall define SafetyStatus.io_sample_time as the time the safety PLC was polled, distinct from header.stamp (the publish time), for use by consumer staleness checks. */
      }
      requirement <'SWR-MSG-004'> 'Interface: The rover_msgs shall define the SafetyStatus latch_cause constants LATCH_CAUSE_UNKNOWN ...' {
        doc /* The rover_msgs shall define the SafetyStatus latch_cause constants LATCH_CAUSE_UNKNOWN = 0, LATCH_CAUSE_SW_USER_BUTTON = 1, LATCH_CAUSE_SW_MOTOR_DRIVER_FAULT = 2, LATCH_CAUSE_CPU_WATCHDOG = 3 and LATCH_CAUSE_HW_USER_BUTTON = 4. */
      }
      requirement <'SWR-MSG-005'> 'Safety: The rover_msgs shall define rover_msgs/SafetyCommandEcho (header, io_sample_time, sw_e_...' {
        doc /* The rover_msgs shall define rover_msgs/SafetyCommandEcho (header, io_sample_time, sw_e_stop_user_button, sw_e_stop_motor_driver_fault, sw_e_stop_latch_reset, cpu_wdg_heartbeat) and document that its fields may be used only to inhibit motion, never to permit it, and that cpu_wdg_heartbeat and sw_e_stop_latch_reset carry no stop state. */
      }
      requirement <'SWR-MSG-006'> 'Interface: The rover_msgs shall not define any message type that carries a safety command (E-Stop ...' {
        doc /* The rover_msgs shall not define any message type that carries a safety command (E-Stop set, E-Stop reset or latch reset). */
      }
      requirement <'SWR-MSG-007'> 'Interface: The rover_msgs shall define rover_msgs/AuxIoState with header, io_sample_time, bool[6] ...' {
        doc /* The rover_msgs shall define rover_msgs/AuxIoState with header, io_sample_time, bool[6] inputs (PLC DIO06..DIO11), bool[6] outputs (read-back of DIO00..DIO05) and link_healthy, documented as outside the safety chain. */
      }
      requirement <'SWR-MSG-008'> 'Interface: The rover_msgs shall define rover_msgs/RoverDriverState (header, DriverStateNamed[] dri...' {
        doc /* The rover_msgs shall define rover_msgs/RoverDriverState (header, DriverStateNamed[] driver_states, bool error) whose DriverState carries current, temperature, FaultFlag (emergency_stop, motor_setup_fault), RuntimeError (safety_stop_active), motor_states_data_timed_out, driver_state_data_timed_out, comm_error and heartbeat_timeout. */
      }
      requirement <'SWR-MSG-009'> 'Interface: The rover_msgs shall define rover_msgs/RcLinkStatus carrying the CRSF LINK_STATISTICS u...' {
        doc /* The rover_msgs shall define rover_msgs/RcLinkStatus carrying the CRSF LINK_STATISTICS uplink RSSI (both antennas), uplink_link_quality (0-100 %), uplink SNR, active antenna, rf_mode, uplink TX power and the downlink RSSI, link quality and SNR. */
      }
      requirement <'SWR-MSG-010'> 'Interface: The rover_msgs shall define rover_msgs/LedAnimation animation ids covering every SYS-SR...' {
        doc /* The rover_msgs shall define rover_msgs/LedAnimation animation ids covering every SYS-SR-010 system state, including a Boot state. */
      }
      requirement <'SWR-MSG-011'> 'Interface: The rover_msgs shall define a system-state message whose state constants enumerate at l...' {
        doc /* The rover_msgs shall define a system-state message whose state constants enumerate at least Boot, Idle, Teleoperation, Autonomous, Emergency Stop, Fault and Low Battery. */
      }
      requirement <'SWR-MSG-012'> 'Build: The rover_msgs shall generate all messages and services with rosidl_generate_interfaces...' {
        doc /* The rover_msgs shall generate all messages and services with rosidl_generate_interfaces depending only on builtin_interfaces, geometry_msgs and std_msgs, and export rosidl_default_runtime. */
      }
    }
    package SWRS_MUX {
      requirement <'SWR-MUX-001'> 'Interface: The rover_twist_mux shall publish the arbitrated velocity command as geometry_msgs/Twis...' {
        doc /* The rover_twist_mux shall publish the arbitrated velocity command as geometry_msgs/TwistStamped on cmd_vel (twist_mux cmd_vel_out remapped) and shall be the only publisher of cmd_vel in the platform launch. */
      }
      requirement <'SWR-MUX-002'> 'Functional: The rover_twist_mux shall accept velocity commands from four inputs with strictly order...' {
        doc /* The rover_twist_mux shall accept velocity commands from four inputs with strictly ordered priorities - cmd_elrs (teleop_elrs_cmd_vel_stamped, 110) > joystick (teleop_foxglove_cmd_vel_stamped, 100) > driver_interface (teleop_driver_interface_cmd_vel_fresh_stamped, 8) > nav (nav_cmd_vel_stamped, 5) - and shall forward only the highest-priority input that has not timed out. */
      }
      requirement <'SWR-MUX-003'> 'Safety: The rover_twist_mux shall stop forwarding an input when no message has been received on...' {
        doc /* The rover_twist_mux shall stop forwarding an input when no message has been received on it within its timeout: 0.5 s for cmd_elrs, joystick and nav, and 0.3 s for driver_interface. */
      }
      requirement <'SWR-MUX-004'> 'Safety: The rover_twist_mux shall mask every velocity input while motion_lock is true or has no...' {
        doc /* The rover_twist_mux shall mask every velocity input while motion_lock is true or has not been received within 0.5 s, by configuring the motion_lock lock at priority 200, above every input priority. */
      }
      requirement <'SWR-MUX-005'> 'Interface: The rover_twist_mux shall publish the motion lock state as std_msgs/Bool on motion_lock...' {
        doc /* The rover_twist_mux shall publish the motion lock state as std_msgs/Bool on motion_lock with QoS KeepLast(1), reliable, at publish_frequency (10.0 Hz). */
      }
      requirement <'SWR-MUX-006'> 'Interface: The rover_twist_mux shall subscribe hardware_interface/safety_status (rover_msgs/Safety...' {
        doc /* The rover_twist_mux shall subscribe hardware_interface/safety_status (rover_msgs/SafetyStatus) and hardware_interface/safety_command_echo (rover_msgs/SafetyCommandEcho) with QoS KeepLast(1), reliable, volatile, and shall use SafetyCommandEcho fields only as reasons to inhibit motion. */
      }
      requirement <'SWR-MUX-007'> 'Safety: The rover_twist_mux shall assert motion_lock (true) from start-up until both a SafetySt...' {
        doc /* The rover_twist_mux shall assert motion_lock (true) from start-up until both a SafetyStatus and a SafetyCommandEcho message have been received. */
      }
      requirement <'SWR-MUX-008'> 'Safety: The rover_twist_mux shall assert motion_lock when the older of the two safety messages ...' {
        doc /* The rover_twist_mux shall assert motion_lock when the older of the two safety messages is older than gpio_timeout (1.0 s). */
      }
      requirement <'SWR-MUX-009'> 'Safety: The rover_twist_mux shall assert motion_lock while SafetyStatus.link_healthy is false, ...' {
        doc /* The rover_twist_mux shall assert motion_lock while SafetyStatus.link_healthy is false, even when no stop condition is active. */
      }
      requirement <'SWR-MUX-010'> 'Safety: The rover_twist_mux shall assert motion_lock while any enabled stop condition is active...' {
        doc /* The rover_twist_mux shall assert motion_lock while any enabled stop condition is active - hardware E-Stop button pressed, software E-Stop user button triggered, motor-driver fault stop, or safety latch held - with all four conditions enabled in the shipped configuration. */
      }
      requirement <'SWR-MUX-011'> 'Parameter: The rover_twist_mux shall treat an open motor contactor as a lock condition only when r...' {
        doc /* The rover_twist_mux shall treat an open motor contactor as a lock condition only when require_motor_contactor_engaged is true, which shall be false in the shipped configuration. */
      }
      requirement <'SWR-MUX-012'> 'Parameter: The rover_twist_mux shall reject a motion-lock publish_frequency outside [1.0, 100.0] H...' {
        doc /* The rover_twist_mux shall reject a motion-lock publish_frequency outside [1.0, 100.0] Hz and a gpio_timeout outside [0.1, 10.0] s. */
      }
      requirement <'SWR-MUX-013'> 'Diagnostics: The rover_twist_mux shall report a Motion lock diagnostic (hardware ID Motion Lock) tha...' {
        doc /* The rover_twist_mux shall report a 'Motion lock' diagnostic (hardware ID 'Motion Lock') that is STALE before the first evaluation, ERROR when the safety input is missing, stale or the PLC link is unhealthy, WARN when a stop condition holds the lock, and OK when motion is permitted, including the per-flag values, topic ages and policy settings. */
      }
      requirement <'SWR-MUX-014'> 'Functional: The rover_twist_mux shall relay a Driver UI command from teleop_driver_interface_cmd_ve...' {
        doc /* The rover_twist_mux shall relay a Driver UI command from teleop_driver_interface_cmd_vel_stamped to teleop_driver_interface_cmd_vel_fresh_stamped unchanged only if its receive-time-minus-stamp latency exceeds the latency baseline by at most max_delay (0.3 s), and shall drop commands with a zero or negative header stamp. */
      }
      requirement <'SWR-MUX-015'> 'Functional: The rover_twist_mux shall, in rover_command_freshness_node, let the filters latency bas...' {
        doc /* The rover_twist_mux shall, in rover_command_freshness_node, let the filter's latency baseline rise by at most max_clock_drift (0.001 s per s of traffic, crediting at most 1.0 s per command gap), and shall re-baseline only on a run of rejected commands that starts after at least resync_gap (1.0 s) of silence and stays within max_delay of itself for resync_time (2.0 s). */
      }
      requirement <'SWR-MUX-016'> 'Interface: The rover_twist_mux shall, in rover_command_freshness_node, subscribe and publish geome...' {
        doc /* The rover_twist_mux shall, in rover_command_freshness_node, subscribe and publish geometry_msgs/TwistStamped with QoS KeepLast(10), reliable. */
      }
      requirement <'SWR-MUX-017'> 'Parameter: The rover_twist_mux shall, in rover_command_freshness_node, treat the nodes parameters ...' {
        doc /* The rover_twist_mux shall, in rover_command_freshness_node, treat the node's parameters as read-only and reject max_delay outside [0.05, 5.0] s, max_clock_drift outside [0.0, 0.1], resync_time outside [0.5, 60.0] s and resync_gap outside [0.1, 60.0] s. */
      }
      requirement <'SWR-MUX-018'> 'Diagnostics: The rover_twist_mux shall, in rover_command_freshness_node, report a Command freshness ...' {
        doc /* The rover_twist_mux shall, in rover_command_freshness_node, report a 'Command freshness' diagnostic (hardware ID 'Command Freshness') that is WARN for 5.0 s after any dropped command and OK otherwise, with accepted/dropped/resync counts, latency baseline, last excess delay and max_delay. */
      }
      requirement <'SWR-MUX-019'> 'Safety: The rover_twist_mux shall publish motion_lock immediately when a received SafetyStatus ...' {
        doc /* The rover_twist_mux shall publish motion_lock immediately when a received SafetyStatus or SafetyCommandEcho changes the evaluated lock state, in addition to the periodic publication, so that the lock path contributes to the 100 ms software E-Stop budget. */
      }
      requirement <'SWR-MUX-020'> 'Interface: The rover_twist_mux shall publish which input is currently selected (or none, or locked...' {
        doc /* The rover_twist_mux shall publish which input is currently selected (or none, or locked) on a ROS topic whenever the selection changes, for use by the platform mode manager. */
      }
    }
    part def RoverPlatformArch {
      part SensorPayload {
        @$::RoverPlatform::RoverPlatformProfile::ExternalActor {
          description = "rover_sensors container: GNSS (RUTX11) and RS16 lidar.";
        }
        out port gps_fix: $::RoverPlatform::RoverPlatformInterfaces::NavSatFix;
      }
      part SafetyPlc {
        @$::RoverPlatform::RoverPlatformProfile::ExternalActor {
          description = "Safety controller PLC: SR latch, contactor, HW E-stop button (Modbus TCP).";
        }
        out port modbus_inputs: $::RoverPlatform::RoverPlatformInterfaces::ModbusFrame;
        in port modbus_coils: ~$::RoverPlatform::RoverPlatformInterfaces::ModbusFrame;
      }
      part Safety {
        @$::RoverPlatform::RoverPlatformProfile::FunctionalSubsystem {
          label = "Safety Supervision";
        }
        part rover_safety {
          @$::RoverPlatform::RoverPlatformProfile::RosPackage {
            behaviourModel = "PlatformModeManager";
            executables = "rover_safety_node; rover_led_safety_node";
            'language' = "C++ (BT.CPP v4)";
            lifecycle = "lifecycle";
            'package' = "rover_safety";
            swrsPrefix = "SAF";
          }
          attribute battery_temp_critical : $::ScalarValues::Real default := 50;
          attribute battery_temp_fatal : $::ScalarValues::Real default := 60;
          attribute led_battery_critical : $::ScalarValues::Real default := 0.1;
          attribute led_battery_low : $::ScalarValues::Real default := 0.4;
          attribute safety_tick_rate : $::ScalarValues::Real default := 10;
          out port trip_e_stop: $::RoverPlatform::RoverPlatformInterfaces::TriggerService {
            out attribute success redefines success;
          }
          in port system_status: ~$::RoverPlatform::RoverPlatformInterfaces::SystemStatus;
          in port safety_status: ~$::RoverPlatform::RoverPlatformInterfaces::SafetyStatus;
          in port safety_command_echo: ~$::RoverPlatform::RoverPlatformInterfaces::SafetyCommandEcho;
          in port rover_driver_state: ~$::RoverPlatform::RoverPlatformInterfaces::DriverState;
          out port led_set_animation: $::RoverPlatform::RoverPlatformInterfaces::LedAnimationRequest {
            out attribute animation_id redefines animation_id;
            out attribute repeating redefines repeating;
          }
          in port battery_status: ~$::RoverPlatform::RoverPlatformInterfaces::BatteryState;
          satisfy $::RoverPlatform::SWRS_SAF::'Interface: The rover_safety shall, in rover_safety_node, subscribe rover_battery/battery_status (s...';
          satisfy $::RoverPlatform::SWRS_SAF::'Safety: The rover_safety shall, in rover_safety_node, classify a battery reading with health WA...';
          satisfy $::RoverPlatform::SWRS_SAF::'Safety: The rover_safety shall, in rover_safety_node, for a battery reading with health OVERHEA...';
          satisfy $::RoverPlatform::SWRS_SAF::'Parameter: The rover_safety shall, in rover_safety_node, reject battery.temp.critical and battery....';
          satisfy $::RoverPlatform::SWRS_SAF::'Functional: The rover_safety shall, in rover_safety_node, tick the RoverSafety behavior tree at tim...';
          satisfy $::RoverPlatform::SWRS_SAF::'Safety: The rover_safety shall, in rover_safety_node, evaluate and act on the battery verdict a...';
          satisfy $::RoverPlatform::SWRS_SAF::'Functional: The rover_safety shall, in rover_safety_node, once a shutdown is started (SignalShutdow...';
          satisfy $::RoverPlatform::SWRS_SAF::'Interface: The rover_safety shall, in rover_safety_node, provide ~/shutdown (std_srvs/Trigger) whe...';
          satisfy $::RoverPlatform::SWRS_SAF::'Diagnostics: The rover_safety shall, in rover_safety_node, publish diagnostics under hardware ID Rov...';
          satisfy $::RoverPlatform::SWRS_SAF::'Lifecycle: The rover_safety shall run rover_safety_node and rover_led_safety_node as nav2::Lifecyc...';
          satisfy $::RoverPlatform::SWRS_SAF::'Build: The rover_safety shall, in rover_safety.launch.py, start rover_led_safety_node always a...';
          satisfy $::RoverPlatform::SWRS_SAF::'Safety: The rover_safety shall, in rover_safety_node, evaluate motor-driver temperature from ha...';
          satisfy $::RoverPlatform::SWRS_SAF::'Interface: The rover_safety shall, in rover_led_safety_node, subscribe rover_battery/battery_statu...';
          satisfy $::RoverPlatform::SWRS_SAF::'Functional: The rover_safety shall, in rover_led_safety_node, select the state animation E_STOP whi...';
          satisfy $::RoverPlatform::SWRS_SAF::'Functional: The rover_safety shall, in rover_led_safety_node, while discharging, request CRITICAL_B...';
          satisfy $::RoverPlatform::SWRS_SAF::'Functional: The rover_safety shall, in rover_led_safety_node, request the ERROR animation while the...';
          satisfy $::RoverPlatform::SWRS_SAF::'Diagnostics: The rover_safety shall, in rover_led_safety_node, publish diagnostics under hardware ID...';
          satisfy $::RoverPlatform::SWRS_SAF::'Functional: The rover_safety shall provide a platform mode manager that maintains exactly one curre...';
          satisfy $::RoverPlatform::SWRS_SAF::'Functional: The rover_safety shall, in its platform mode manager, implement these transitions: Boot...';
          satisfy $::RoverPlatform::SWRS_SAF::'Safety: The rover_safety shall, in its platform mode manager, enter Low Battery when the batter...';
          satisfy $::RoverPlatform::SWRS_SAF::'Safety: The rover_safety shall, in its platform mode manager, enter Fault when the battery verd...';
          satisfy $::RoverPlatform::SWRS_SAF::'Functional: The rover_safety shall, in rover_led_safety_node, display each SYS-SR-010 state with a ...';
        }
        out port trip_e_stop: $::RoverPlatform::RoverPlatformInterfaces::TriggerService {
          out attribute success redefines success;
        }
        in port system_status: ~$::RoverPlatform::RoverPlatformInterfaces::SystemStatus {
          in attribute ram_percent redefines ram_percent;
          in attribute cpu_percent redefines cpu_percent;
          in attribute cpu_temp redefines cpu_temp;
          in attribute disk_percent redefines disk_percent;
        }
        in port safety_status: ~$::RoverPlatform::RoverPlatformInterfaces::SafetyStatus {
          in attribute motor_contactor_engaged redefines motor_contactor_engaged;
          in attribute hw_e_stop_user_button redefines hw_e_stop_user_button;
          in attribute latch_active redefines latch_active;
          in attribute latch_cause redefines latch_cause;
          in attribute link_healthy redefines link_healthy;
        }
        in port safety_command_echo: ~$::RoverPlatform::RoverPlatformInterfaces::SafetyCommandEcho {
          in attribute cpu_wdg_heartbeat redefines cpu_wdg_heartbeat;
          in attribute sw_e_stop_user_button redefines sw_e_stop_user_button;
          in attribute sw_e_stop_latch_reset redefines sw_e_stop_latch_reset;
          in attribute sw_e_stop_motor_driver_fault redefines sw_e_stop_motor_driver_fault;
        }
        in port rover_driver_state: ~$::RoverPlatform::RoverPlatformInterfaces::DriverState {
          in attribute current redefines current;
          in attribute fault redefines fault;
          in attribute temperature redefines temperature;
        }
        out port led_set_animation: $::RoverPlatform::RoverPlatformInterfaces::LedAnimationRequest {
          out attribute animation_id redefines animation_id;
          out attribute repeating redefines repeating;
        }
        in port battery_status: ~$::RoverPlatform::RoverPlatformInterfaces::BatteryState {
          in attribute current redefines current;
          in attribute power_supply_health redefines power_supply_health;
          in attribute percentage redefines percentage;
          in attribute temperature redefines temperature;
          in attribute voltage redefines voltage;
        }
        connect battery_status to rover_safety.battery_status {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "sensor_msgs/BatteryState";
            qos = "depth 5";
            rate = "per BMS packet (no fixed rate)";
            source = "rover_battery/src/infrastructure/ros2_battery_state_publisher.cpp:83-85";
            topic = "rover_battery/battery_status";
          }
        }
        connect rover_safety.trip_e_stop to trip_e_stop {
          @$::RoverPlatform::RoverPlatformProfile::RosService {
            rate = "on battery verdict";
            service = "hardware_interface/sw_user_e_stop_set";
            source = "rover_safety/behavior_trees/rover_safety.xml:4-16";
            srvType = "std_srvs/Trigger";
          }
        }
        connect rover_driver_state to rover_safety.rover_driver_state {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "rover_msgs/RoverDriverState";
            qos = "reliable, KeepLast(5)";
            rate = "20 Hz";
            source = "rover_hardware_interface/src/system_ros_interface/system_ros_interface.cpp:148-150";
            topic = "hardware_interface/rover_driver_state";
          }
        }
        connect rover_safety.led_set_animation to led_set_animation {
          @$::RoverPlatform::RoverPlatformProfile::RosService {
            rate = "on state change";
            service = "led/set_animation";
            source = "rover_safety/behavior_trees/rover_led_safety.xml:97-107";
            srvType = "rover_msgs/SetLedAnimation";
          }
        }
        connect safety_command_echo to rover_safety.safety_command_echo {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "rover_msgs/SafetyCommandEcho";
            qos = "reliable, volatile, KeepLast(1)";
            rate = "20 Hz";
            source = "rover_hardware_interface/src/system_ros_interface/system_ros_interface.cpp:169-184";
            topic = "hardware_interface/safety_command_echo";
          }
        }
        connect safety_status to rover_safety.safety_status {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "rover_msgs/SafetyStatus";
            qos = "reliable, volatile, KeepLast(1)";
            rate = "20 Hz";
            source = "rover_hardware_interface/src/system_ros_interface/system_ros_interface.cpp:169-184";
            topic = "hardware_interface/safety_status";
          }
        }
        connect system_status to rover_safety.system_status {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "rover_msgs/SystemStatus";
            qos = "depth 10";
            rate = "5 Hz";
            source = "rover_diag_manager/config/system_diag.yaml:7";
            topic = "system_status";
          }
        }
      }
      part RcTransmitter {
        @$::RoverPlatform::RoverPlatformProfile::ExternalActor {
          description = "ExpressLRS RC transmitter + receiver. The receiver UART is on the RUTX11 router (USB-UART adapter), which forwards the CRSF bytes over UDP.";
        }
        out port crsf_udp: $::RoverPlatform::RoverPlatformInterfaces::ByteStream;
      }
      part Power {
        @$::RoverPlatform::RoverPlatformProfile::FunctionalSubsystem {
          label = "Power";
        }
        part rover_battery {
          @$::RoverPlatform::RoverPlatformProfile::RosPackage {
            behaviourModel = "";
            executables = "rover_battery_node; rover_udp_battery_receiver_node";
            'language' = "C++";
            lifecycle = "plain";
            'package' = "rover_battery";
            swrsPrefix = "BAT";
          }
          attribute design_capacity : $::ScalarValues::Real default := 40;
          attribute watchdog_timeout : $::ScalarValues::Real default := 10;
          in port rover_battery_udp_data: ~$::RoverPlatform::RoverPlatformInterfaces::ByteStream;
          out port diagnostics: $::RoverPlatform::RoverPlatformInterfaces::Diagnostics {
            out attribute level redefines level;
          }
          out port battery_status: $::RoverPlatform::RoverPlatformInterfaces::BatteryState {
            out attribute current redefines current;
            out attribute power_supply_health redefines power_supply_health;
            out attribute percentage redefines percentage;
            out attribute temperature redefines temperature;
            out attribute voltage redefines voltage;
          }
          satisfy $::RoverPlatform::SWRS_BAT::'Interface: The rover_battery shall subscribe udp_msgs/UdpPacket on rover_battery_udp_data (depth 1...';
          satisfy $::RoverPlatform::SWRS_BAT::'Functional: The rover_battery shall decode only payloads of exactly 392 bytes (385-byte BmsData + 7...';
          satisfy $::RoverPlatform::SWRS_BAT::'Interface: The rover_battery shall publish sensor_msgs/BatteryState on rover_battery/battery_statu...';
          satisfy $::RoverPlatform::SWRS_BAT::'Functional: The rover_battery shall fill BatteryState with voltage = pack voltage [V], current = pa...';
          satisfy $::RoverPlatform::SWRS_BAT::'Functional: The rover_battery shall classify battery health from the BMS alarm bits as DEAD on a le...';
          satisfy $::RoverPlatform::SWRS_BAT::'Functional: The rover_battery shall report power_supply_status NOT_CHARGING for BMS status 0, CHARG...';
          satisfy $::RoverPlatform::SWRS_BAT::'Interface: The rover_battery shall publish rover_msgs/ChargingStatus on rover_battery/charging_sta...';
          satisfy $::RoverPlatform::SWRS_BAT::'Safety: The rover_battery shall, when no valid BMS frame has been received for watchdog_timeout...';
          satisfy $::RoverPlatform::SWRS_BAT::'Parameter: The rover_battery shall declare design_capacity (default 40.0 Ah, range [0.1, 1000.0]),...';
          satisfy $::RoverPlatform::SWRS_BAT::'Diagnostics: The rover_battery shall publish diagnostics under hardware ID RoverBattery with Battery...';
          satisfy $::RoverPlatform::SWRS_BAT::'Build: The rover_battery shall, in rover_battery.launch.py, load rover_battery_node and the UD...';
          satisfy $::RoverPlatform::SWRS_BAT::'Performance: The rover_battery shall publish rover_battery/battery_status at 1 Hz or faster regardle...';
          satisfy $::RoverPlatform::SWRS_BAT::'Interface: The rover_battery shall set BatteryState.header.stamp to the time the BMS frame was rec...';
          satisfy $::RoverPlatform::SWRS_BAT::'Performance: The rover_battery shall report pack voltage, current and state of charge within 5 % of ...';
          satisfy $::RoverPlatform::SWRS_BAT::'Lifecycle: The rover_battery shall implement rover_battery_node as a managed lifecycle node that c...';
        }
        in port rover_battery_udp_data: ~$::RoverPlatform::RoverPlatformInterfaces::ByteStream;
        out port diagnostics: $::RoverPlatform::RoverPlatformInterfaces::Diagnostics {
          out attribute level redefines level;
        }
        out port battery_status: $::RoverPlatform::RoverPlatformInterfaces::BatteryState {
          out attribute current redefines current;
          out attribute power_supply_health redefines power_supply_health;
          out attribute percentage redefines percentage;
          out attribute temperature redefines temperature;
          out attribute voltage redefines voltage;
        }
        connect rover_battery.battery_status to battery_status {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "sensor_msgs/BatteryState";
            qos = "depth 5";
            rate = "per BMS packet (no fixed rate)";
            source = "rover_battery/src/infrastructure/ros2_battery_state_publisher.cpp:83-85";
            topic = "rover_battery/battery_status";
          }
        }
        connect rover_battery.diagnostics to diagnostics {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "diagnostic_msgs/DiagnosticArray";
            qos = "reliable";
            rate = "1 Hz";
            source = "rover_battery/src/infrastructure/rover_battery_node.cpp:41,73";
            topic = "diagnostics";
          }
        }
        connect rover_battery_udp_data to rover_battery.rover_battery_udp_data {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "udp_msgs/UdpPacket";
            qos = "depth 100";
            rate = "per packet";
            source = "rover_battery/launch/rover_battery.launch.py:69";
            topic = "rover_battery_udp_data";
          }
        }
      }
      part PlatformServices {
        @$::RoverPlatform::RoverPlatformProfile::FunctionalSubsystem {
          label = "Platform Services";
        }
        part rover_transport {
          @$::RoverPlatform::RoverPlatformProfile::RosPackage {
            behaviourModel = "";
            executables = "UdpReceiverNode; UdpSenderNode; rover_modbus_driver (library)";
            'language' = "C++";
            lifecycle = "lifecycle";
            'package' = "rover_transport";
            swrsPrefix = "TRN";
          }
          in port udp_write_led: ~$::RoverPlatform::RoverPlatformInterfaces::ByteStream;
          out port rover_battery_udp_data: $::RoverPlatform::RoverPlatformInterfaces::ByteStream {
            out attribute length redefines length;
          }
          out port rc_raw_udp: $::RoverPlatform::RoverPlatformInterfaces::ByteStream {
            out attribute length redefines length;
          }
          out port led_udp: $::RoverPlatform::RoverPlatformInterfaces::ByteStream {
            out attribute length redefines length;
          }
          in port crsf_udp: ~$::RoverPlatform::RoverPlatformInterfaces::ByteStream;
          in port bms_udp: ~$::RoverPlatform::RoverPlatformInterfaces::ByteStream;
          satisfy $::RoverPlatform::SWRS_TRN::'Lifecycle: The rover_transport shall implement UdpReceiverNode and UdpSenderNode as ROS 2 managed ...';
          satisfy $::RoverPlatform::SWRS_TRN::'Lifecycle: The rover_transport shall, when a transport nodes autostart parameter is true, trigger ...';
          satisfy $::RoverPlatform::SWRS_TRN::'Interface: The rover_transport shall, in UdpReceiverNode, bind a UDP/IPv4 socket to the configured...';
          satisfy $::RoverPlatform::SWRS_TRN::'Functional: The rover_transport shall keep the UDP receiver armed after a successful zero-length da...';
          satisfy $::RoverPlatform::SWRS_TRN::'Functional: The rover_transport shall recover from a receive error on a UDP transport by re-arming ...';
          satisfy $::RoverPlatform::SWRS_TRN::'Interface: The rover_transport shall, in UdpSenderNode, subscribe to udp_write (udp_msgs/UdpPacket...';
          satisfy $::RoverPlatform::SWRS_TRN::'Parameter: The rover_transport shall declare the UDP node parameters ip (string), port (int) and a...';
          satisfy $::RoverPlatform::SWRS_TRN::'Safety: The rover_transport shall not write any byte to a UDP endpoint while the owning node is...';
          satisfy $::RoverPlatform::SWRS_TRN::'Performance: The rover_transport shall use a 2048-byte receive buffer for each UDP socket, and shall...';
          satisfy $::RoverPlatform::SWRS_TRN::'Lifecycle: The rover_transport shall make closing a UDP socket wait until no receive or send handl...';
          satisfy $::RoverPlatform::SWRS_TRN::'Interface: The rover_transport shall register UdpReceiverNode and UdpSenderNode as rclcpp_componen...';
          satisfy $::RoverPlatform::SWRS_TRN::'Interface: The rover_transport shall provide, in rover_modbus_driver, an in-process synchronous Mo...';
          satisfy $::RoverPlatform::SWRS_TRN::'Safety: The rover_transport shall refuse a coil write whose CoilInfo.is_coil_engage_allowed is ...';
          satisfy $::RoverPlatform::SWRS_TRN::'Functional: The rover_transport shall establish the initial Modbus TCP connection with up to connec...';
          satisfy $::RoverPlatform::SWRS_TRN::'Functional: The rover_transport shall, after any failed Modbus transaction, drop the TCP link, then...';
          satisfy $::RoverPlatform::SWRS_TRN::'Performance: The rover_transport shall abort a Modbus transaction whose reply does not arrive within...';
          satisfy $::RoverPlatform::SWRS_TRN::'Performance: The rover_transport shall bound every Modbus TCP connection attempt, initial or re-dial...';
          satisfy $::RoverPlatform::SWRS_TRN::'Diagnostics: The rover_transport shall publish diagnostic status on /diagnostics at 1 Hz or faster f...';
          satisfy $::RoverPlatform::SWRS_TRN::'Build: The rover_transport shall build as ament_cmake packages on the rovers ROS 2 distributio...';
          satisfy $::RoverPlatform::SWRS_TRN::'Safety: The rover_transport shall, in UdpReceiverNode, accept a source_ip parameter (string, de...';
        }
        part rover_msgs {
          @$::RoverPlatform::RoverPlatformProfile::RosPackage {
            behaviourModel = "";
            executables = "(interface package: 29 msg, 13 srv)";
            'language' = "IDL";
            lifecycle = "none";
            'package' = "rover_msgs";
            swrsPrefix = "MSG";
          }
          satisfy $::RoverPlatform::SWRS_MSG::'Interface: The rover_msgs shall define rover_msgs/SafetyStatus with fields std_msgs/Header header,...';
          satisfy $::RoverPlatform::SWRS_MSG::'Interface: The rover_msgs shall document in SafetyStatus that hw_e_stop_user_button true means pre...';
          satisfy $::RoverPlatform::SWRS_MSG::'Interface: The rover_msgs shall define SafetyStatus.io_sample_time as the time the safety PLC was ...';
          satisfy $::RoverPlatform::SWRS_MSG::'Interface: The rover_msgs shall define the SafetyStatus latch_cause constants LATCH_CAUSE_UNKNOWN ...';
          satisfy $::RoverPlatform::SWRS_MSG::'Safety: The rover_msgs shall define rover_msgs/SafetyCommandEcho (header, io_sample_time, sw_e_...';
          satisfy $::RoverPlatform::SWRS_MSG::'Interface: The rover_msgs shall not define any message type that carries a safety command (E-Stop ...';
          satisfy $::RoverPlatform::SWRS_MSG::'Interface: The rover_msgs shall define rover_msgs/AuxIoState with header, io_sample_time, bool[6] ...';
          satisfy $::RoverPlatform::SWRS_MSG::'Interface: The rover_msgs shall define rover_msgs/RoverDriverState (header, DriverStateNamed[] dri...';
          satisfy $::RoverPlatform::SWRS_MSG::'Interface: The rover_msgs shall define rover_msgs/RcLinkStatus carrying the CRSF LINK_STATISTICS u...';
          satisfy $::RoverPlatform::SWRS_MSG::'Interface: The rover_msgs shall define rover_msgs/LedAnimation animation ids covering every SYS-SR...';
          satisfy $::RoverPlatform::SWRS_MSG::'Interface: The rover_msgs shall define a system-state message whose state constants enumerate at l...';
          satisfy $::RoverPlatform::SWRS_MSG::'Build: The rover_msgs shall generate all messages and services with rosidl_generate_interfaces...';
        }
        part rover_bringup {
          @$::RoverPlatform::RoverPlatformProfile::RosPackage {
            behaviourModel = "";
            executables = "rover_bringup.launch.py; rover_web_bridges.launch.py (foxglove_bridge, rosbridge)";
            'language' = "Python launch";
            lifecycle = "none";
            'package' = "rover_bringup";
            swrsPrefix = "BRG";
          }
          out port ui_e_stop_services: $::RoverPlatform::RoverPlatformInterfaces::TriggerService {
            out attribute success redefines success;
          }
          out port teleop_web_cmd_vel_stamped: $::RoverPlatform::RoverPlatformInterfaces::TwistStamped {
            out attribute angular_z redefines angular_z;
            out attribute linear_x redefines linear_x;
            out attribute stamp redefines stamp;
          }
          out port teleop_foxglove_cmd_vel_stamped: $::RoverPlatform::RoverPlatformInterfaces::TwistStamped {
            out attribute angular_z redefines angular_z;
            out attribute linear_x redefines linear_x;
            out attribute stamp redefines stamp;
          }
          out port foxglove_ws_out: $::RoverPlatform::RoverPlatformInterfaces::ByteStream {
            out attribute length redefines length;
          }
          in port foxglove_ws_in: ~$::RoverPlatform::RoverPlatformInterfaces::ByteStream;
          in port drive_mode: ~$::RoverPlatform::RoverPlatformInterfaces::DriveMode;
          in port diagnostics_agg: ~$::RoverPlatform::RoverPlatformInterfaces::Diagnostics;
          satisfy $::RoverPlatform::SWRS_BRG::'Lifecycle: The rover_bringup shall start the platform stack only when the environment variable ROB...';
          satisfy $::RoverPlatform::SWRS_BRG::'Diagnostics: The rover_bringup shall log a warning when SYSTEM_BUILD_VERSION (default v1.0.0) is low...';
          satisfy $::RoverPlatform::SWRS_BRG::'Functional: The rover_bringup shall start rover_controller (robot description, controller_manager a...';
          satisfy $::RoverPlatform::SWRS_BRG::'Functional: The rover_bringup shall start rover_diag_manager (system diagnostics and the diagnostic...';
          satisfy $::RoverPlatform::SWRS_BRG::'Lifecycle: The rover_bringup shall start rover_battery, rover_led, rover_safety, rover_localizatio...';
          satisfy $::RoverPlatform::SWRS_BRG::'Safety: The rover_bringup shall start rover_safety in the second stage unless the launch argume...';
          satisfy $::RoverPlatform::SWRS_BRG::'Functional: The rover_bringup shall start rover_localization with use_ekf True and fuse_gps set fro...';
          satisfy $::RoverPlatform::SWRS_BRG::'Parameter: The rover_bringup shall pass namespace (default $ROVER_NAMESPACE, else empty), log_leve...';
          satisfy $::RoverPlatform::SWRS_BRG::'Build: The rover_bringup shall declare run dependencies on rover_controller, rover_localizatio...';
          satisfy $::RoverPlatform::SWRS_BRG::'Build: The rover_bringup shall run on the ROS 2 distribution fixed in SYS-SR-020 with rmw_zeno...';
          satisfy $::RoverPlatform::SWRS_BRG::'Lifecycle: The rover_bringup shall be started automatically at power-up and bring the platform to ...';
          satisfy $::RoverPlatform::SWRS_BRG::'Interface: The rover_bringup shall start foxglove_bridge as node rover_foxglove_bridge (WebSocket ...';
          satisfy $::RoverPlatform::SWRS_BRG::'Performance: The rover_bringup shall restrict foxglove_bridge to the whitelisted topics (/tf, /tf_st...';
          satisfy $::RoverPlatform::SWRS_BRG::'Interface: The rover_bringup shall start rosbridge_websocket as node rover_rosbridge_websocket (po...';
          satisfy $::RoverPlatform::SWRS_BRG::'Lifecycle: The rover_bringup shall have an automated launch test that verifies the two-stage start...';
        }
        out port ui_e_stop_services: $::RoverPlatform::RoverPlatformInterfaces::TriggerService {
          out attribute success redefines success;
        }
        in port udp_write_led: ~$::RoverPlatform::RoverPlatformInterfaces::ByteStream {
          in attribute length redefines length;
        }
        out port teleop_web_cmd_vel_stamped: $::RoverPlatform::RoverPlatformInterfaces::TwistStamped {
          out attribute angular_z redefines angular_z;
          out attribute linear_x redefines linear_x;
          out attribute stamp redefines stamp;
        }
        out port teleop_foxglove_cmd_vel_stamped: $::RoverPlatform::RoverPlatformInterfaces::TwistStamped {
          out attribute angular_z redefines angular_z;
          out attribute linear_x redefines linear_x;
          out attribute stamp redefines stamp;
        }
        out port rover_battery_udp_data: $::RoverPlatform::RoverPlatformInterfaces::ByteStream {
          out attribute length redefines length;
        }
        out port rc_raw_udp: $::RoverPlatform::RoverPlatformInterfaces::ByteStream {
          out attribute length redefines length;
        }
        out port led_udp: $::RoverPlatform::RoverPlatformInterfaces::ByteStream {
          out attribute length redefines length;
        }
        out port foxglove_ws_out: $::RoverPlatform::RoverPlatformInterfaces::ByteStream {
          out attribute length redefines length;
        }
        in port foxglove_ws_in: ~$::RoverPlatform::RoverPlatformInterfaces::ByteStream {
          in attribute length redefines length;
        }
        in port drive_mode: ~$::RoverPlatform::RoverPlatformInterfaces::DriveMode {
          in attribute guard redefines guard;
          in attribute mode redefines mode;
          in attribute reason redefines reason;
        }
        in port diagnostics_agg: ~$::RoverPlatform::RoverPlatformInterfaces::Diagnostics {
          in attribute level redefines level;
        }
        in port crsf_udp: ~$::RoverPlatform::RoverPlatformInterfaces::ByteStream {
          in attribute length redefines length;
        }
        in port bms_udp: ~$::RoverPlatform::RoverPlatformInterfaces::ByteStream {
          in attribute length redefines length;
        }
        connect rover_transport.rc_raw_udp to rc_raw_udp {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "udp_msgs/UdpPacket";
            qos = "reliable, depth 100";
            rate = "one per datagram, ~250 Hz";
            source = "rover_crsf_teleop/launch/rover_crsf_teleop.launch.py:28,121";
            topic = "rc/raw_udp";
          }
        }
        connect rover_bringup.ui_e_stop_services to ui_e_stop_services {
          @$::RoverPlatform::RoverPlatformProfile::RosService {
            rate = "on demand";
            service = "hardware_interface/sw_user_e_stop_set|reset, sw_e_stop_latch_reset";
            source = "rover_bringup/launch/rover_web_bridges.launch.py:106-149";
            srvType = "std_srvs/Trigger";
          }
        }
        connect rover_bringup.teleop_web_cmd_vel_stamped to teleop_web_cmd_vel_stamped {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "geometry_msgs/TwistStamped";
            qos = "reliable";
            rate = "10 Hz";
            source = "rover_bringup/launch/rover_web_bridges.launch.py:74-78";
            topic = "teleop_web_cmd_vel_stamped";
          }
        }
        connect rover_bringup.foxglove_ws_out to foxglove_ws_out {
          @$::RoverPlatform::RoverPlatformProfile::HwBus {
            address = "TCP 8765 (foxglove_bridge), 9090 (rosbridge)";
            bus = "Websocket (LAN/Wi-Fi)";
            rate = "per topic";
            source = "rover_bringup/launch/rover_web_bridges.launch.py:54-198";
          }
        }
        connect drive_mode to rover_bringup.drive_mode {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "rover_msgs/DriveMode";
            qos = "reliable, transient_local";
            rate = "on change";
            source = "rover_msgs/msg/DriveMode.msg:1";
            topic = "drive_mode";
          }
        }
        connect foxglove_ws_in to rover_bringup.foxglove_ws_in {
          @$::RoverPlatform::RoverPlatformProfile::HwBus {
            address = "TCP 8765 (foxglove_bridge), 9090 (rosbridge)";
            bus = "Websocket (LAN/Wi-Fi)";
            rate = "10 Hz UI stream";
            source = "rover_bringup/launch/rover_web_bridges.launch.py:54-198";
          }
        }
        connect udp_write_led to rover_transport.udp_write_led {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "udp_msgs/UdpPacket";
            qos = "best effort, depth 32";
            rate = "50 Hz";
            source = "rover_led/launch/rover_led.launch.py:104";
            topic = "udp_write/led_channel_<n>";
          }
        }
        connect bms_udp to rover_transport.bms_udp {
          @$::RoverPlatform::RoverPlatformProfile::HwBus {
            address = "192.168.1.201:4444 (392-byte payload)";
            bus = "UDP";
            rate = "BMS dependent";
            source = "rover_battery/config/rover_battery.yaml:6-7";
          }
        }
        connect crsf_udp to rover_transport.crsf_udp {
          @$::RoverPlatform::RoverPlatformProfile::HwBus {
            address = "192.168.1.201:10111, source 192.168.1.1 only";
            bus = "UDP via RUTX11 Serial Utilities Over IP (receiver UART 460800 8N1 on the router USB-UART)";
            rate = "250 Hz frames";
            source = "rover_crsf_teleop/config/rover_crsf_teleop.yaml:4-24,195";
          }
        }
        connect rover_transport.rover_battery_udp_data to rover_battery_udp_data {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "udp_msgs/UdpPacket";
            qos = "depth 100";
            rate = "per packet";
            source = "rover_battery/launch/rover_battery.launch.py:69";
            topic = "rover_battery_udp_data";
          }
        }
        connect rover_transport.led_udp to led_udp {
          @$::RoverPlatform::RoverPlatformProfile::HwBus {
            address = "192.168.77.202:3334 (ch1), 192.168.77.201:3333 (ch2)";
            bus = "UDP";
            rate = "50 Hz";
            source = "rover_led/config/rover_a1_udp_led_channel_1.yaml:3-4";
          }
        }
        connect diagnostics_agg to rover_bringup.diagnostics_agg {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "diagnostic_msgs/DiagnosticArray";
            qos = "reliable";
            rate = "1 Hz";
            source = "rover_diag_manager/config/diagnostic_aggregator.yaml:15";
            topic = "diagnostics_agg";
          }
        }
        connect rover_bringup.teleop_foxglove_cmd_vel_stamped to teleop_foxglove_cmd_vel_stamped {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "geometry_msgs/TwistStamped";
            qos = "reliable";
            rate = "TBD";
            source = "rover_twist_mux/config/rover_twist_mux.yaml:16-19";
            topic = "teleop_foxglove_cmd_vel_stamped";
          }
        }
      }
      part Orchestrator {
        @$::RoverPlatform::RoverPlatformProfile::ExternalActor {
          description = "rover_orchestrator container: driving modes (rover_drive_mode, the only writer of the web-teleop and Nav 2 mux inputs), Nav 2, mission manager.";
        }
        in port teleop_web_cmd_vel_stamped: ~$::RoverPlatform::RoverPlatformInterfaces::TwistStamped;
        out port teleop_driver_interface_cmd_vel_stamped: $::RoverPlatform::RoverPlatformInterfaces::TwistStamped {
          out attribute angular_z redefines angular_z;
          out attribute linear_x redefines linear_x;
          out attribute stamp redefines stamp;
        }
        in port odom: ~$::RoverPlatform::RoverPlatformInterfaces::Odometry;
        out port nav_cmd_vel_stamped: $::RoverPlatform::RoverPlatformInterfaces::TwistStamped {
          out attribute angular_z redefines angular_z;
          out attribute linear_x redefines linear_x;
          out attribute stamp redefines stamp;
        }
        in port motion_lock: ~$::RoverPlatform::RoverPlatformInterfaces::BoolFlag;
        out port drive_mode: $::RoverPlatform::RoverPlatformInterfaces::DriveMode {
          out attribute guard redefines guard;
          out attribute mode redefines mode;
          out attribute reason redefines reason;
        }
      }
      part OperatorStation {
        @$::RoverPlatform::RoverPlatformProfile::ExternalActor {
          description = "Drive UI (rover_drive_interface) and Foxglove Studio on the operator PC, over the LAN/Wi-Fi websocket.";
        }
        in port foxglove_ws_out: ~$::RoverPlatform::RoverPlatformInterfaces::ByteStream;
        out port foxglove_ws_in: $::RoverPlatform::RoverPlatformInterfaces::ByteStream;
      }
      part Localization {
        @$::RoverPlatform::RoverPlatformProfile::FunctionalSubsystem {
          label = "Localization";
        }
        part rover_localization {
          @$::RoverPlatform::RoverPlatformProfile::RosPackage {
            behaviourModel = "";
            executables = "rover_ekf_node; rover_ekf_global_node (GPS); rover_navsat_transform_node (GPS)";
            'language' = "YAML";
            lifecycle = "plain";
            'package' = "rover_localization";
            swrsPrefix = "LOC";
          }
          attribute ekf_frequency : $::ScalarValues::Real default := 50;
          in port tf_static: ~$::RoverPlatform::RoverPlatformInterfaces::Transform;
          in port odometry_wheels: ~$::RoverPlatform::RoverPlatformInterfaces::Odometry;
          out port odom: $::RoverPlatform::RoverPlatformInterfaces::Odometry;
          in port imu_data: ~$::RoverPlatform::RoverPlatformInterfaces::Imu;
          in port gps_heading_imu: ~$::RoverPlatform::RoverPlatformInterfaces::Imu;
          in port gps_fix: ~$::RoverPlatform::RoverPlatformInterfaces::NavSatFix;
          satisfy $::RoverPlatform::SWRS_LOC::'Functional: The rover_localization shall run a robot_localization EKF node named rover_ekf_node whe...';
          satisfy $::RoverPlatform::SWRS_LOC::'Performance: The rover_localization shall configure rover_ekf_node to run at 50.0 Hz in 2D mode (two...';
          satisfy $::RoverPlatform::SWRS_LOC::'Interface: The rover_localization shall fuse only the body linear velocities vx and vy from nav_ms...';
          satisfy $::RoverPlatform::SWRS_LOC::'Interface: The rover_localization shall fuse only the yaw rate (vyaw) from sensor_msgs/Imu on imu/...';
          satisfy $::RoverPlatform::SWRS_LOC::'Interface: The rover_localization shall publish the rover_ekf_node estimate as nav_msgs/Odometry o...';
          satisfy $::RoverPlatform::SWRS_LOC::'Interface: The rover_localization shall make rover_ekf_node the only broadcaster of the odom -> ba...';
          satisfy $::RoverPlatform::SWRS_LOC::'Interface: The rover_localization shall expose the robot_localization services of rover_ekf_node a...';
          satisfy $::RoverPlatform::SWRS_LOC::'Diagnostics: The rover_localization shall enable print_diagnostics on every EKF and remap all locali...';
          satisfy $::RoverPlatform::SWRS_LOC::'Functional: The rover_localization shall, when fuse_gps (default $ROVER_USE_GPS) is one of true/1/y...';
          satisfy $::RoverPlatform::SWRS_LOC::'Functional: The rover_localization shall configure rover_ekf_global_node at 50.0 Hz in 2D mode with...';
          satisfy $::RoverPlatform::SWRS_LOC::'Parameter: The rover_localization shall broadcast map -> odom from rover_ekf_global_node only when...';
          satisfy $::RoverPlatform::SWRS_LOC::'Interface: The rover_localization shall run rover_navsat_transform_node at 10.0 Hz with a 3.0 s st...';
          satisfy $::RoverPlatform::SWRS_LOC::'Parameter: The rover_localization shall pass to rover_gps_heading_node publish_heading true, headi...';
          satisfy $::RoverPlatform::SWRS_LOC::'Parameter: The rover_localization launch file shall accept the arguments use_ekf (default False), ...';
          satisfy $::RoverPlatform::SWRS_LOC::'Parameter: The rover_localization shall ship a valid configuration file for every value accepted b...';
          satisfy $::RoverPlatform::SWRS_LOC::'Performance: The rover_localization shall have an automated test that verifies the EKF publishes odo...';
        }
        part rover_gps_heading {
          @$::RoverPlatform::RoverPlatformProfile::RosPackage {
            behaviourModel = "";
            executables = "rover_gps_heading_node";
            'language' = "C++";
            lifecycle = "plain";
            'package' = "rover_gps_heading";
            swrsPrefix = "GPSH";
          }
          in port odom: ~$::RoverPlatform::RoverPlatformInterfaces::Odometry;
          out port gps_heading_imu: $::RoverPlatform::RoverPlatformInterfaces::Imu;
          in port gps_fix: ~$::RoverPlatform::RoverPlatformInterfaces::NavSatFix;
          satisfy $::RoverPlatform::SWRS_GPSH::'Interface: The rover_gps_heading shall subscribe to sensor_msgs/NavSatFix on the relative topic gp...';
          satisfy $::RoverPlatform::SWRS_GPSH::'Interface: The rover_gps_heading shall subscribe to nav_msgs/Odometry on the relative topic odom (...';
          satisfy $::RoverPlatform::SWRS_GPSH::'Interface: The rover_gps_heading shall publish sensor_msgs/Imu on gps/heading_imu (QoS depth 10) w...';
          satisfy $::RoverPlatform::SWRS_GPSH::'Functional: The rover_gps_heading shall publish gps/heading_imu only after the alignment has reache...';
          satisfy $::RoverPlatform::SWRS_GPSH::'Functional: The rover_gps_heading shall form one course measurement from each straight segment of a...';
          satisfy $::RoverPlatform::SWRS_GPSH::'Functional: The rover_gps_heading shall treat motion as straight only when |vx| >= alignment.min_sp...';
          satisfy $::RoverPlatform::SWRS_GPSH::'Functional: The rover_gps_heading shall ignore a GNSS fix for alignment (and abort the current segm...';
          satisfy $::RoverPlatform::SWRS_GPSH::'Functional: The rover_gps_heading shall latch the circular mean of the most recent alignment.requir...';
          satisfy $::RoverPlatform::SWRS_GPSH::'Functional: The rover_gps_heading shall report the published heading standard deviation as max(min_...';
          satisfy $::RoverPlatform::SWRS_GPSH::'Interface: The rover_gps_heading shall provide the std_srvs/Trigger service gps/reset_heading_alig...';
          satisfy $::RoverPlatform::SWRS_GPSH::'Diagnostics: The rover_gps_heading shall publish a Heading alignment diagnostic (hardware ID RoverGp...';
          satisfy $::RoverPlatform::SWRS_GPSH::'Diagnostics: The rover_gps_heading shall grade the Heading alignment diagnostic OK when aligned, OK ...';
          satisfy $::RoverPlatform::SWRS_GPSH::'Parameter: The rover_gps_heading shall declare publish_heading (default true), min_heading_std_rad...';
          satisfy $::RoverPlatform::SWRS_GPSH::'Lifecycle: The rover_gps_heading shall validate the alignment configuration at construction, rejec...';
          satisfy $::RoverPlatform::SWRS_GPSH::'Build: The rover_gps_heading shall keep its domain and application code (geo math, HeadingAlig...';
        }
        in port tf_static: ~$::RoverPlatform::RoverPlatformInterfaces::Transform {
          in attribute x redefines x;
          in attribute y redefines y;
          in attribute yaw redefines yaw;
        }
        in port odometry_wheels: ~$::RoverPlatform::RoverPlatformInterfaces::Odometry {
          in attribute vx redefines vx;
          in attribute wz redefines wz;
          in attribute x redefines x;
          in attribute y redefines y;
          in attribute yaw redefines yaw;
        }
        out port odom: $::RoverPlatform::RoverPlatformInterfaces::Odometry;
        in port imu_data: ~$::RoverPlatform::RoverPlatformInterfaces::Imu {
          in attribute linear_acceleration_x redefines linear_acceleration_x;
          in attribute angular_velocity_z redefines angular_velocity_z;
          in attribute orientation_yaw redefines orientation_yaw;
        }
        in port gps_fix: ~$::RoverPlatform::RoverPlatformInterfaces::NavSatFix {
          in attribute latitude redefines latitude;
          in attribute altitude redefines altitude;
          in attribute longitude redefines longitude;
        }
        connect gps_fix to rover_localization.gps_fix {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "sensor_msgs/NavSatFix";
            qos = "reliable, volatile, depth 10";
            rate = "TBD (payload)";
            source = "rover_localization/launch/rover_localization.launch.py:231-252";
            topic = "gps/fix";
          }
        }
        connect odometry_wheels to rover_localization.odometry_wheels {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "nav_msgs/Odometry";
            qos = "reliable";
            rate = "50 Hz";
            source = "rover_controller/config/wheel_01_controller.yaml:45,159";
            topic = "odometry/wheels";
          }
        }
        connect rover_localization.odom to odom {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "nav_msgs/Odometry";
            qos = "reliable";
            rate = "50 Hz";
            source = "rover_localization/config/rel_localization.yaml:4-15";
            topic = "odom (+ TF odom->base_footprint)";
          }
        }
        connect gps_fix to rover_gps_heading.gps_fix {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "sensor_msgs/NavSatFix";
            qos = "reliable, volatile, depth 10";
            rate = "TBD (payload)";
            source = "rover_gps_heading/src/infrastructure/rover_gps_heading_node.cpp:104-105";
            topic = "gps/fix";
          }
        }
        connect imu_data to rover_localization.imu_data {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "sensor_msgs/Imu";
            qos = "sensor data";
            rate = "50 Hz";
            source = "rover_controller/config/wheel_01_controller.yaml:150";
            topic = "imu/data";
          }
        }
        connect rover_localization.odom to rover_gps_heading.odom {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "nav_msgs/Odometry";
            qos = "depth 10";
            rate = "50 Hz";
            source = "rover_localization/launch/rover_localization.launch.py:140-167";
            topic = "odom";
          }
        }
        connect rover_gps_heading.gps_heading_imu to rover_localization.gps_heading_imu {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "sensor_msgs/Imu";
            qos = "depth 10";
            rate = "per odom msg after alignment";
            source = "rover_gps_heading/src/infrastructure/ros2_heading_publisher.cpp:39";
            topic = "gps/heading_imu";
          }
        }
        connect tf_static to rover_localization.tf_static {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "tf2_msgs/TFMessage";
            qos = "reliable, transient local";
            rate = "latched";
            source = "rover_description/launch/rover_load_urdf.launch.py:191-199";
            topic = "/tf_static";
          }
        }
      }
      part LedPanels {
        @$::RoverPlatform::RoverPlatformProfile::ExternalActor {
          description = "2x SK9822 LED panels (40 LEDs each) behind UDP controllers.";
        }
        in port led_udp: ~$::RoverPlatform::RoverPlatformInterfaces::ByteStream;
      }
      part ImuSensor {
        @$::RoverPlatform::RoverPlatformProfile::ExternalActor {
          description = "Phidget Spatial MOT0110 IMU (USB/VINT hub port 0).";
        }
        out port imu_raw: $::RoverPlatform::RoverPlatformInterfaces::Imu;
      }
      part HmiStatus {
        @$::RoverPlatform::RoverPlatformProfile::FunctionalSubsystem {
          label = "HMI & Status";
        }
        part rover_led {
          @$::RoverPlatform::RoverPlatformProfile::RosPackage {
            behaviourModel = "";
            executables = "led_controller_node; led_driver_node; udp senders x2";
            'language' = "C++";
            lifecycle = "lifecycle";
            'package' = "rover_led";
            swrsPrefix = "LED";
          }
          out port udp_write_led: $::RoverPlatform::RoverPlatformInterfaces::ByteStream;
          in port led_set_animation: ~$::RoverPlatform::RoverPlatformInterfaces::LedAnimationRequest;
          satisfy $::RoverPlatform::SWRS_LED::'Functional: The rover_led shall give every LED segment four animation layers, ERROR (priority 0), A...';
          satisfy $::RoverPlatform::SWRS_LED::'Functional: The rover_led shall play ALERT-layer animations one after another in FIFO order from a ...';
          satisfy $::RoverPlatform::SWRS_LED::'Interface: The rover_led shall provide the service led/set_animation (rover_msgs/SetLedAnimation) ...';
          satisfy $::RoverPlatform::SWRS_LED::'Interface: The rover_led shall provide the service led/stop_animation (rover_msgs/StopLedAnimation...';
          satisfy $::RoverPlatform::SWRS_LED::'Performance: The rover_led shall publish one led/channel_<n>_frame (sensor_msgs/Image, encoding rgba...';
          satisfy $::RoverPlatform::SWRS_LED::'Interface: The rover_led shall publish led/state (rover_msgs/LedState, reliable, transient-local, ...';
          satisfy $::RoverPlatform::SWRS_LED::'Interface: The rover_led shall publish the loaded animation catalog once at start-up on led/animat...';
          satisfy $::RoverPlatform::SWRS_LED::'Interface: The rover_led shall publish each panel frame additionally on led/channel_<n>_preview (s...';
          satisfy $::RoverPlatform::SWRS_LED::'Parameter: The rover_led shall load the panel layout and animation catalog from the file given by ...';
          satisfy $::RoverPlatform::SWRS_LED::'Functional: The rover_led shall provide a catalog animation for each of the SYS-SR-010 states Emerg...';
          satisfy $::RoverPlatform::SWRS_LED::'Functional: The rover_led shall encode each RGBA panel frame as an SK9822 packet: a 4-byte start fr...';
          satisfy $::RoverPlatform::SWRS_LED::'Functional: The rover_led shall drop, without sending, any received panel frame that is older than ...';
          satisfy $::RoverPlatform::SWRS_LED::'Interface: The rover_led shall subscribe to led/channel_1_frame and led/channel_2_frame (sensor_ms...';
          satisfy $::RoverPlatform::SWRS_LED::'Interface: The rover_led shall provide the service led/set_brightness (rover_msgs/SetLedBrightness...';
          satisfy $::RoverPlatform::SWRS_LED::'Lifecycle: The rover_led driver shall be a managed lifecycle node that, with autostart=true (defau...';
          satisfy $::RoverPlatform::SWRS_LED::'Functional: The rover_led driver shall, when led_control_handshake is true, request LED control fro...';
          satisfy $::RoverPlatform::SWRS_LED::'Diagnostics: The rover_led controller shall publish on diagnostics, hardware id Bumper Led, a Led co...';
          satisfy $::RoverPlatform::SWRS_LED::'Diagnostics: The rover_led driver shall publish on diagnostics, hardware id Bumper Led, a Led driver...';
          satisfy $::RoverPlatform::SWRS_LED::'Build: The rover_led launch file rover_led.launch.py shall load rover_led_driver, rover_led_co...';
          satisfy $::RoverPlatform::SWRS_LED::'Functional: The rover_led shall provide a catalog animation (and a matching rover_msgs/LedAnimation...';
          satisfy $::RoverPlatform::SWRS_LED::'Functional: The rover_led shall document, in the package, the state-to-LED mapping required by SYS-...';
        }
        part rover_diag_manager {
          @$::RoverPlatform::RoverPlatformProfile::RosPackage {
            behaviourModel = "";
            executables = "rover_diag_manager_node; aggregator_node";
            'language' = "C++";
            lifecycle = "plain";
            'package' = "rover_diag_manager";
            swrsPrefix = "DIAG";
          }
          attribute diagnostics_agg_rate : $::ScalarValues::Real default := 1;
          attribute system_status_rate : $::ScalarValues::Real default := 5;
          out port system_status: $::RoverPlatform::RoverPlatformInterfaces::SystemStatus {
            out attribute ram_percent redefines ram_percent;
            out attribute cpu_percent redefines cpu_percent;
            out attribute cpu_temp redefines cpu_temp;
            out attribute disk_percent redefines disk_percent;
          }
          in port diagnostics_from_rover_battery: ~$::RoverPlatform::RoverPlatformInterfaces::Diagnostics;
          out port diagnostics_agg: $::RoverPlatform::RoverPlatformInterfaces::Diagnostics {
            out attribute level redefines level;
          }
          in port diagnostics: ~$::RoverPlatform::RoverPlatformInterfaces::Diagnostics;
          satisfy $::RoverPlatform::SWRS_DIAG::'Interface: The rover_diag_manager shall publish a rover_msgs/SystemStatus message on the relative ...';
          satisfy $::RoverPlatform::SWRS_DIAG::'Performance: The rover_diag_manager shall sample the system metrics and publish system_status at the...';
          satisfy $::RoverPlatform::SWRS_DIAG::'Parameter: The rover_diag_manager shall declare the parameters cpu_usage_warn_threshold (default 9...';
          satisfy $::RoverPlatform::SWRS_DIAG::'Functional: The rover_diag_manager shall re-read the warning thresholds on every sampling tick so t...';
          satisfy $::RoverPlatform::SWRS_DIAG::'Diagnostics: The rover_diag_manager shall publish an OS status diagnostic (hardware ID Rover PC) on ...';
          satisfy $::RoverPlatform::SWRS_DIAG::'Diagnostics: The rover_diag_manager shall grade the OS status diagnostic WARN when any metric is str...';
          satisfy $::RoverPlatform::SWRS_DIAG::'Diagnostics: The rover_diag_manager shall report the OS status diagnostic as STALE with the message ...';
          satisfy $::RoverPlatform::SWRS_DIAG::'Functional: The rover_diag_manager shall read CPU temperature from /sys/class/thermal/thermal_zone0...';
          satisfy $::RoverPlatform::SWRS_DIAG::'Interface: The rover_diag_manager shall launch a diagnostic_aggregator aggregator_node named rover...';
          satisfy $::RoverPlatform::SWRS_DIAG::'Performance: The rover_diag_manager shall configure the diagnostic aggregator to publish diagnostics...';
          satisfy $::RoverPlatform::SWRS_DIAG::'Diagnostics: The rover_diag_manager shall group diagnostic statuses by node-name prefix into the agg...';
          satisfy $::RoverPlatform::SWRS_DIAG::'Diagnostics: The rover_diag_manager shall configure the diagnostic aggregator so that the drive, bat...';
          satisfy $::RoverPlatform::SWRS_DIAG::'Parameter: The rover_diag_manager launch file shall accept the arguments log_level (DEBUG/INFO/WAR...';
          satisfy $::RoverPlatform::SWRS_DIAG::'Lifecycle: The rover_diag_manager shall run rover_diag_manager_node as a plain (non-lifecycle) nod...';
          satisfy $::RoverPlatform::SWRS_DIAG::'Build: The rover_diag_manager shall keep its domain and application layers in a separate stati...';
        }
        out port udp_write_led: $::RoverPlatform::RoverPlatformInterfaces::ByteStream {
          out attribute length redefines length;
        }
        out port system_status: $::RoverPlatform::RoverPlatformInterfaces::SystemStatus {
          out attribute ram_percent redefines ram_percent;
          out attribute cpu_percent redefines cpu_percent;
          out attribute cpu_temp redefines cpu_temp;
          out attribute disk_percent redefines disk_percent;
        }
        in port led_set_animation: ~$::RoverPlatform::RoverPlatformInterfaces::LedAnimationRequest {
          in attribute animation_id redefines animation_id;
          in attribute repeating redefines repeating;
        }
        in port diagnostics_from_rover_battery: ~$::RoverPlatform::RoverPlatformInterfaces::Diagnostics {
          in attribute level redefines level;
        }
        out port diagnostics_agg: $::RoverPlatform::RoverPlatformInterfaces::Diagnostics {
          out attribute level redefines level;
        }
        in port diagnostics: ~$::RoverPlatform::RoverPlatformInterfaces::Diagnostics {
          in attribute level redefines level;
        }
        connect led_set_animation to rover_led.led_set_animation {
          @$::RoverPlatform::RoverPlatformProfile::RosService {
            rate = "on state change";
            service = "led/set_animation";
            source = "rover_safety/behavior_trees/rover_led_safety.xml:97-107";
            srvType = "rover_msgs/SetLedAnimation";
          }
        }
        connect rover_led.udp_write_led to udp_write_led {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "udp_msgs/UdpPacket";
            qos = "best effort, depth 32";
            rate = "50 Hz";
            source = "rover_led/launch/rover_led.launch.py:104";
            topic = "udp_write/led_channel_<n>";
          }
        }
        connect rover_diag_manager.system_status to system_status {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "rover_msgs/SystemStatus";
            qos = "depth 10";
            rate = "5 Hz";
            source = "rover_diag_manager/config/system_diag.yaml:7";
            topic = "system_status";
          }
        }
        connect diagnostics to rover_diag_manager.diagnostics {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "diagnostic_msgs/DiagnosticArray";
            qos = "reliable";
            rate = "1 Hz";
            source = "rover_hardware_interface/src/system_ros_interface/system_ros_interface.cpp:188";
            topic = "diagnostics";
          }
        }
        connect rover_diag_manager.diagnostics_agg to diagnostics_agg {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "diagnostic_msgs/DiagnosticArray";
            qos = "reliable";
            rate = "1 Hz";
            source = "rover_diag_manager/config/diagnostic_aggregator.yaml:15";
            topic = "diagnostics_agg";
          }
        }
        connect diagnostics_from_rover_battery to rover_diag_manager.diagnostics_from_rover_battery {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "diagnostic_msgs/DiagnosticArray";
            qos = "reliable";
            rate = "1 Hz";
            source = "rover_battery/src/infrastructure/rover_battery_node.cpp:41,73";
            topic = "diagnostics";
          }
        }
      }
      part DriveMotors {
        @$::RoverPlatform::RoverPlatformProfile::ExternalActor {
          description = "4x Phidget DCC1000 motor controllers with encoders (USB/VINT).";
        }
        in port motor_command: ~$::RoverPlatform::RoverPlatformInterfaces::WheelCommand;
        out port encoder_feedback: $::RoverPlatform::RoverPlatformInterfaces::WheelState;
      }
      part DriveControl {
        @$::RoverPlatform::RoverPlatformProfile::FunctionalSubsystem {
          label = "Drive & Control";
        }
        part rover_hardware_interface {
          @$::RoverPlatform::RoverPlatformProfile::RosPackage {
            behaviourModel = "EStopLatch";
            executables = "RoverA1System; PhidgetImuSensor; rover_hardware_controller";
            'language' = "C++";
            lifecycle = "ros2_control";
            'package' = "rover_hardware_interface";
            swrsPrefix = "HWI";
          }
          attribute contactor_dropout_tolerance : $::ScalarValues::Real default := 0.5;
          attribute driver_comm_timeout : $::ScalarValues::Real default := 0.3;
          attribute driver_states_update_frequency : $::ScalarValues::Real default := 20;
          attribute gear_ratio : $::ScalarValues::Real default := 23.3;
          attribute imu_data_interval : $::ScalarValues::Real default := 0.008;
          attribute modbus_response_timeout : $::ScalarValues::Real default := 0.15;
          attribute motor_current_limit : $::ScalarValues::Real default := 15;
          attribute motor_failsafe_timeout : $::ScalarValues::Real default := 0.5;
          attribute safety_io_poll_period : $::ScalarValues::Real default := 0.1;
          attribute safety_latch_reset_pulse : $::ScalarValues::Real default := 0.1;
          attribute safety_wdg_kick_period : $::ScalarValues::Real default := 0.2;
          attribute velocity_command_zero_tolerance : $::ScalarValues::Real default := 0.4;
          attribute velocity_state_zero_tolerance : $::ScalarValues::Real default := 0.05;
          in port wheel_velocity_command: ~$::RoverPlatform::RoverPlatformInterfaces::WheelCommand;
          out port wheel_state: $::RoverPlatform::RoverPlatformInterfaces::WheelState {
            out attribute effort redefines effort;
            out attribute position redefines position;
            out attribute velocity redefines velocity;
          }
          in port ui_e_stop_services: ~$::RoverPlatform::RoverPlatformInterfaces::TriggerService;
          in port trip_e_stop: ~$::RoverPlatform::RoverPlatformInterfaces::TriggerService;
          out port safety_status: $::RoverPlatform::RoverPlatformInterfaces::SafetyStatus {
            out attribute motor_contactor_engaged redefines motor_contactor_engaged;
            out attribute hw_e_stop_user_button redefines hw_e_stop_user_button;
            out attribute latch_active redefines latch_active;
            out attribute latch_cause redefines latch_cause;
            out attribute link_healthy redefines link_healthy;
          }
          out port safety_command_echo: $::RoverPlatform::RoverPlatformInterfaces::SafetyCommandEcho {
            out attribute cpu_wdg_heartbeat redefines cpu_wdg_heartbeat;
            out attribute sw_e_stop_user_button redefines sw_e_stop_user_button;
            out attribute sw_e_stop_latch_reset redefines sw_e_stop_latch_reset;
            out attribute sw_e_stop_motor_driver_fault redefines sw_e_stop_motor_driver_fault;
          }
          out port rover_driver_state: $::RoverPlatform::RoverPlatformInterfaces::DriverState {
            out attribute current redefines current;
            out attribute fault redefines fault;
            out attribute temperature redefines temperature;
          }
          out port motor_command: $::RoverPlatform::RoverPlatformInterfaces::WheelCommand {
            out attribute fl redefines fl;
            out attribute fr redefines fr;
            out attribute rl redefines rl;
            out attribute rr redefines rr;
          }
          in port modbus_inputs: ~$::RoverPlatform::RoverPlatformInterfaces::ModbusFrame;
          out port modbus_coils: $::RoverPlatform::RoverPlatformInterfaces::ModbusFrame {
            out attribute coils redefines coils;
            out attribute discrete_inputs redefines discrete_inputs;
          }
          out port imu_state: $::RoverPlatform::RoverPlatformInterfaces::Imu {
            out attribute linear_acceleration_x redefines linear_acceleration_x;
            out attribute angular_velocity_z redefines angular_velocity_z;
            out attribute orientation_yaw redefines orientation_yaw;
          }
          in port imu_raw: ~$::RoverPlatform::RoverPlatformInterfaces::Imu;
          in port encoder_feedback: ~$::RoverPlatform::RoverPlatformInterfaces::WheelState;
          in port e_stop_services: ~$::RoverPlatform::RoverPlatformInterfaces::TriggerService;
          out port diagnostics: $::RoverPlatform::RoverPlatformInterfaces::Diagnostics {
            out attribute level redefines level;
          }
          satisfy $::RoverPlatform::SWRS_HWI::'Interface: The rover_hardware_interface shall export the pluginlib class rover_hardware_interface/...';
          satisfy $::RoverPlatform::SWRS_HWI::'Functional: The rover_hardware_interface shall convert wheel velocity commands from rad/s at the wh...';
          satisfy $::RoverPlatform::SWRS_HWI::'Parameter: The rover_hardware_interface shall configure each DCC1000 with the URDF motor_accelerat...';
          satisfy $::RoverPlatform::SWRS_HWI::'Lifecycle: The rover_hardware_interface shall, in on_configure(), start the safety controller befo...';
          satisfy $::RoverPlatform::SWRS_HWI::'Safety: The rover_hardware_interface shall, in every write() cycle, forward the controller velo...';
          satisfy $::RoverPlatform::SWRS_HWI::'Interface: The rover_hardware_interface shall provide the service hardware_interface/sw_user_e_sto...';
          satisfy $::RoverPlatform::SWRS_HWI::'Performance: The rover_hardware_interface shall command zero velocity to all four drive joints withi...';
          satisfy $::RoverPlatform::SWRS_HWI::'Safety: The rover_hardware_interface shall refuse hardware_interface/sw_user_e_stop_reset (succ...';
          satisfy $::RoverPlatform::SWRS_HWI::'Safety: The rover_hardware_interface shall provide hardware_interface/sw_e_stop_latch_reset (st...';
          satisfy $::RoverPlatform::SWRS_HWI::'Safety: The rover_hardware_interface shall refuse hardware_interface/sw_e_stop_latch_reset (suc...';
          satisfy $::RoverPlatform::SWRS_HWI::'Safety: The rover_hardware_interface shall, on every safety-controller start, drive the heartbe...';
          satisfy $::RoverPlatform::SWRS_HWI::'Safety: The rover_hardware_interface shall toggle the CPU watchdog heartbeat coil (COIL_1) ever...';
          satisfy $::RoverPlatform::SWRS_HWI::'Interface: The rover_hardware_interface shall connect by Modbus TCP to the safety PLC at modbus_ho...';
          satisfy $::RoverPlatform::SWRS_HWI::'Interface: The rover_hardware_interface shall publish rover_msgs/SafetyStatus on hardware_interfac...';
          satisfy $::RoverPlatform::SWRS_HWI::'Safety: The rover_hardware_interface shall set SafetyStatus.link_healthy and AuxIoState.link_he...';
          satisfy $::RoverPlatform::SWRS_HWI::'Performance: The rover_hardware_interface shall publish a change of the hardware E-Stop button, soft...';
          satisfy $::RoverPlatform::SWRS_HWI::'Diagnostics: The rover_hardware_interface shall populate SafetyStatus.latch_cause with the trip sour...';
          satisfy $::RoverPlatform::SWRS_HWI::'Safety: The rover_hardware_interface shall assert the sw_e_stop_motor_driver_fault coil (COIL_3...';
          satisfy $::RoverPlatform::SWRS_HWI::'Safety: The rover_hardware_interface shall latch a welded-contactor fault and inhibit motion wh...';
          satisfy $::RoverPlatform::SWRS_HWI::'Safety: The rover_hardware_interface shall arm each DCC1000 hardware failsafe with motor_failsa...';
          satisfy $::RoverPlatform::SWRS_HWI::'Diagnostics: The rover_hardware_interface shall publish rover_msgs/RoverDriverState on hardware_inte...';
          satisfy $::RoverPlatform::SWRS_HWI::'Diagnostics: The rover_hardware_interface shall publish on diagnostics, with hardware id Rover Syste...';
          satisfy $::RoverPlatform::SWRS_HWI::'Diagnostics: The rover_hardware_interface shall report the safety plc link diagnostic as ERROR when ...';
          satisfy $::RoverPlatform::SWRS_HWI::'Lifecycle: The rover_hardware_interface shall, in on_error(), assert the software E-Stop, broadcas...';
          satisfy $::RoverPlatform::SWRS_HWI::'Interface: The rover_hardware_interface shall provide hardware_interface/aux_output_<0..5>/set (st...';
          satisfy $::RoverPlatform::SWRS_HWI::'Interface: The rover_hardware_interface shall export the pluginlib class rover_hardware_interface/...';
          satisfy $::RoverPlatform::SWRS_HWI::'Diagnostics: The rover_hardware_interface shall publish an IMU diagnostic status on diagnostics at >...';
          satisfy $::RoverPlatform::SWRS_HWI::'Build: The rover_hardware_interface shall keep domain and application code free of ROS, hardwa...';
        }
        part rover_description {
          @$::RoverPlatform::RoverPlatformProfile::RosPackage {
            behaviourModel = "";
            executables = "rover_state_publisher_node (robot_state_publisher)";
            'language' = "xacro";
            lifecycle = "none";
            'package' = "rover_description";
            swrsPrefix = "DESC";
          }
          attribute body_mass : $::ScalarValues::Real default := 39.5373;
          attribute tyre_radius : $::ScalarValues::Real default := 0.1699;
          attribute tyre_y_offset : $::ScalarValues::Real default := -0.00171;
          attribute wheel_joint_velocity_limit : $::ScalarValues::Real default := 10.958;
          attribute wheel_mass : $::ScalarValues::Real default := 4.79607;
          attribute wheel_mount_point_z : $::ScalarValues::Real default := 0.037363;
          attribute wheel_width : $::ScalarValues::Real default := 0.108;
          attribute wheelbase : $::ScalarValues::Real default := 0.503;
          out port tf_static: $::RoverPlatform::RoverPlatformInterfaces::Transform {
            out attribute x redefines x;
            out attribute y redefines y;
            out attribute yaw redefines yaw;
          }
          out port robot_description: $::RoverPlatform::RoverPlatformInterfaces::ByteStream {
            out attribute length redefines length;
          }
          satisfy $::RoverPlatform::SWRS_DESC::'Build: The rover_description shall generate the rover_a1 robot description from urdf/rover_a1....';
          satisfy $::RoverPlatform::SWRS_DESC::'Interface: The rover_description shall define the frame tree base_footprint -> base_link -> body_l...';
          satisfy $::RoverPlatform::SWRS_DESC::'Functional: The rover_description shall place base_link above base_footprint by tyre_radius minus t...';
          satisfy $::RoverPlatform::SWRS_DESC::'Parameter: The rover_description shall take wheel geometry only from config/wheel_01.yaml (tyre_ra...';
          satisfy $::RoverPlatform::SWRS_DESC::'Parameter: The rover_description shall model each wheel joint as a continuous joint about +y with ...';
          satisfy $::RoverPlatform::SWRS_DESC::'Interface: The rover_description shall declare, with use_sim false, a <ros2_control> system compon...';
          satisfy $::RoverPlatform::SWRS_DESC::'Parameter: The rover_description shall set the RoverA1System safety-link parameters modbus_host 19...';
          satisfy $::RoverPlatform::SWRS_DESC::'Parameter: The rover_description shall set the RoverA1System drive-train parameters driver_states_...';
          satisfy $::RoverPlatform::SWRS_DESC::'Parameter: The rover_description shall set the E-Stop reset tolerances velocity_command_zero_toler...';
          satisfy $::RoverPlatform::SWRS_DESC::'Parameter: The rover_description shall declare, with use_sim false, a <ros2_control> sensor compon...';
          satisfy $::RoverPlatform::SWRS_DESC::'Functional: The rover_description shall, in rover_load_urdf.launch.py, take the IMU, lidar and GPS ...';
          satisfy $::RoverPlatform::SWRS_DESC::'Parameter: The rover_description shall declare body_link inertial mass 39.5373 kg and each wheel l...';
          satisfy $::RoverPlatform::SWRS_DESC::'Interface: The rover_description shall start robot_state_publisher as node rover_state_publisher_n...';
          satisfy $::RoverPlatform::SWRS_DESC::'Functional: The rover_description shall, with use_sim true, provide Gazebo sensors on the IMU, lida...';
          satisfy $::RoverPlatform::SWRS_DESC::'Performance: The rover_description shall configure the simulated IMU update_rate at 100 Hz or faster.';
          satisfy $::RoverPlatform::SWRS_DESC::'Parameter: The rover_description shall configure the simulated battery (Gazebo LinearBatteryPlugin...';
        }
        part rover_controller {
          @$::RoverPlatform::RoverPlatformProfile::RosPackage {
            behaviourModel = "SkidSteerKinematics";
            executables = "controller_manager; rover_drive_controller (diff_drive); pid_controller_x4 (SeededPidController); rover_imu_broadcaster; rover_joint_state_broadcaster";
            'language' = "C++/YAML";
            lifecycle = "ros2_control";
            'package' = "rover_controller";
            swrsPrefix = "CTL";
          }
          attribute cm_update_rate : $::ScalarValues::Real default := 100;
          attribute cmd_vel_timeout : $::ScalarValues::Real default := 0.5;
          attribute diff_drive_update_rate : $::ScalarValues::Real default := 50;
          attribute imu_broadcaster_rate : $::ScalarValues::Real default := 50;
          attribute joint_state_rate : $::ScalarValues::Real default := 20;
          attribute max_angular_acceleration : $::ScalarValues::Real default := 3.74;
          attribute max_angular_velocity : $::ScalarValues::Real default := 1.5;
          attribute max_linear_acceleration : $::ScalarValues::Real default := 2.7;
          attribute max_linear_velocity : $::ScalarValues::Real default := 0.95;
          attribute wheel_radius : $::ScalarValues::Real default := 0.1651;
          attribute wheel_separation : $::ScalarValues::Real default := 0.617;
          attribute wheel_separation_multiplier : $::ScalarValues::Real default := 1.659;
          out port wheel_velocity_command: $::RoverPlatform::RoverPlatformInterfaces::WheelCommand {
            out attribute fl redefines fl;
            out attribute fr redefines fr;
            out attribute rl redefines rl;
            out attribute rr redefines rr;
          }
          in port wheel_state: ~$::RoverPlatform::RoverPlatformInterfaces::WheelState;
          in port robot_description: ~$::RoverPlatform::RoverPlatformInterfaces::ByteStream;
          out port odometry_wheels: $::RoverPlatform::RoverPlatformInterfaces::Odometry {
            out attribute vx redefines vx;
            out attribute wz redefines wz;
            out attribute x redefines x;
            out attribute y redefines y;
            out attribute yaw redefines yaw;
          }
          in port imu_state: ~$::RoverPlatform::RoverPlatformInterfaces::Imu;
          out port imu_data: $::RoverPlatform::RoverPlatformInterfaces::Imu {
            out attribute linear_acceleration_x redefines linear_acceleration_x;
            out attribute angular_velocity_z redefines angular_velocity_z;
            out attribute orientation_yaw redefines orientation_yaw;
          }
          in port cmd_vel: ~$::RoverPlatform::RoverPlatformInterfaces::TwistStamped;
          satisfy $::RoverPlatform::SWRS_CTL::'Performance: The rover_controller shall configure controller_manager with update_rate 100 Hz and run...';
          satisfy $::RoverPlatform::SWRS_CTL::'Functional: The rover_controller shall chain rover_drive_controller (diff_drive_controller/DiffDriv...';
          satisfy $::RoverPlatform::SWRS_CTL::'Functional: The rover_controller shall provide SeededPidController, a pid_controller::PidController...';
          satisfy $::RoverPlatform::SWRS_CTL::'Parameter: The rover_controller shall configure every wheel PID with p 0.05, i 1.0, d 0.04, feedfo...';
          satisfy $::RoverPlatform::SWRS_CTL::'Parameter: The rover_controller shall configure rover_drive_controller with wheel_separation 0.617...';
          satisfy $::RoverPlatform::SWRS_CTL::'Interface: The rover_controller shall make rover_drive_controller subscribe to cmd_vel and publish...';
          satisfy $::RoverPlatform::SWRS_CTL::'Performance: The rover_controller shall produce wheel odometry whose travelled-distance error is <= ...';
          satisfy $::RoverPlatform::SWRS_CTL::'Safety: The rover_controller shall limit rover_drive_controller body velocity to linear.x withi...';
          satisfy $::RoverPlatform::SWRS_CTL::'Performance: The rover_controller shall limit rover_drive_controller linear acceleration and deceler...';
          satisfy $::RoverPlatform::SWRS_CTL::'Safety: The rover_controller shall configure rover_drive_controller with cmd_vel_timeout 0.5 s ...';
          satisfy $::RoverPlatform::SWRS_CTL::'Safety: The rover_controller shall keep the outer-wheel reference at full linear plus full angu...';
          satisfy $::RoverPlatform::SWRS_CTL::'Build: The rover_controller shall keep wheel_radius and wheel_separation in config/wheel_01_co...';
          satisfy $::RoverPlatform::SWRS_CTL::'Interface: The rover_controller shall run rover_joint_state_broadcaster (joint_state_broadcaster) ...';
          satisfy $::RoverPlatform::SWRS_CTL::'Interface: The rover_controller shall run rover_imu_broadcaster (imu_sensor_broadcaster) publishin...';
          satisfy $::RoverPlatform::SWRS_CTL::'Performance: The rover_controller shall publish imu/data at 100 Hz or faster.';
          satisfy $::RoverPlatform::SWRS_CTL::'Build: The rover_controller shall give every controller an update_rate that divides the contro...';
          satisfy $::RoverPlatform::SWRS_CTL::'Lifecycle: The rover_controller shall, in rover_controller.launch.py, activate rover_joint_state_b...';
          satisfy $::RoverPlatform::SWRS_CTL::'Performance: The rover_controller shall, in rover_controller.launch.py, start ros2_control_node with...';
          satisfy $::RoverPlatform::SWRS_CTL::'Functional: The rover_controller shall, in rover_controller.launch.py, select the controller config...';
          satisfy $::RoverPlatform::SWRS_CTL::'Safety: The rover_controller shall make its tuning tools (wheel_step_response, wheel_odom_calib...';
        }
        in port ui_e_stop_services: ~$::RoverPlatform::RoverPlatformInterfaces::TriggerService {
          in attribute success redefines success;
        }
        in port trip_e_stop: ~$::RoverPlatform::RoverPlatformInterfaces::TriggerService {
          in attribute success redefines success;
        }
        out port tf_static: $::RoverPlatform::RoverPlatformInterfaces::Transform {
          out attribute x redefines x;
          out attribute y redefines y;
          out attribute yaw redefines yaw;
        }
        out port safety_status: $::RoverPlatform::RoverPlatformInterfaces::SafetyStatus {
          out attribute motor_contactor_engaged redefines motor_contactor_engaged;
          out attribute hw_e_stop_user_button redefines hw_e_stop_user_button;
          out attribute latch_active redefines latch_active;
          out attribute latch_cause redefines latch_cause;
          out attribute link_healthy redefines link_healthy;
        }
        out port safety_command_echo: $::RoverPlatform::RoverPlatformInterfaces::SafetyCommandEcho {
          out attribute cpu_wdg_heartbeat redefines cpu_wdg_heartbeat;
          out attribute sw_e_stop_user_button redefines sw_e_stop_user_button;
          out attribute sw_e_stop_latch_reset redefines sw_e_stop_latch_reset;
          out attribute sw_e_stop_motor_driver_fault redefines sw_e_stop_motor_driver_fault;
        }
        out port rover_driver_state: $::RoverPlatform::RoverPlatformInterfaces::DriverState {
          out attribute current redefines current;
          out attribute fault redefines fault;
          out attribute temperature redefines temperature;
        }
        out port odometry_wheels: $::RoverPlatform::RoverPlatformInterfaces::Odometry {
          out attribute vx redefines vx;
          out attribute wz redefines wz;
          out attribute x redefines x;
          out attribute y redefines y;
          out attribute yaw redefines yaw;
        }
        out port motor_command: $::RoverPlatform::RoverPlatformInterfaces::WheelCommand {
          out attribute fl redefines fl;
          out attribute fr redefines fr;
          out attribute rl redefines rl;
          out attribute rr redefines rr;
        }
        in port modbus_inputs: ~$::RoverPlatform::RoverPlatformInterfaces::ModbusFrame {
          in attribute coils redefines coils;
          in attribute discrete_inputs redefines discrete_inputs;
        }
        out port modbus_coils: $::RoverPlatform::RoverPlatformInterfaces::ModbusFrame {
          out attribute coils redefines coils;
          out attribute discrete_inputs redefines discrete_inputs;
        }
        in port imu_raw: ~$::RoverPlatform::RoverPlatformInterfaces::Imu {
          in attribute linear_acceleration_x redefines linear_acceleration_x;
          in attribute angular_velocity_z redefines angular_velocity_z;
          in attribute orientation_yaw redefines orientation_yaw;
        }
        out port imu_data: $::RoverPlatform::RoverPlatformInterfaces::Imu {
          out attribute linear_acceleration_x redefines linear_acceleration_x;
          out attribute angular_velocity_z redefines angular_velocity_z;
          out attribute orientation_yaw redefines orientation_yaw;
        }
        in port encoder_feedback: ~$::RoverPlatform::RoverPlatformInterfaces::WheelState {
          in attribute effort redefines effort;
          in attribute position redefines position;
          in attribute velocity redefines velocity;
        }
        in port e_stop_services: ~$::RoverPlatform::RoverPlatformInterfaces::TriggerService {
          in attribute success redefines success;
        }
        out port diagnostics: $::RoverPlatform::RoverPlatformInterfaces::Diagnostics {
          out attribute level redefines level;
        }
        in port cmd_vel: ~$::RoverPlatform::RoverPlatformInterfaces::TwistStamped {
          in attribute angular_z redefines angular_z;
          in attribute linear_x redefines linear_x;
          in attribute stamp redefines stamp;
        }
        connect rover_hardware_interface.safety_status to safety_status {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "rover_msgs/SafetyStatus";
            qos = "reliable, volatile, KeepLast(1)";
            rate = "20 Hz";
            source = "rover_hardware_interface/src/system_ros_interface/system_ros_interface.cpp:169-184";
            topic = "hardware_interface/safety_status";
          }
        }
        connect rover_controller.imu_data to imu_data {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "sensor_msgs/Imu";
            qos = "sensor data";
            rate = "50 Hz";
            source = "rover_controller/config/wheel_01_controller.yaml:150";
            topic = "imu/data";
          }
        }
        connect rover_hardware_interface.motor_command to motor_command {
          @$::RoverPlatform::RoverPlatformProfile::HwBus {
            address = "channels FL=0 FR=1 RL=4 RR=5";
            bus = "Phidget22 USB/VINT";
            rate = "100 Hz";
            source = "rover_hardware_interface/src/rover_driver/rover_a1_driver.cpp:73-80";
          }
        }
        connect rover_hardware_interface.safety_command_echo to safety_command_echo {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "rover_msgs/SafetyCommandEcho";
            qos = "reliable, volatile, KeepLast(1)";
            rate = "20 Hz";
            source = "rover_hardware_interface/src/system_ros_interface/system_ros_interface.cpp:169-184";
            topic = "hardware_interface/safety_command_echo";
          }
        }
        connect rover_hardware_interface.modbus_coils to modbus_coils {
          @$::RoverPlatform::RoverPlatformProfile::HwBus {
            address = "192.168.88.11:502";
            bus = "Modbus TCP FC5";
            rate = "5 Hz watchdog kick + on demand";
            source = "rover_description/urdf/rover_a1/rover_a1_macro.urdf.xacro:148-170";
          }
        }
        connect trip_e_stop to rover_hardware_interface.trip_e_stop {
          @$::RoverPlatform::RoverPlatformProfile::RosService {
            rate = "on battery verdict";
            service = "hardware_interface/sw_user_e_stop_set";
            source = "rover_safety/behavior_trees/rover_safety.xml:4-16";
            srvType = "std_srvs/Trigger";
          }
        }
        connect e_stop_services to rover_hardware_interface.e_stop_services {
          @$::RoverPlatform::RoverPlatformProfile::RosService {
            rate = "on switch edge";
            service = "hardware_interface/sw_user_e_stop_set|reset, sw_e_stop_latch_reset";
            source = "rover_hardware_interface/src/rover_system/rover_system.cpp:130-143";
            srvType = "std_srvs/Trigger";
          }
        }
        connect rover_hardware_interface.imu_state to rover_controller.imu_state {
          @$::RoverPlatform::RoverPlatformProfile::Ros2Control {
            interfaces = "imu/* state interfaces";
            rate = "125 Hz sampling";
            source = "rover_description/urdf/common/imu.urdf.xacro:98-104";
          }
        }
        connect rover_hardware_interface.wheel_state to rover_controller.wheel_state {
          @$::RoverPlatform::RoverPlatformProfile::Ros2Control {
            interfaces = "<joint>/position|velocity|effort state interfaces";
            rate = "100 Hz (encoders refresh at 10 Hz)";
            source = "rover_description/urdf/rover_a1/rover_a1_macro.urdf.xacro:104-129";
          }
        }
        connect encoder_feedback to rover_hardware_interface.encoder_feedback {
          @$::RoverPlatform::RoverPlatformProfile::HwBus {
            address = "encoder 1024 CPR";
            bus = "Phidget22 USB/VINT";
            rate = "10 Hz";
            source = "rover_description/urdf/rover_a1/rover_a1_macro.urdf.xacro:190,254";
          }
        }
        connect modbus_inputs to rover_hardware_interface.modbus_inputs {
          @$::RoverPlatform::RoverPlatformProfile::HwBus {
            address = "192.168.88.11:502";
            bus = "Modbus TCP FC1/FC2";
            rate = "10 Hz poll";
            source = "rover_description/urdf/rover_a1/rover_a1_macro.urdf.xacro:178";
          }
        }
        connect rover_hardware_interface.rover_driver_state to rover_driver_state {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "rover_msgs/RoverDriverState";
            qos = "reliable, KeepLast(5)";
            rate = "20 Hz";
            source = "rover_hardware_interface/src/system_ros_interface/system_ros_interface.cpp:148-150";
            topic = "hardware_interface/rover_driver_state";
          }
        }
        connect rover_description.robot_description to rover_controller.robot_description {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "std_msgs/String";
            qos = "reliable, transient local";
            rate = "latched";
            source = "rover_description/launch/rover_load_urdf.launch.py:191-199";
            topic = "robot_description";
          }
        }
        connect ui_e_stop_services to rover_hardware_interface.ui_e_stop_services {
          @$::RoverPlatform::RoverPlatformProfile::RosService {
            rate = "on demand";
            service = "hardware_interface/sw_user_e_stop_set|reset, sw_e_stop_latch_reset";
            source = "rover_bringup/launch/rover_web_bridges.launch.py:106-149";
            srvType = "std_srvs/Trigger";
          }
        }
        connect rover_description.tf_static to tf_static {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "tf2_msgs/TFMessage";
            qos = "reliable, transient local";
            rate = "latched";
            source = "rover_description/launch/rover_load_urdf.launch.py:191-199";
            topic = "/tf_static";
          }
        }
        connect rover_controller.wheel_velocity_command to rover_hardware_interface.wheel_velocity_command {
          @$::RoverPlatform::RoverPlatformProfile::Ros2Control {
            interfaces = "<joint>/velocity command interfaces";
            rate = "100 Hz";
            source = "rover_controller/config/wheel_01_controller.yaml:13";
          }
        }
        connect rover_controller.odometry_wheels to odometry_wheels {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "nav_msgs/Odometry";
            qos = "reliable";
            rate = "50 Hz";
            source = "rover_controller/config/wheel_01_controller.yaml:45,159";
            topic = "odometry/wheels";
          }
        }
        connect rover_hardware_interface.diagnostics to diagnostics {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "diagnostic_msgs/DiagnosticArray";
            qos = "reliable";
            rate = "1 Hz";
            source = "rover_hardware_interface/src/system_ros_interface/system_ros_interface.cpp:188";
            topic = "diagnostics";
          }
        }
        connect imu_raw to rover_hardware_interface.imu_raw {
          @$::RoverPlatform::RoverPlatformProfile::HwBus {
            address = "hub port 0";
            bus = "Phidget22 USB/VINT";
            rate = "125 Hz";
            source = "rover_description/urdf/common/imu.urdf.xacro:101-104";
          }
        }
        connect cmd_vel to rover_controller.cmd_vel {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "geometry_msgs/TwistStamped";
            qos = "reliable";
            rate = "follows active source";
            source = "rover_twist_mux/launch/rover_twist_mux.launch.py:110-117";
            topic = "cmd_vel";
          }
        }
      }
      part CommandArbitration {
        @$::RoverPlatform::RoverPlatformProfile::FunctionalSubsystem {
          label = "Command Arbitration";
        }
        part rover_twist_mux {
          @$::RoverPlatform::RoverPlatformProfile::RosPackage {
            behaviourModel = "CommandArbitration";
            executables = "rover_twist_mux_node; rover_motion_lock_node; rover_command_freshness_node";
            'language' = "C++/YAML";
            lifecycle = "plain";
            'package' = "rover_twist_mux";
            swrsPrefix = "MUX";
          }
          attribute motion_lock_gpio_timeout : $::ScalarValues::Real default := 1;
          attribute motion_lock_rate : $::ScalarValues::Real default := 10;
          attribute prio_driver_interface : $::ScalarValues::Real default := 8;
          attribute prio_elrs : $::ScalarValues::Real default := 110;
          attribute prio_joystick : $::ScalarValues::Real default := 100;
          attribute prio_motion_lock : $::ScalarValues::Real default := 200;
          attribute prio_nav : $::ScalarValues::Real default := 5;
          attribute timeout_driver_interface : $::ScalarValues::Real default := 0.3;
          attribute timeout_elrs : $::ScalarValues::Real default := 0.5;
          attribute timeout_joystick : $::ScalarValues::Real default := 0.5;
          attribute timeout_motion_lock : $::ScalarValues::Real default := 0.5;
          attribute timeout_nav : $::ScalarValues::Real default := 0.5;
          in port teleop_foxglove_cmd_vel_stamped: ~$::RoverPlatform::RoverPlatformInterfaces::TwistStamped;
          in port teleop_elrs_cmd_vel_stamped: ~$::RoverPlatform::RoverPlatformInterfaces::TwistStamped;
          in port teleop_driver_interface_cmd_vel_stamped: ~$::RoverPlatform::RoverPlatformInterfaces::TwistStamped;
          in port safety_status: ~$::RoverPlatform::RoverPlatformInterfaces::SafetyStatus;
          in port safety_command_echo: ~$::RoverPlatform::RoverPlatformInterfaces::SafetyCommandEcho;
          in port nav_cmd_vel_stamped: ~$::RoverPlatform::RoverPlatformInterfaces::TwistStamped;
          out port motion_lock: $::RoverPlatform::RoverPlatformInterfaces::BoolFlag {
            out attribute data redefines data;
          }
          out port cmd_vel: $::RoverPlatform::RoverPlatformInterfaces::TwistStamped {
            out attribute angular_z redefines angular_z;
            out attribute linear_x redefines linear_x;
            out attribute stamp redefines stamp;
          }
          satisfy $::RoverPlatform::SWRS_MUX::'Interface: The rover_twist_mux shall publish the arbitrated velocity command as geometry_msgs/Twis...';
          satisfy $::RoverPlatform::SWRS_MUX::'Functional: The rover_twist_mux shall accept velocity commands from four inputs with strictly order...';
          satisfy $::RoverPlatform::SWRS_MUX::'Safety: The rover_twist_mux shall stop forwarding an input when no message has been received on...';
          satisfy $::RoverPlatform::SWRS_MUX::'Safety: The rover_twist_mux shall mask every velocity input while motion_lock is true or has no...';
          satisfy $::RoverPlatform::SWRS_MUX::'Interface: The rover_twist_mux shall publish the motion lock state as std_msgs/Bool on motion_lock...';
          satisfy $::RoverPlatform::SWRS_MUX::'Interface: The rover_twist_mux shall subscribe hardware_interface/safety_status (rover_msgs/Safety...';
          satisfy $::RoverPlatform::SWRS_MUX::'Safety: The rover_twist_mux shall assert motion_lock (true) from start-up until both a SafetySt...';
          satisfy $::RoverPlatform::SWRS_MUX::'Safety: The rover_twist_mux shall assert motion_lock when the older of the two safety messages ...';
          satisfy $::RoverPlatform::SWRS_MUX::'Safety: The rover_twist_mux shall assert motion_lock while SafetyStatus.link_healthy is false, ...';
          satisfy $::RoverPlatform::SWRS_MUX::'Safety: The rover_twist_mux shall assert motion_lock while any enabled stop condition is active...';
          satisfy $::RoverPlatform::SWRS_MUX::'Parameter: The rover_twist_mux shall treat an open motor contactor as a lock condition only when r...';
          satisfy $::RoverPlatform::SWRS_MUX::'Parameter: The rover_twist_mux shall reject a motion-lock publish_frequency outside [1.0, 100.0] H...';
          satisfy $::RoverPlatform::SWRS_MUX::'Diagnostics: The rover_twist_mux shall report a Motion lock diagnostic (hardware ID Motion Lock) tha...';
          satisfy $::RoverPlatform::SWRS_MUX::'Functional: The rover_twist_mux shall relay a Driver UI command from teleop_driver_interface_cmd_ve...';
          satisfy $::RoverPlatform::SWRS_MUX::'Functional: The rover_twist_mux shall, in rover_command_freshness_node, let the filters latency bas...';
          satisfy $::RoverPlatform::SWRS_MUX::'Interface: The rover_twist_mux shall, in rover_command_freshness_node, subscribe and publish geome...';
          satisfy $::RoverPlatform::SWRS_MUX::'Parameter: The rover_twist_mux shall, in rover_command_freshness_node, treat the nodes parameters ...';
          satisfy $::RoverPlatform::SWRS_MUX::'Diagnostics: The rover_twist_mux shall, in rover_command_freshness_node, report a Command freshness ...';
          satisfy $::RoverPlatform::SWRS_MUX::'Safety: The rover_twist_mux shall publish motion_lock immediately when a received SafetyStatus ...';
          satisfy $::RoverPlatform::SWRS_MUX::'Interface: The rover_twist_mux shall publish which input is currently selected (or none, or locked...';
        }
        part rover_crsf_teleop {
          @$::RoverPlatform::RoverPlatformProfile::RosPackage {
            behaviourModel = "";
            executables = "RoverCrsfTeleopNode (component in rover_crsf_container)";
            'language' = "C++";
            lifecycle = "lifecycle";
            'package' = "rover_crsf_teleop";
            swrsPrefix = "CRSF";
          }
          out port teleop_elrs_cmd_vel_stamped: $::RoverPlatform::RoverPlatformInterfaces::TwistStamped {
            out attribute angular_z redefines angular_z;
            out attribute linear_x redefines linear_x;
            out attribute stamp redefines stamp;
          }
          in port safety_status: ~$::RoverPlatform::RoverPlatformInterfaces::SafetyStatus;
          in port rc_raw_udp: ~$::RoverPlatform::RoverPlatformInterfaces::ByteStream;
          out port e_stop_services: $::RoverPlatform::RoverPlatformInterfaces::TriggerService {
            out attribute success redefines success;
          }
          satisfy $::RoverPlatform::SWRS_CRSF::'Interface: The rover_crsf_teleop shall subscribe the raw CRSF byte stream as udp_msgs/UdpPacket da...';
          satisfy $::RoverPlatform::SWRS_CRSF::'Functional: The rover_crsf_teleop shall decode CRSF RC_CHANNELS_PACKED (0x16, 16 x 11-bit channels)...';
          satisfy $::RoverPlatform::SWRS_CRSF::'Interface: The rover_crsf_teleop shall publish velocity commands as geometry_msgs/TwistStamped wit...';
          satisfy $::RoverPlatform::SWRS_CRSF::'Performance: The rover_crsf_teleop shall run its control loop every 20 ms (50 Hz) while ACTIVE and p...';
          satisfy $::RoverPlatform::SWRS_CRSF::'Functional: The rover_crsf_teleop shall map linear_x_channel (3) to linear.x in [-0.95, 0.95] m/s a...';
          satisfy $::RoverPlatform::SWRS_CRSF::'Functional: The rover_crsf_teleop shall shape each stick axis as (1 - e) * m + e * m^3 of the norma...';
          satisfy $::RoverPlatform::SWRS_CRSF::'Performance: The rover_crsf_teleop shall scale linear.x and angular.z by the same factor so that |v|...';
          satisfy $::RoverPlatform::SWRS_CRSF::'Parameter: The rover_crsf_teleop shall declare linear_x_out_min/max and angular_z_out_min/max defa...';
          satisfy $::RoverPlatform::SWRS_CRSF::'Safety: The rover_crsf_teleop shall, when the commanded velocity becomes zero (stick released, ...';
          satisfy $::RoverPlatform::SWRS_CRSF::'Safety: The rover_crsf_teleop shall judge the RC link healthy only while an RC channels frame w...';
          satisfy $::RoverPlatform::SWRS_CRSF::'Safety: The rover_crsf_teleop shall, when the RC link is judged lost while ACTIVE, command zero...';
          satisfy $::RoverPlatform::SWRS_CRSF::'Safety: The rover_crsf_teleop shall, on a change of the e_stop_channel (5) switch position, cal...';
          satisfy $::RoverPlatform::SWRS_CRSF::'Safety: The rover_crsf_teleop shall ignore switch positions for switch_settle_frames (100 ticks...';
          satisfy $::RoverPlatform::SWRS_CRSF::'Safety: The rover_crsf_teleop shall keep requesting hardware_interface/sw_user_e_stop_set while...';
          satisfy $::RoverPlatform::SWRS_CRSF::'Lifecycle: The rover_crsf_teleop shall be a managed lifecycle node that validates parameters and s...';
          satisfy $::RoverPlatform::SWRS_CRSF::'Build: The rover_crsf_teleop shall, in rover_crsf_teleop.launch.py, load rover_udp_drivers Udp...';
          satisfy $::RoverPlatform::SWRS_CRSF::'Parameter: The rover_crsf_teleop shall fail the configure transition when a channel role is outsid...';
          satisfy $::RoverPlatform::SWRS_CRSF::'Safety: The rover_crsf_teleop shall start an RC calibration session only while the node is not ...';
          satisfy $::RoverPlatform::SWRS_CRSF::'Interface: The rover_crsf_teleop shall provide the calibration services rc/calibration/start (rove...';
          satisfy $::RoverPlatform::SWRS_CRSF::'Interface: The rover_crsf_teleop shall, when publish_rc_topics is true, echo decoded frames on rc/...';
          satisfy $::RoverPlatform::SWRS_CRSF::'Diagnostics: The rover_crsf_teleop shall publish diagnostics under hardware ID RC Receiver with the ...';
          satisfy $::RoverPlatform::SWRS_CRSF::'Safety: The rover_crsf_teleop shall accept RC input only from the configured router address: ro...';
        }
        in port teleop_foxglove_cmd_vel_stamped: ~$::RoverPlatform::RoverPlatformInterfaces::TwistStamped {
          in attribute angular_z redefines angular_z;
          in attribute linear_x redefines linear_x;
          in attribute stamp redefines stamp;
        }
        in port teleop_driver_interface_cmd_vel_stamped: ~$::RoverPlatform::RoverPlatformInterfaces::TwistStamped {
          in attribute angular_z redefines angular_z;
          in attribute linear_x redefines linear_x;
          in attribute stamp redefines stamp;
        }
        in port safety_status: ~$::RoverPlatform::RoverPlatformInterfaces::SafetyStatus {
          in attribute motor_contactor_engaged redefines motor_contactor_engaged;
          in attribute hw_e_stop_user_button redefines hw_e_stop_user_button;
          in attribute latch_active redefines latch_active;
          in attribute latch_cause redefines latch_cause;
          in attribute link_healthy redefines link_healthy;
        }
        in port safety_command_echo: ~$::RoverPlatform::RoverPlatformInterfaces::SafetyCommandEcho {
          in attribute cpu_wdg_heartbeat redefines cpu_wdg_heartbeat;
          in attribute sw_e_stop_user_button redefines sw_e_stop_user_button;
          in attribute sw_e_stop_latch_reset redefines sw_e_stop_latch_reset;
          in attribute sw_e_stop_motor_driver_fault redefines sw_e_stop_motor_driver_fault;
        }
        in port rc_raw_udp: ~$::RoverPlatform::RoverPlatformInterfaces::ByteStream {
          in attribute length redefines length;
        }
        in port nav_cmd_vel_stamped: ~$::RoverPlatform::RoverPlatformInterfaces::TwistStamped {
          in attribute angular_z redefines angular_z;
          in attribute linear_x redefines linear_x;
          in attribute stamp redefines stamp;
        }
        out port motion_lock: $::RoverPlatform::RoverPlatformInterfaces::BoolFlag {
          out attribute data redefines data;
        }
        out port e_stop_services: $::RoverPlatform::RoverPlatformInterfaces::TriggerService {
          out attribute success redefines success;
        }
        out port cmd_vel: $::RoverPlatform::RoverPlatformInterfaces::TwistStamped {
          out attribute angular_z redefines angular_z;
          out attribute linear_x redefines linear_x;
          out attribute stamp redefines stamp;
        }
        connect safety_status to rover_twist_mux.safety_status {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "rover_msgs/SafetyStatus";
            qos = "reliable, volatile, KeepLast(1)";
            rate = "20 Hz";
            source = "rover_hardware_interface/src/system_ros_interface/system_ros_interface.cpp:169-184";
            topic = "hardware_interface/safety_status";
          }
        }
        connect rover_crsf_teleop.teleop_elrs_cmd_vel_stamped to rover_twist_mux.teleop_elrs_cmd_vel_stamped {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "geometry_msgs/TwistStamped";
            qos = "reliable, depth 10";
            rate = "50 Hz";
            source = "rover_twist_mux/config/rover_twist_mux.yaml:21-24";
            topic = "teleop_elrs_cmd_vel_stamped";
          }
        }
        connect rover_twist_mux.motion_lock to motion_lock {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "std_msgs/Bool";
            qos = "reliable";
            rate = "10 Hz";
            source = "rover_twist_mux/config/rover_motion_lock.yaml:3";
            topic = "motion_lock";
          }
        }
        connect rc_raw_udp to rover_crsf_teleop.rc_raw_udp {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "udp_msgs/UdpPacket";
            qos = "reliable, depth 100";
            rate = "one per datagram, ~250 Hz";
            source = "rover_crsf_teleop/launch/rover_crsf_teleop.launch.py:28,121";
            topic = "rc/raw_udp";
          }
        }
        connect rover_crsf_teleop.e_stop_services to e_stop_services {
          @$::RoverPlatform::RoverPlatformProfile::RosService {
            rate = "on switch edge";
            service = "hardware_interface/sw_user_e_stop_set|reset, sw_e_stop_latch_reset";
            source = "rover_hardware_interface/src/rover_system/rover_system.cpp:130-143";
            srvType = "std_srvs/Trigger";
          }
        }
        connect teleop_foxglove_cmd_vel_stamped to rover_twist_mux.teleop_foxglove_cmd_vel_stamped {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "geometry_msgs/TwistStamped";
            qos = "reliable";
            rate = "TBD";
            source = "rover_twist_mux/config/rover_twist_mux.yaml:16-19";
            topic = "teleop_foxglove_cmd_vel_stamped";
          }
        }
        connect safety_command_echo to rover_twist_mux.safety_command_echo {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "rover_msgs/SafetyCommandEcho";
            qos = "reliable, volatile, KeepLast(1)";
            rate = "20 Hz";
            source = "rover_hardware_interface/src/system_ros_interface/system_ros_interface.cpp:169-184";
            topic = "hardware_interface/safety_command_echo";
          }
        }
        connect nav_cmd_vel_stamped to rover_twist_mux.nav_cmd_vel_stamped {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "geometry_msgs/TwistStamped";
            qos = "reliable";
            rate = "TBD (Nav 2 controller rate)";
            source = "rover_twist_mux/config/rover_twist_mux.yaml:6-9";
            topic = "nav_cmd_vel_stamped";
          }
        }
        connect safety_status to rover_crsf_teleop.safety_status {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "rover_msgs/SafetyStatus";
            qos = "reliable, volatile, KeepLast(1)";
            rate = "20 Hz";
            source = "rover_hardware_interface/src/system_ros_interface/system_ros_interface.cpp:169-184";
            topic = "hardware_interface/safety_status";
          }
        }
        connect teleop_driver_interface_cmd_vel_stamped to rover_twist_mux.teleop_driver_interface_cmd_vel_stamped {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "geometry_msgs/TwistStamped";
            qos = "reliable";
            rate = "10 Hz";
            source = "rover_twist_mux/config/rover_twist_mux.yaml:10-15";
            topic = "teleop_driver_interface_cmd_vel_stamped";
          }
        }
        connect rover_twist_mux.cmd_vel to cmd_vel {
          @$::RoverPlatform::RoverPlatformProfile::RosTopic {
            msgType = "geometry_msgs/TwistStamped";
            qos = "reliable";
            rate = "follows active source";
            source = "rover_twist_mux/launch/rover_twist_mux.launch.py:110-117";
            topic = "cmd_vel";
          }
        }
      }
      part BmsBridge {
        @$::RoverPlatform::RoverPlatformProfile::ExternalActor {
          description = "Daly BMS read by an ESP32, forwarded over UDP.";
        }
        out port bms_udp: $::RoverPlatform::RoverPlatformInterfaces::ByteStream;
      }
      connect DriveMotors.encoder_feedback to DriveControl.encoder_feedback {
        @$::RoverPlatform::RoverPlatformProfile::HwBus {
          address = "encoder 1024 CPR";
          bus = "Phidget22 USB/VINT";
          rate = "10 Hz";
          source = "rover_description/urdf/rover_a1/rover_a1_macro.urdf.xacro:190,254";
        }
      }
      connect DriveControl.rover_driver_state to Safety.rover_driver_state {
        @$::RoverPlatform::RoverPlatformProfile::RosTopic {
          msgType = "rover_msgs/RoverDriverState";
          qos = "reliable, KeepLast(5)";
          rate = "20 Hz";
          source = "rover_hardware_interface/src/system_ros_interface/system_ros_interface.cpp:148-150";
          topic = "hardware_interface/rover_driver_state";
        }
      }
      connect Power.diagnostics to HmiStatus.diagnostics_from_rover_battery {
        @$::RoverPlatform::RoverPlatformProfile::RosTopic {
          msgType = "diagnostic_msgs/DiagnosticArray";
          qos = "reliable";
          rate = "1 Hz";
          source = "rover_battery/src/infrastructure/rover_battery_node.cpp:41,73";
          topic = "diagnostics";
        }
      }
      connect HmiStatus.system_status to Safety.system_status {
        @$::RoverPlatform::RoverPlatformProfile::RosTopic {
          msgType = "rover_msgs/SystemStatus";
          qos = "depth 10";
          rate = "5 Hz";
          source = "rover_diag_manager/config/system_diag.yaml:7";
          topic = "system_status";
        }
      }
      connect PlatformServices.teleop_foxglove_cmd_vel_stamped to CommandArbitration.teleop_foxglove_cmd_vel_stamped {
        @$::RoverPlatform::RoverPlatformProfile::RosTopic {
          msgType = "geometry_msgs/TwistStamped";
          qos = "reliable";
          rate = "TBD";
          source = "rover_twist_mux/config/rover_twist_mux.yaml:16-19";
          topic = "teleop_foxglove_cmd_vel_stamped";
        }
      }
      connect Orchestrator.teleop_driver_interface_cmd_vel_stamped to CommandArbitration.teleop_driver_interface_cmd_vel_stamped {
        @$::RoverPlatform::RoverPlatformProfile::RosTopic {
          msgType = "geometry_msgs/TwistStamped";
          qos = "reliable";
          rate = "10 Hz";
          source = "rover_twist_mux/config/rover_twist_mux.yaml:10-15";
          topic = "teleop_driver_interface_cmd_vel_stamped";
        }
      }
      connect ImuSensor.imu_raw to DriveControl.imu_raw {
        @$::RoverPlatform::RoverPlatformProfile::HwBus {
          address = "hub port 0";
          bus = "Phidget22 USB/VINT";
          rate = "125 Hz";
          source = "rover_description/urdf/common/imu.urdf.xacro:101-104";
        }
      }
      connect PlatformServices.led_udp to LedPanels.led_udp {
        @$::RoverPlatform::RoverPlatformProfile::HwBus {
          address = "192.168.77.202:3334 (ch1), 192.168.77.201:3333 (ch2)";
          bus = "UDP";
          rate = "50 Hz";
          source = "rover_led/config/rover_a1_udp_led_channel_1.yaml:3-4";
        }
      }
      connect SensorPayload.gps_fix to Localization.gps_fix {
        @$::RoverPlatform::RoverPlatformProfile::RosTopic {
          msgType = "sensor_msgs/NavSatFix";
          qos = "reliable, volatile, depth 10";
          rate = "TBD (payload)";
          source = "rover_gps_heading/src/infrastructure/rover_gps_heading_node.cpp:104-105";
          topic = "gps/fix";
        }
      }
      connect DriveControl.modbus_coils to SafetyPlc.modbus_coils {
        @$::RoverPlatform::RoverPlatformProfile::HwBus {
          address = "192.168.88.11:502";
          bus = "Modbus TCP FC5";
          rate = "5 Hz watchdog kick + on demand";
          source = "rover_description/urdf/rover_a1/rover_a1_macro.urdf.xacro:148-170";
        }
      }
      connect PlatformServices.ui_e_stop_services to DriveControl.ui_e_stop_services {
        @$::RoverPlatform::RoverPlatformProfile::RosService {
          rate = "on demand";
          service = "hardware_interface/sw_user_e_stop_set|reset, sw_e_stop_latch_reset";
          source = "rover_bringup/launch/rover_web_bridges.launch.py:106-149";
          srvType = "std_srvs/Trigger";
        }
      }
      connect PlatformServices.rc_raw_udp to CommandArbitration.rc_raw_udp {
        @$::RoverPlatform::RoverPlatformProfile::RosTopic {
          msgType = "udp_msgs/UdpPacket";
          qos = "reliable, depth 100";
          rate = "one per datagram, ~250 Hz";
          source = "rover_crsf_teleop/launch/rover_crsf_teleop.launch.py:28,121";
          topic = "rc/raw_udp";
        }
      }
      connect DriveControl.motor_command to DriveMotors.motor_command {
        @$::RoverPlatform::RoverPlatformProfile::HwBus {
          address = "channels FL=0 FR=1 RL=4 RR=5";
          bus = "Phidget22 USB/VINT";
          rate = "100 Hz";
          source = "rover_hardware_interface/src/rover_driver/rover_a1_driver.cpp:73-80";
        }
      }
      connect OperatorStation.foxglove_ws_in to PlatformServices.foxglove_ws_in {
        @$::RoverPlatform::RoverPlatformProfile::HwBus {
          address = "TCP 8765 (foxglove_bridge), 9090 (rosbridge)";
          bus = "Websocket (LAN/Wi-Fi)";
          rate = "10 Hz UI stream";
          source = "rover_bringup/launch/rover_web_bridges.launch.py:54-198";
        }
      }
      connect Power.battery_status to Safety.battery_status {
        @$::RoverPlatform::RoverPlatformProfile::RosTopic {
          msgType = "sensor_msgs/BatteryState";
          qos = "depth 5";
          rate = "per BMS packet (no fixed rate)";
          source = "rover_battery/src/infrastructure/ros2_battery_state_publisher.cpp:83-85";
          topic = "rover_battery/battery_status";
        }
      }
      connect HmiStatus.diagnostics_agg to PlatformServices.diagnostics_agg {
        @$::RoverPlatform::RoverPlatformProfile::RosTopic {
          msgType = "diagnostic_msgs/DiagnosticArray";
          qos = "reliable";
          rate = "1 Hz";
          source = "rover_diag_manager/config/diagnostic_aggregator.yaml:15";
          topic = "diagnostics_agg";
        }
      }
      connect CommandArbitration.motion_lock to Orchestrator.motion_lock {
        @$::RoverPlatform::RoverPlatformProfile::RosTopic {
          msgType = "std_msgs/Bool";
          qos = "reliable";
          rate = "10 Hz";
          source = "rover_twist_mux/config/rover_motion_lock.yaml:3";
          topic = "motion_lock";
        }
      }
      connect RcTransmitter.crsf_udp to PlatformServices.crsf_udp {
        @$::RoverPlatform::RoverPlatformProfile::HwBus {
          address = "192.168.1.201:10111, source 192.168.1.1 only";
          bus = "UDP via RUTX11 Serial Utilities Over IP (receiver UART 460800 8N1 on the router USB-UART)";
          rate = "250 Hz frames";
          source = "rover_crsf_teleop/config/rover_crsf_teleop.yaml:4-24,195";
        }
      }
      connect Orchestrator.drive_mode to PlatformServices.drive_mode {
        @$::RoverPlatform::RoverPlatformProfile::RosTopic {
          msgType = "rover_msgs/DriveMode";
          qos = "reliable, transient_local";
          rate = "on change";
          source = "rover_msgs/msg/DriveMode.msg:1";
          topic = "drive_mode";
        }
      }
      connect DriveControl.safety_status to Safety.safety_status {
        @$::RoverPlatform::RoverPlatformProfile::RosTopic {
          msgType = "rover_msgs/SafetyStatus";
          qos = "reliable, volatile, KeepLast(1)";
          rate = "20 Hz";
          source = "rover_hardware_interface/src/system_ros_interface/system_ros_interface.cpp:169-184";
          topic = "hardware_interface/safety_status";
        }
      }
      connect DriveControl.safety_status to CommandArbitration.safety_status {
        @$::RoverPlatform::RoverPlatformProfile::RosTopic {
          msgType = "rover_msgs/SafetyStatus";
          qos = "reliable, volatile, KeepLast(1)";
          rate = "20 Hz";
          source = "rover_hardware_interface/src/system_ros_interface/system_ros_interface.cpp:169-184";
          topic = "hardware_interface/safety_status";
        }
      }
      connect HmiStatus.udp_write_led to PlatformServices.udp_write_led {
        @$::RoverPlatform::RoverPlatformProfile::RosTopic {
          msgType = "udp_msgs/UdpPacket";
          qos = "best effort, depth 32";
          rate = "50 Hz";
          source = "rover_led/launch/rover_led.launch.py:104";
          topic = "udp_write/led_channel_<n>";
        }
      }
      connect DriveControl.odometry_wheels to Localization.odometry_wheels {
        @$::RoverPlatform::RoverPlatformProfile::RosTopic {
          msgType = "nav_msgs/Odometry";
          qos = "reliable";
          rate = "50 Hz";
          source = "rover_controller/config/wheel_01_controller.yaml:45,159";
          topic = "odometry/wheels";
        }
      }
      connect CommandArbitration.cmd_vel to DriveControl.cmd_vel {
        @$::RoverPlatform::RoverPlatformProfile::RosTopic {
          msgType = "geometry_msgs/TwistStamped";
          qos = "reliable";
          rate = "follows active source";
          source = "rover_twist_mux/launch/rover_twist_mux.launch.py:110-117";
          topic = "cmd_vel";
        }
      }
      connect PlatformServices.rover_battery_udp_data to Power.rover_battery_udp_data {
        @$::RoverPlatform::RoverPlatformProfile::RosTopic {
          msgType = "udp_msgs/UdpPacket";
          qos = "depth 100";
          rate = "per packet";
          source = "rover_battery/launch/rover_battery.launch.py:69";
          topic = "rover_battery_udp_data";
        }
      }
      connect CommandArbitration.e_stop_services to DriveControl.e_stop_services {
        @$::RoverPlatform::RoverPlatformProfile::RosService {
          rate = "on switch edge";
          service = "hardware_interface/sw_user_e_stop_set|reset, sw_e_stop_latch_reset";
          source = "rover_hardware_interface/src/rover_system/rover_system.cpp:130-143";
          srvType = "std_srvs/Trigger";
        }
      }
      connect DriveControl.tf_static to Localization.tf_static {
        @$::RoverPlatform::RoverPlatformProfile::RosTopic {
          msgType = "tf2_msgs/TFMessage";
          qos = "reliable, transient local";
          rate = "latched";
          source = "rover_description/launch/rover_load_urdf.launch.py:191-199";
          topic = "/tf_static";
        }
      }
      connect Localization.odom to Orchestrator.odom {
        @$::RoverPlatform::RoverPlatformProfile::RosTopic {
          msgType = "nav_msgs/Odometry";
          qos = "reliable";
          rate = "50 Hz";
          source = "rover_localization/config/rel_localization.yaml:4-15";
          topic = "odom (+ TF odom->base_footprint)";
        }
      }
      connect DriveControl.safety_command_echo to Safety.safety_command_echo {
        @$::RoverPlatform::RoverPlatformProfile::RosTopic {
          msgType = "rover_msgs/SafetyCommandEcho";
          qos = "reliable, volatile, KeepLast(1)";
          rate = "20 Hz";
          source = "rover_hardware_interface/src/system_ros_interface/system_ros_interface.cpp:169-184";
          topic = "hardware_interface/safety_command_echo";
        }
      }
      connect DriveControl.imu_data to Localization.imu_data {
        @$::RoverPlatform::RoverPlatformProfile::RosTopic {
          msgType = "sensor_msgs/Imu";
          qos = "sensor data";
          rate = "50 Hz";
          source = "rover_controller/config/wheel_01_controller.yaml:150";
          topic = "imu/data";
        }
      }
      connect DriveControl.safety_command_echo to CommandArbitration.safety_command_echo {
        @$::RoverPlatform::RoverPlatformProfile::RosTopic {
          msgType = "rover_msgs/SafetyCommandEcho";
          qos = "reliable, volatile, KeepLast(1)";
          rate = "20 Hz";
          source = "rover_hardware_interface/src/system_ros_interface/system_ros_interface.cpp:169-184";
          topic = "hardware_interface/safety_command_echo";
        }
      }
      connect Orchestrator.nav_cmd_vel_stamped to CommandArbitration.nav_cmd_vel_stamped {
        @$::RoverPlatform::RoverPlatformProfile::RosTopic {
          msgType = "geometry_msgs/TwistStamped";
          qos = "reliable";
          rate = "TBD (Nav 2 controller rate)";
          source = "rover_twist_mux/config/rover_twist_mux.yaml:6-9";
          topic = "nav_cmd_vel_stamped";
        }
      }
      connect PlatformServices.foxglove_ws_out to OperatorStation.foxglove_ws_out {
        @$::RoverPlatform::RoverPlatformProfile::HwBus {
          address = "TCP 8765 (foxglove_bridge), 9090 (rosbridge)";
          bus = "Websocket (LAN/Wi-Fi)";
          rate = "per topic";
          source = "rover_bringup/launch/rover_web_bridges.launch.py:54-198";
        }
      }
      connect Safety.trip_e_stop to DriveControl.trip_e_stop {
        @$::RoverPlatform::RoverPlatformProfile::RosService {
          rate = "on battery verdict";
          service = "hardware_interface/sw_user_e_stop_set";
          source = "rover_safety/behavior_trees/rover_safety.xml:4-16";
          srvType = "std_srvs/Trigger";
        }
      }
      connect PlatformServices.teleop_web_cmd_vel_stamped to Orchestrator.teleop_web_cmd_vel_stamped {
        @$::RoverPlatform::RoverPlatformProfile::RosTopic {
          msgType = "geometry_msgs/TwistStamped";
          qos = "reliable";
          rate = "10 Hz";
          source = "rover_bringup/launch/rover_web_bridges.launch.py:74-78";
          topic = "teleop_web_cmd_vel_stamped";
        }
      }
      connect SafetyPlc.modbus_inputs to DriveControl.modbus_inputs {
        @$::RoverPlatform::RoverPlatformProfile::HwBus {
          address = "192.168.88.11:502";
          bus = "Modbus TCP FC1/FC2";
          rate = "10 Hz poll";
          source = "rover_description/urdf/rover_a1/rover_a1_macro.urdf.xacro:178";
        }
      }
      connect Safety.led_set_animation to HmiStatus.led_set_animation {
        @$::RoverPlatform::RoverPlatformProfile::RosService {
          rate = "on state change";
          service = "led/set_animation";
          source = "rover_safety/behavior_trees/rover_led_safety.xml:97-107";
          srvType = "rover_msgs/SetLedAnimation";
        }
      }
      connect BmsBridge.bms_udp to PlatformServices.bms_udp {
        @$::RoverPlatform::RoverPlatformProfile::HwBus {
          address = "192.168.1.201:4444 (392-byte payload)";
          bus = "UDP";
          rate = "BMS dependent";
          source = "rover_battery/config/rover_battery.yaml:6-7";
        }
      }
      connect DriveControl.diagnostics to HmiStatus.diagnostics {
        @$::RoverPlatform::RoverPlatformProfile::RosTopic {
          msgType = "diagnostic_msgs/DiagnosticArray";
          qos = "reliable";
          rate = "1 Hz";
          source = "rover_hardware_interface/src/system_ros_interface/system_ros_interface.cpp:188";
          topic = "diagnostics";
        }
      }
    }
    part def RoverPlatformDeployment {
      part SensorsContainer {
        @$::RoverPlatform::RoverPlatformProfile::ExecutionNode {
          description = "rover-a1-sensors container (GNSS, lidar)";
          source = "rover_gps_heading/src/infrastructure/rover_gps_heading_node.cpp:104-105";
        }
      }
      part SafetyPlcNode {
        @$::RoverPlatform::RoverPlatformProfile::ExecutionNode {
          description = "Safety PLC (Modbus TCP 192.168.88.11:502)";
          source = "rover_description/urdf/rover_a1/rover_a1_macro.urdf.xacro:148-149";
        }
      }
      part PlatformContainer {
        @$::RoverPlatform::RoverPlatformProfile::ExecutionNode {
          description = "rover-a1-platform container: ARM64 rover computer, Ubuntu 26.04, ROS 2 Lyrical, rmw_zenoh_cpp";
          source = "rover_docker/rover_a1_platform/Dockerfile:1-5 (outside rover_ros)";
        }
      }
      part PhidgetVintHub {
        @$::RoverPlatform::RoverPlatformProfile::ExecutionNode {
          description = "Phidget VINT hub: 4x DCC1000 + MOT0110 IMU (USB)";
          source = "rover_hardware_interface/src/rover_driver/rover_a1_driver.cpp:73-80";
        }
      }
      part OrchestratorContainer {
        @$::RoverPlatform::RoverPlatformProfile::ExecutionNode {
          description = "rover-a1-orchestrator container (Nav 2, missions)";
          source = "rover_twist_mux/config/rover_twist_mux.yaml:5-9";
        }
      }
      part OperatorPc {
        @$::RoverPlatform::RoverPlatformProfile::ExecutionNode {
          description = "Operator PC / browser (Drive UI, Foxglove)";
          source = "rover_bringup/launch/rover_web_bridges.launch.py:54-198";
        }
      }
      part LedControllers {
        @$::RoverPlatform::RoverPlatformProfile::ExecutionNode {
          description = "SK9822 panel UDP controllers 192.168.77.201/202";
          source = "rover_led/config/rover_a1_udp_led_channel_1.yaml:3-4";
        }
      }
      part Esp32BmsBridge {
        @$::RoverPlatform::RoverPlatformProfile::ExecutionNode {
          description = "ESP32 reading the Daly BMS, UDP sender";
          source = "rover_battery/config/rover_battery.yaml:6-7";
        }
      }
      part ElrsReceiver {
        @$::RoverPlatform::RoverPlatformProfile::ExecutionNode {
          description = "ExpressLRS receiver, USB-UART on the RUTX11 router; Serial Utilities Over IP sends CRSF as UDP to PlatformContainer 192.168.1.201:10111 (source 192.168.1.1)";
          source = "rover_crsf_teleop/config/rover_crsf_teleop.yaml:4-24";
        }
      }
    }
    package RoverPlatformAlloc {
      abstract allocation def RoverPlatformAlloc {
        end source: $::RoverPlatform::RoverPlatformArch;
        end target: $::RoverPlatform::RoverPlatformDeployment;
      }
      allocation def SoftwareToExecution specializes $::RoverPlatform::RoverPlatformAlloc::RoverPlatformAlloc {
        allocation 'SoftwareToExecution#1' allocate source.DriveControl.rover_hardware_interface to target.PlatformContainer;
        allocation 'SoftwareToExecution#2' allocate source.DriveControl.rover_description to target.PlatformContainer;
        allocation 'SoftwareToExecution#3' allocate source.DriveControl.rover_controller to target.PlatformContainer;
        allocation 'SoftwareToExecution#4' allocate source.CommandArbitration.rover_twist_mux to target.PlatformContainer;
        allocation 'SoftwareToExecution#5' allocate source.CommandArbitration.rover_crsf_teleop to target.PlatformContainer;
        allocation 'SoftwareToExecution#6' allocate source.PlatformServices.rover_msgs to target.PlatformContainer;
        allocation 'SoftwareToExecution#7' allocate source.Safety.rover_safety to target.PlatformContainer;
        allocation 'SoftwareToExecution#8' allocate source.Power.rover_battery to target.PlatformContainer;
        allocation 'SoftwareToExecution#9' allocate source.LedPanels to target.LedControllers;
        allocation 'SoftwareToExecution#10' allocate source.HmiStatus.rover_led to target.PlatformContainer;
        allocation 'SoftwareToExecution#11' allocate source.HmiStatus.rover_diag_manager to target.PlatformContainer;
        allocation 'SoftwareToExecution#12' allocate source.Localization.rover_localization to target.PlatformContainer;
        allocation 'SoftwareToExecution#13' allocate source.Localization.rover_gps_heading to target.PlatformContainer;
        allocation 'SoftwareToExecution#14' allocate source.PlatformServices.rover_bringup to target.PlatformContainer;
        allocation 'SoftwareToExecution#15' allocate source.PlatformServices.rover_transport to target.PlatformContainer;
        allocation 'SoftwareToExecution#16' allocate source.BmsBridge to target.Esp32BmsBridge;
        allocation 'SoftwareToExecution#17' allocate source.DriveMotors to target.PhidgetVintHub;
        allocation 'SoftwareToExecution#18' allocate source.ImuSensor to target.PhidgetVintHub;
        allocation 'SoftwareToExecution#19' allocate source.Orchestrator to target.OrchestratorContainer;
        allocation 'SoftwareToExecution#20' allocate source.OperatorStation to target.OperatorPc;
        allocation 'SoftwareToExecution#21' allocate source.RcTransmitter to target.ElrsReceiver;
        allocation 'SoftwareToExecution#22' allocate source.SafetyPlc to target.SafetyPlcNode;
        allocation 'SoftwareToExecution#23' allocate source.SensorPayload to target.SensorsContainer;
      }
    }
  }
