function build_mode_manager()
%BUILD_MODE_MANAGER PROPOSED platform mode state machine (SYS-SR-010, SYS-SR-009, SYS-SR-017).
%   behaviour/PlatformModeManager.slx, 10 Hz (rover_safety tick rate). No such state machine
%   exists in rover_ros today (SWR-SAF-018..022 Proposed); this is the design proposal that
%   rover_safety would implement.
%   Inputs:  ready          controllers active and safety link healthy (bringup done)
%            e_stop         SW E-stop or PLC latch active (safety_status / command echo)
%            fault          driver fault, stale safety link or diagnostics ERROR
%            reset_request  operator acknowledges a fault
%            active_source  twist_mux winner: 0 none, 1 ELRS, 2 joystick, 3 driver UI, 4 nav
%                           (twist_mux does not publish this today: SWR-MUX-020)
%            soc            battery state of charge, 0..1
%            charging       1 while charging
%   Outputs: mode           0 Boot, 1 Idle, 2 Teleoperation, 3 Autonomous,
%                           4 EmergencyStop, 5 Fault, 6 LowBattery
%            led_animation  rover_msgs/LedAnimation id (255 = none defined, SWR-LED-020)
%            motion_allowed 0 blocks all velocity commands
%            safe_stop      1 in LowBattery.SafeStop
%   Thresholds: SYS-SR-017 gives [TBD]. ASSUMPTION: the rover_led_safety thresholds
%   (led_battery_low, led_battery_critical) are used until the TBDs are set.

% Copyright 2026 Mechatronics Academy
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%
%     http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.

name = 'PlatformModeManager';
file = mbse.newModel(name, 0.1);
ws = get_param(name, 'ModelWorkspace');
assignin(ws, 'soc_low', mbse.param('led_battery_low'));
assignin(ws, 'soc_safe_stop', mbse.param('led_battery_critical'));

ch = mbse.addChart([name '/ModeManager'], [300 60 520 260], 0.1);
ins = {'ready', 'e_stop', 'fault', 'reset_request', 'active_source', 'soc', 'charging'};
outNames = {'mode', 'led_animation', 'motion_allowed', 'safe_stop'};
mbse.sfData(ch, ins, 'Input');
mbse.sfData(ch, outNames, 'Output');
mbse.sfData(ch, {'soc_low', 'soc_safe_stop'}, 'Parameter');

act = @(mode, led, allowed, stop) sprintf( ...
    'entry: mode = %d; led_animation = %d; motion_allowed = %d; safe_stop = %d;', mode, led, allowed, stop);

boot = mbse.sfState(ch, sprintf('Boot\n%s', act(0, 255, 0, 0)), [40 40 260 80]);
estop = mbse.sfState(ch, sprintf('EmergencyStop\n%s', act(4, 0, 0, 0)), [420 40 260 80]);
fault = mbse.sfState(ch, sprintf('Fault\n%s', act(5, 2, 0, 0)), [800 40 260 80]);

oper = mbse.sfState(ch, 'Operational', [40 200 640 260]);
idle = mbse.sfState(oper, sprintf('Idle\n%s', act(1, 1, 1, 0)), [70 250 170 90]);
tele = mbse.sfState(oper, sprintf('Teleoperation\n%s', act(2, 4, 1, 0)), [280 250 170 90]);
auto = mbse.sfState(oper, sprintf('Autonomous\n%s', act(3, 12, 1, 0)), [490 250 170 90]);

low = mbse.sfState(ch, 'LowBattery', [760 200 460 260]);
warn = mbse.sfState(low, sprintf('Warning\n%s', act(6, 5, 1, 0)), [790 250 190 90]);
stop = mbse.sfState(low, sprintf('SafeStop\n%s', act(6, 6, 0, 1)), [1010 250 190 90]);

mbse.sfDefault(ch, boot);
mbse.sfDefault(oper, idle);
mbse.sfDefault(low, warn);

% Boot
mbse.sfTrans(ch, boot, estop, '[ready > 0.5 && e_stop > 0.5]', 1, 3, 9);
mbse.sfTrans(ch, boot, oper, '[ready > 0.5]', 2, 6, 0);
% Operational: E-stop has priority over fault over low battery
mbse.sfTrans(ch, oper, estop, '[e_stop > 0.5]', 1, 1, 6);
mbse.sfTrans(ch, oper, fault, '[fault > 0.5]', 2, 2, 7);
mbse.sfTrans(ch, oper, low, '[soc < soc_low && charging < 0.5]', 3, 3, 9);
% LowBattery
mbse.sfTrans(ch, low, estop, '[e_stop > 0.5]', 1, 11, 5);
mbse.sfTrans(ch, low, fault, '[fault > 0.5]', 2, 0, 6);
mbse.sfTrans(ch, low, oper, '[soc >= soc_low || charging > 0.5]', 3, 7, 5);
mbse.sfTrans(low, warn, stop, '[soc < soc_safe_stop]', 1, 3, 9);
% EmergencyStop: leaves only when the latch and SW E-stop are cleared (explicit reset)
mbse.sfTrans(ch, estop, fault, '[e_stop < 0.5 && fault > 0.5]', 1, 3, 9);
mbse.sfTrans(ch, estop, oper, '[e_stop < 0.5]', 2, 7, 1);
% Fault
mbse.sfTrans(ch, fault, estop, '[e_stop > 0.5]', 1, 8, 4);
mbse.sfTrans(ch, fault, oper, '[fault < 0.5 && reset_request > 0.5]', 2, 7, 2);
% Inside Operational
mbse.sfTrans(oper, idle, tele, '[active_source >= 1 && active_source <= 3]', 1, 2, 10);
mbse.sfTrans(oper, idle, auto, '[active_source == 4]', 2, 1, 11);
mbse.sfTrans(oper, tele, auto, '[active_source == 4]', 1, 2, 10);
mbse.sfTrans(oper, tele, idle, '[active_source == 0]', 2, 8, 4);
mbse.sfTrans(oper, auto, tele, '[active_source >= 1 && active_source <= 3]', 1, 8, 4);
mbse.sfTrans(oper, auto, idle, '[active_source == 0]', 2, 7, 5);

for k = 1:numel(ins)
    y = 60 + (k - 1) * 30;
    add_block('simulink/Sources/In1', [name '/' ins{k}], 'Position', [120 y 150 y + 14], ...
        'Interpolate', 'off');
    add_line(name, [ins{k} '/1'], ['ModeManager/' num2str(k)], 'autorouting', 'on');
end
for k = 1:numel(outNames)
    y = 60 + (k - 1) * 40;
    add_block('simulink/Sinks/Out1', [name '/' outNames{k}], 'Position', [680 y 710 y + 14]);
    h = add_line(name, ['ModeManager/' num2str(k)], [outNames{k} '/1'], 'autorouting', 'on');
    set_param(h, 'Name', outNames{k});
end

set_param(name, 'Description', ['PROPOSED platform mode manager for rover_safety ' ...
    '(SYS-SR-010). Generated by scripts/build_mode_manager.m.']);
save_system(name, file);
close_system(name, 0);
fprintf('build_mode_manager: %s\n', file);
end
