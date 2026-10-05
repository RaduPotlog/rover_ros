function build_estop_latch()
%BUILD_ESTOP_LATCH Executable as-built model of the E-stop chain.
%   behaviour/EStopLatch.slx, 100 Hz. Sources: rover_arch/SAFETY_CHAIN.md,
%   rover_hardware_interface (emergency_stop.cpp, rover_control_loop_use_case.cpp:106-136,
%   rover_safety_controller.cpp:486-492 - the IO cache is refreshed by the poll thread only).
%     SwEStopRequest (Stateflow) - HWI sw_user_e_stop_set/reset services; reset only with
%                                  zero command and zero wheel state (emergency_stop.cpp).
%     HwiModbusWrite             - coil writes reach the PLC after link_delay; the latch
%                                  reset becomes a safety_latch_reset_pulse long pulse. As
%                                  built, the latch reset has no zero-velocity check
%                                  (P.latch_reset_requires_zero = 0).
%     PlcLatch (Stateflow)       - set-dominant SR latch: set by HW button, SW coil or CPU
%                                  watchdog loss; latched at start-up; contactor = ~latch.
%     IoPoll                     - HWI reads PLC IO every safety_io_poll_period and
%                                  publishes safety_status at driver_states_update_frequency.
%     WriteGate                  - write() commands zero while the polled SW E-stop or latch
%                                  is active (100 Hz).

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

name = 'EStopLatch';
file = mbse.newModel(name, 0.01);
P = mbse.paramStruct({'safety_io_poll_period', 'driver_states_update_frequency', ...
    'safety_latch_reset_pulse', 'modbus_response_timeout'});
P.dt = 0.01;
P.link_delay = 0;                 % ideal link; worst case = modbus_response_timeout (tests)
P.latch_reset_requires_zero = 0;  % as built (Deviation vs SAFETY_CHAIN.md section 5)
P.publish_phase = 0;              % 20 Hz publish timer offset vs the 10 Hz poll (unknown; tests sweep it)
assignin(get_param(name, 'ModelWorkspace'), 'P', P);

add_block('simulink/Sources/Digital Clock', [name '/Clock'], 'SampleTime', '0.01', ...
    'Position', [40 20 90 50]);

% SW E-stop request held by the hardware interface
ch = mbse.addChart([name '/SwEStopRequest'], [300 60 460 180], 0.01);
mbse.sfData(ch, {'sw_set', 'sw_reset', 'cmd_zero', 'wheels_zero'}, 'Input');
mbse.sfData(ch, {'sw_coil'}, 'Output');
rel = mbse.sfState(ch, sprintf('Released\nentry: sw_coil = 0;'), [40 60 140 70]);
ass = mbse.sfState(ch, sprintf('Asserted\nentry: sw_coil = 1;'), [300 60 140 70]);
mbse.sfDefault(ch, rel);
mbse.sfTrans(ch, rel, ass, '[sw_set > 0.5]', 1, 2, 10);
mbse.sfTrans(ch, ass, rel, '[sw_reset > 0.5 && cmd_zero > 0.5 && wheels_zero > 0.5]', 1, 8, 4);

writeCode = strjoin({
    'function [sw_coil_plc, reset_pulse] = HwiModbusWrite(t, sw_coil, latch_reset, cmd_zero, wheels_zero, P)'
    'persistent buf idx prev_req pulse_until'
    'n = max(1, round(P.link_delay / P.dt) + 1);'
    'if isempty(buf), buf = zeros(1, 64); idx = 1; prev_req = 0; pulse_until = -inf; end'
    'buf(idx) = sw_coil;'
    'sw_coil_plc = buf(mod(idx - n, 64) + 1);'
    'idx = mod(idx, 64) + 1;'
    'accept = P.latch_reset_requires_zero < 0.5 || (cmd_zero > 0.5 && wheels_zero > 0.5);'
    'if latch_reset > 0.5 && prev_req < 0.5 && accept'
    '    pulse_until = t + P.link_delay + P.safety_latch_reset_pulse;'
    'end'
    'prev_req = latch_reset;'
    'reset_pulse = double(t >= pulse_until - P.safety_latch_reset_pulse - 1e-9 && t < pulse_until - 1e-9);'
    }, newline);
mbse.addFcn([name '/HwiModbusWrite'], writeCode, {'P'}, [560 60 720 200]);

% PLC set-dominant latch
pl = mbse.addChart([name '/PlcLatch'], [820 60 980 200], 0.01);
mbse.sfData(pl, {'hw_button', 'sw_coil_plc', 'reset_pulse', 'cpu_wdg_ok'}, 'Input');
mbse.sfData(pl, {'latch_active', 'contactor_engaged'}, 'Output');
lat = mbse.sfState(pl, sprintf('Latched\nentry: latch_active = 1; contactor_engaged = 0;'), [40 60 200 80]);
clr = mbse.sfState(pl, sprintf('Clear\nentry: latch_active = 0; contactor_engaged = 1;'), [360 60 200 80]);
mbse.sfDefault(pl, lat);
setCond = '(hw_button > 0.5 || sw_coil_plc > 0.5 || cpu_wdg_ok < 0.5)';
mbse.sfTrans(pl, clr, lat, ['[' setCond ']'], 1, 8, 4);
mbse.sfTrans(pl, lat, clr, ['[reset_pulse > 0.5 && ~' setCond ']'], 1, 2, 10);

pollCode = strjoin({
    'function [seen_sw, seen_latch, pub_hw_button, pub_latch, pub_sw_e_stop] = IoPoll(t, hw_button, sw_coil_plc, latch_active, P)'
    'persistent s_sw s_latch s_hw next_poll next_pub p_hw p_latch p_sw'
    'if isempty(s_sw), s_sw = 0; s_latch = 1; s_hw = 0; next_poll = 0; next_pub = P.publish_phase; p_hw = 0; p_latch = 1; p_sw = 0; end'
    'if t >= next_poll - 1e-9'
    '    s_sw = sw_coil_plc; s_latch = latch_active; s_hw = hw_button;'
    '    next_poll = t + P.safety_io_poll_period;'
    'end'
    'if t >= next_pub - 1e-9'
    '    p_hw = s_hw; p_latch = s_latch; p_sw = s_sw;'
    '    next_pub = t + 1 / P.driver_states_update_frequency;'
    'end'
    'seen_sw = s_sw; seen_latch = s_latch;'
    'pub_hw_button = p_hw; pub_latch = p_latch; pub_sw_e_stop = p_sw;'
    }, newline);
mbse.addFcn([name '/IoPoll'], pollCode, {'P'}, [1080 60 1240 220]);

gateCode = strjoin({
    'function [e_stop_active, wheel_cmd_zero] = WriteGate(seen_sw, seen_latch)'
    '% RoverControlLoopUseCase::updateEStopActiveState / decideWriteCommand'
    'e_stop_active = double(seen_sw > 0.5 || seen_latch > 0.5);'
    'wheel_cmd_zero = e_stop_active;'
    }, newline);
mbse.addFcn([name '/WriteGate'], gateCode, {}, [1340 60 1500 160]);

% Root inputs
ins = {'hw_button', 'sw_set', 'sw_reset', 'latch_reset', 'cmd_zero', 'wheels_zero', 'cpu_wdg_ok'};
for k = 1:numel(ins)
    y = 40 + (k - 1) * 40;
    add_block('simulink/Sources/In1', [name '/' ins{k}], 'Position', [120 y 150 y + 14], ...
        'Interpolate', 'off');
end
L = @(a, b) add_line(name, a, b, 'autorouting', 'on');
L('sw_set/1', 'SwEStopRequest/1');
L('sw_reset/1', 'SwEStopRequest/2');
L('cmd_zero/1', 'SwEStopRequest/3');
L('wheels_zero/1', 'SwEStopRequest/4');
L('Clock/1', 'HwiModbusWrite/1');
L('SwEStopRequest/1', 'HwiModbusWrite/2');
L('latch_reset/1', 'HwiModbusWrite/3');
L('cmd_zero/1', 'HwiModbusWrite/4');
L('wheels_zero/1', 'HwiModbusWrite/5');
L('hw_button/1', 'PlcLatch/1');
L('HwiModbusWrite/1', 'PlcLatch/2');
L('HwiModbusWrite/2', 'PlcLatch/3');
L('cpu_wdg_ok/1', 'PlcLatch/4');
L('Clock/1', 'IoPoll/1');
L('hw_button/1', 'IoPoll/2');
L('HwiModbusWrite/1', 'IoPoll/3');
L('PlcLatch/1', 'IoPoll/4');
L('IoPoll/1', 'WriteGate/1');
L('IoPoll/2', 'WriteGate/2');

outs = {'sw_coil', 'SwEStopRequest/1'; 'latch_active', 'PlcLatch/1'; ...
        'contactor_engaged', 'PlcLatch/2'; 'e_stop_active', 'WriteGate/1'; ...
        'wheel_cmd_zero', 'WriteGate/2'; 'pub_hw_button', 'IoPoll/3'; ...
        'pub_latch', 'IoPoll/4'; 'pub_sw_e_stop', 'IoPoll/5'};
for k = 1:size(outs, 1)
    y = 40 + (k - 1) * 40;
    add_block('simulink/Sinks/Out1', [name '/' outs{k, 1}], 'Position', [1640 y 1670 y + 14]);
    h = L(outs{k, 2}, [outs{k, 1} '/1']);
    set_param(h, 'Name', outs{k, 1});
end

set_param(name, 'Description', ['As-built E-stop chain (rover_hardware_interface + safety PLC). ' ...
    'Generated by scripts/build_estop_latch.m.']);
save_system(name, file);
close_system(name, 0);
fprintf('build_estop_latch: %s\n', file);
end
