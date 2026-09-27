function build_command_arbitration()
%BUILD_COMMAND_ARBITRATION Executable as-built model of velocity command arbitration.
%   behaviour/CommandArbitration.slx, 100 Hz (controller_manager rate):
%     MotionLock  - rover_motion_lock_node: 10 Hz Bool, fail-safe locked when no
%                   safety_status yet, stale (> gpio_timeout) or link unhealthy.
%     TwistMux    - upstream twist_mux semantics (/opt/ros/lyrical/include/twist_mux/
%                   topic_handle.hpp): a source is masked when expired or below the
%                   lock priority; a stale lock counts as locked; the mux publishes
%                   only when the winning source's message arrives.
%     DiffDrive   - diff_drive_controller at 50 Hz: holds the last cmd_vel, zeroes it
%                   after cmd_vel_timeout, clamps velocity and acceleration.
%   Sources in port order: 1 ELRS, 2 Foxglove joystick, 3 Driver UI, 4 Nav 2.
%   Each source has <vx, wz, msg>, where msg = 1 in the step a message arrives.
%   All parameters come from data/platform_parameters.json.

name = 'CommandArbitration';
file = mbse.newModel(name, 0.01);

P = mbse.paramStruct({'prio_elrs', 'prio_joystick', 'prio_driver_interface', 'prio_nav', ...
    'timeout_elrs', 'timeout_joystick', 'timeout_driver_interface', 'timeout_nav', ...
    'prio_motion_lock', 'timeout_motion_lock', 'motion_lock_rate', 'motion_lock_gpio_timeout', ...
    'cmd_vel_timeout', 'diff_drive_update_rate', 'max_linear_velocity', ...
    'max_linear_acceleration', 'max_angular_velocity', 'max_angular_acceleration'});
P.dt = 0.01;
assignin(get_param(name, 'ModelWorkspace'), 'P', P);

add_block('simulink/Sources/Digital Clock', [name '/Clock'], 'SampleTime', '0.01', ...
    'Position', [60 20 110 50]);

lockCode = strjoin({
    'function [lock_msg, lock_value] = MotionLock(t, safety_msg, hw_button, sw_button, driver_fault, latch, link_healthy, P)'
    '% rover_motion_lock_node: publishes motion_lock at motion_lock_rate.'
    'persistent last_rx inhibit next_pub'
    'if isempty(last_rx), last_rx = -inf; inhibit = true; next_pub = 0; end'
    'if safety_msg > 0.5'
    '    last_rx = t;'
    '    inhibit = hw_button > 0.5 || sw_button > 0.5 || driver_fault > 0.5 || latch > 0.5 || link_healthy < 0.5;'
    'end'
    'stale = (t - last_rx) > P.motion_lock_gpio_timeout;'
    'lock_value = double(inhibit || stale);'
    'lock_msg = 0;'
    'if t >= next_pub - 1e-9'
    '    lock_msg = 1;'
    '    next_pub = t + 1 / P.motion_lock_rate;'
    'end'
    }, newline);
mbse.addFcn([name '/MotionLock'], lockCode, {'P'}, [400 300 560 440]);

muxCode = strjoin({
    'function [pub, vx, wz, active] = TwistMux(t, u, lock_msg, lock_value, P)'
    '% u = [vx wz msg] x 4 sources (ELRS, joystick, driver UI, nav).'
    'persistent stamp lock_stamp lock_data'
    'if isempty(stamp), stamp = -inf(1, 4); lock_stamp = -inf; lock_data = 0; end'
    'prio = [P.prio_elrs P.prio_joystick P.prio_driver_interface P.prio_nav];'
    'tout = [P.timeout_elrs P.timeout_joystick P.timeout_driver_interface P.timeout_nav];'
    'if lock_msg > 0.5, lock_stamp = t; lock_data = lock_value; end'
    'locked = (t - lock_stamp) > P.timeout_motion_lock || lock_data > 0.5;'
    'lock_prio = 0;'
    'if locked, lock_prio = P.prio_motion_lock; end'
    'arrived = false(1, 4);'
    'for k = 1:4'
    '    if u(3 * k) > 0.5, stamp(k) = t; arrived(k) = true; end'
    'end'
    'masked = ((t - stamp) > tout) | (prio < lock_prio);'
    'best = 0; bestPrio = -1;'
    'for k = 1:4'
    '    if ~masked(k) && prio(k) > bestPrio, best = k; bestPrio = prio(k); end'
    'end'
    'pub = 0; vx = 0; wz = 0; active = best;'
    'if best > 0 && arrived(best)'
    '    pub = 1; vx = u(3 * best - 2); wz = u(3 * best - 1);'
    'end'
    }, newline);
mbse.addFcn([name '/TwistMux'], muxCode, {'P'}, [700 120 860 260]);

ddCode = strjoin({
    'function [vx_out, wz_out] = DiffDrive(t, pub, vx, wz, P)'
    '% diff_drive_controller: runs at diff_drive_update_rate, speed limiter per axis.'
    'persistent last_vx last_wz last_stamp out_vx out_wz next_update'
    'if isempty(last_vx), last_vx = 0; last_wz = 0; last_stamp = -inf; out_vx = 0; out_wz = 0; next_update = 0; end'
    'if pub > 0.5, last_vx = vx; last_wz = wz; last_stamp = t; end'
    'if t >= next_update - 1e-9'
    '    dt = 1 / P.diff_drive_update_rate;'
    '    next_update = t + dt;'
    '    tv = last_vx; tw = last_wz;'
    '    if (t - last_stamp) > P.cmd_vel_timeout, tv = 0; tw = 0; end'
    '    tv = min(max(tv, -P.max_linear_velocity), P.max_linear_velocity);'
    '    tw = min(max(tw, -P.max_angular_velocity), P.max_angular_velocity);'
    '    dv = P.max_linear_acceleration * dt; dw = P.max_angular_acceleration * dt;'
    '    out_vx = out_vx + min(max(tv - out_vx, -dv), dv);'
    '    out_wz = out_wz + min(max(tw - out_wz, -dw), dw);'
    'end'
    'vx_out = out_vx; wz_out = out_wz;'
    }, newline);
mbse.addFcn([name '/DiffDrive'], ddCode, {'P'}, [1000 120 1160 260]);

% Source inputs are muxed into one vector u for the TwistMux block
srcNames = {'elrs', 'joystick', 'driver_ui', 'nav'};
inNames = {};
for s = 1:4
    inNames = [inNames, {[srcNames{s} '_vx'], [srcNames{s} '_wz'], [srcNames{s} '_msg']}]; %#ok<AGROW>
end
add_block('simulink/Signal Routing/Mux', [name '/SourceMux'], 'Inputs', '12', ...
    'Position', [560 40 570 400]);
for k = 1:12
    y = 40 + (k - 1) * 28;
    add_block('simulink/Sources/In1', [name '/' inNames{k}], 'Position', [300 y 330 y + 14], ...
        'Interpolate', 'off');
    add_line(name, [inNames{k} '/1'], ['SourceMux/' num2str(k)]);
end
safetyNames = {'safety_msg', 'hw_button', 'sw_button', 'driver_fault', 'latch', 'link_healthy'};
for k = 1:numel(safetyNames)
    y = 420 + (k - 1) * 28;
    add_block('simulink/Sources/In1', [name '/' safetyNames{k}], 'Position', [300 y 330 y + 14], ...
        'Interpolate', 'off');
    add_line(name, [safetyNames{k} '/1'], ['MotionLock/' num2str(k + 1)]);
end
add_line(name, 'Clock/1', 'MotionLock/1', 'autorouting', 'on');
add_line(name, 'Clock/1', 'TwistMux/1', 'autorouting', 'on');
add_line(name, 'Clock/1', 'DiffDrive/1', 'autorouting', 'on');
add_line(name, 'SourceMux/1', 'TwistMux/2', 'autorouting', 'on');
add_line(name, 'MotionLock/1', 'TwistMux/3', 'autorouting', 'on');
add_line(name, 'MotionLock/2', 'TwistMux/4', 'autorouting', 'on');
for k = 1:3
    add_line(name, ['TwistMux/' num2str(k)], ['DiffDrive/' num2str(k + 1)], 'autorouting', 'on');
end

outs = {'cmd_vx', 'DiffDrive/1'; 'cmd_wz', 'DiffDrive/2'; 'mux_pub', 'TwistMux/1'; ...
        'mux_vx', 'TwistMux/2'; 'mux_wz', 'TwistMux/3'; 'active_source', 'TwistMux/4'; ...
        'motion_lock', 'MotionLock/2'};
for k = 1:size(outs, 1)
    y = 60 + (k - 1) * 40;
    add_block('simulink/Sinks/Out1', [name '/' outs{k, 1}], 'Position', [1300 y 1330 y + 14]);
    h = add_line(name, outs{k, 2}, [outs{k, 1} '/1'], 'autorouting', 'on');
    set_param(h, 'Name', outs{k, 1});   % Dataset element name in out.yout
end

set_param(name, 'Description', ['As-built command arbitration (rover_twist_mux + ' ...
    'rover_controller diff_drive limits). Generated by scripts/build_command_arbitration.m.']);
save_system(name, file);
close_system(name, 0);
fprintf('build_command_arbitration: %s\n', file);
end
