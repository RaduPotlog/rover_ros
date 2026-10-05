classdef tCommandArbitration < matlab.unittest.TestCase
    %TCOMMANDARBITRATION Verifies behaviour/CommandArbitration.slx (as built).
    %   Every test method names the requirements it verifies on a "% Verifies:" line;
    %   scripts/build_trace_links.m turns these into Requirements Toolbox verify links.
    %   Tests tagged KnownDeviation document a requirement the as-built design misses.

    properties (Constant)
        Model = 'CommandArbitration'
        Dt = 0.01
    end

    methods (TestClassSetup)
        function loadModel(tc)
            setup_project();
            load_system(tc.Model);
            tc.addTeardown(@() close_system(tc.Model, 0));
        end
    end

    methods (Test)
        function testPriorityOrder(tc)
            % Verifies: SYS-SR-005, SWR-MUX-002, SWR-MUX-003
            % All four sources publish; each phase silences the current winner.
            [t, U] = tc.healthy(8);
            values = [0.1 0.2 0.3 0.4];
            stopAt = [2 4 6 inf];
            for s = 1:4
                U = tc.stream(t, U, s, values(s), 0, 20, 0, stopAt(s));
            end
            y = mbse.simModel(tc.Model, t, U);
            checkAt = [1.9 3.9 5.9 7.9];   % end of each phase
            for s = 1:4
                k = find(y.t >= checkAt(s), 1);
                tc.verifyEqual(y.active_source(k), s, sprintf('phase %d winner', s));
                tc.verifyEqual(y.cmd_vx(k), values(s), 'AbsTol', 1e-9, sprintf('phase %d command', s));
            end
        end

        function testSpeedLimitAllSources(tc)
            % Verifies: SYS-SR-004, SWR-CTL-008
            limV = min(1.0, mbse.param('max_linear_velocity'));
            for s = 1:4
                [t, U] = tc.healthy(3);
                U = tc.stream(t, U, s, 2.0, 3.0, 20, 0, inf);
                y = mbse.simModel(tc.Model, t, U);
                tc.verifyLessThanOrEqual(max(abs(y.cmd_vx)), 1.0, sprintf('source %d linear', s));
                tc.verifyLessThanOrEqual(max(abs(y.cmd_wz)), 1.7, sprintf('source %d angular', s));
                tc.verifyEqual(max(abs(y.cmd_vx)), limV, 'AbsTol', 1e-9);
            end
        end

        function testAccelerationLimit(tc)
            % Verifies: SYS-SR-013, SWR-CTL-009
            [t, U] = tc.healthy(3);
            U = tc.stream(t, U, 1, 0.9, 1.4, 50, 0.5, inf);
            y = mbse.simModel(tc.Model, t, U);
            dt = 1 / mbse.param('diff_drive_update_rate');
            tc.verifyLessThanOrEqual(max(abs(diff(y.cmd_vx))), mbse.param('max_linear_acceleration') * dt + 1e-9);
            tc.verifyLessThanOrEqual(max(abs(diff(y.cmd_wz))), mbse.param('max_angular_acceleration') * dt + 1e-9);
        end

        function testMotionLockMasksAllSources(tc)
            % Verifies: SWR-MUX-004, SWR-MUX-010, SYS-SR-006
            [t, U] = tc.healthy(4);
            for s = 1:4
                U = tc.stream(t, U, s, 0.1 * s, 0, 20, 0, inf);
            end
            U(t >= 2, 14) = 1;   % hardware E-stop button in SafetyStatus
            y = mbse.simModel(tc.Model, t, U);
            after = y.t >= 2 + 1 / mbse.param('motion_lock_rate') + 0.02;
            tc.verifyEqual(max(y.mux_pub(after)), 0, 'mux still forwards a source while locked');
            tc.verifyEqual(max(y.active_source(after)), 0);
            tc.verifyEqual(y.cmd_vx(end), 0, 'AbsTol', 1e-9);
        end

        function testStaleSafetyLocks(tc)
            % Verifies: SWR-MUX-008, SYS-SR-012
            [t, U] = tc.healthy(5);
            U(t >= 2, 13) = 0;   % safety_status stops arriving
            U = tc.stream(t, U, 1, 0.3, 0, 20, 0, inf);
            y = mbse.simModel(tc.Model, t, U);
            lockAfter = 2 + mbse.param('motion_lock_gpio_timeout') + 1 / mbse.param('motion_lock_rate') + 0.05;
            tc.verifyEqual(y.motion_lock(find(y.t >= lockAfter, 1)), 1);
            tc.verifyEqual(max(y.mux_pub(y.t >= lockAfter + 0.05)), 0);
        end

        function testLockedAtStartup(tc)
            % Verifies: SWR-MUX-007
            t = (0:tc.Dt:2)';
            U = zeros(numel(t), 18);
            U(:, 18) = 1;
            U = tc.stream(t, U, 1, 0.3, 0, 20, 0, inf);   % commands but no safety_status yet
            y = mbse.simModel(tc.Model, t, U);
            tc.verifyEqual(max(y.mux_pub), 0);
            tc.verifyEqual(max(abs(y.cmd_vx)), 0);
        end
    end

    methods (Test, TestTags = {'KnownDeviation'})
        function testCommandWatchdog(tc)
            % Verifies: SYS-SR-011, SWR-CTL-010, SWR-MUX-003
            % Stream at 50 Hz, stop at 2 s; deceleration must start within 500 ms of the
            % last message. As built: cmd_vel_timeout 0.5 s checked with '>' at 50 Hz, so
            % the first reduced output comes up to one controller period (20 ms) later:
            % a marginal miss of SYS-SR-011.
            [t, U] = tc.healthy(4);
            U = tc.stream(t, U, 1, 0.5, 0, 50, 0, 2.0);
            y = mbse.simModel(tc.Model, t, U);
            lastMsg = max(t(U(:, 3) > 0.5));
            kDecel = find(y.t > lastMsg & y.cmd_vx < 0.5 - 1e-9, 1);
            tc.verifyNotEmpty(kDecel);
            tc.verifyLessThanOrEqual(y.t(kDecel) - lastMsg, 0.5 + 1e-9, ...
                'deceleration starts later than 500 ms after the last command');
        end
    end

    methods (Access = private)
        function [t, U] = healthy(tc, stopTime)
            % Healthy safety_status at 20 Hz, link healthy, no stop condition.
            t = (0:tc.Dt:stopTime)';
            U = zeros(numel(t), 18);
            U(:, 13) = mod(round(t / tc.Dt), 5) == 0;
            U(:, 18) = 1;
        end

        function U = stream(tc, t, U, source, vx, wz, rateHz, tStart, tStop)
            n = round(1 / (rateHz * tc.Dt));
            on = t >= tStart - 1e-9 & t < tStop - 1e-9 & mod(round(t / tc.Dt), n) == 0;
            c = 3 * source - 2;
            U(:, c) = vx;
            U(:, c + 1) = wz;
            U(:, c + 2) = on;
        end
    end
end

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
