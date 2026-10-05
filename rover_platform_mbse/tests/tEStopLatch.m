classdef tEStopLatch < matlab.unittest.TestCase
    %TESTOPLATCH Verifies behaviour/EStopLatch.slx (as built).
    %   Inputs: 1 hw_button, 2 sw_set, 3 sw_reset, 4 latch_reset, 5 cmd_zero,
    %   6 wheels_zero, 7 cpu_wdg_ok. Every run first clears the start-up latch with a
    %   latch reset at 0.2 s. Tests tagged KnownDeviation document a requirement the
    %   as-built design misses.

    properties (Constant)
        Model = 'EStopLatch'
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
        function testStartupLatched(tc)
            % Verifies: SWR-HWI-011, SYS-SR-008
            [t, U] = tc.base(1);
            U(:, 4) = 0;   % no latch reset
            y = mbse.simModel(tc.Model, t, U);
            tc.verifyEqual(min(y.latch_active), 1);
            tc.verifyEqual(max(y.contactor_engaged), 0);
        end

        function testHwButtonRemovesDrivePower(tc)
            % Verifies: SYS-SR-006
            % The PLC opens the contactor in the same scan, without any software step.
            [t, U] = tc.base(2);
            U(t >= 1, 1) = 1;
            y = mbse.simModel(tc.Model, t, U);
            k = find(y.t >= 1, 1);
            tc.verifyEqual(y.contactor_engaged(k - 1), 1, 'drive power before the press');
            tc.verifyEqual(y.contactor_engaged(k), 0, 'drive power not removed at the press');
        end

        function testLatchHoldsAfterButtonRelease(tc)
            % Verifies: SYS-SR-008, SWR-HWI-009
            [t, U] = tc.base(4);
            U(t >= 1 & t < 1.3, 1) = 1;   % press and release
            U(t >= 1, 5) = 0;             % commands still non-zero
            U(t >= 3 & t < 3.05, 4) = 1;  % explicit operator reset
            y = mbse.simModel(tc.Model, t, U);
            held = y.t >= 1.3 & y.t < 3;
            tc.verifyEqual(min(y.latch_active(held)), 1, 'latch released without a reset');
            tc.verifyEqual(min(y.wheel_cmd_zero(held & y.t >= 1.4)), 1);
            tc.verifyEqual(y.latch_active(end), 0, 'explicit reset did not clear the latch');
        end

        function testSetDominantLatch(tc)
            % Verifies: SYS-SR-008
            [t, U] = tc.base(3);
            U(t >= 1, 1) = 1;              % button held
            U(t >= 2 & t < 2.05, 4) = 1;   % reset attempt while held
            y = mbse.simModel(tc.Model, t, U);
            tc.verifyEqual(min(y.latch_active(y.t >= 1)), 1);
        end

        function testSwEStopNeedsLatchReset(tc)
            % Verifies: SYS-SR-008, SWR-HWI-006, SWR-HWI-008
            [t, U] = tc.base(5);
            U(t >= 1 & t < 1.05, 2) = 1;   % SW E-stop set
            U(t >= 2 & t < 2.05, 3) = 1;   % SW E-stop reset at standstill
            U(t >= 4 & t < 4.05, 4) = 1;   % latch reset
            y = mbse.simModel(tc.Model, t, U);
            tc.verifyEqual(y.sw_coil(find(y.t >= 2.1, 1)), 0, 'SW E-stop not released at standstill');
            tc.verifyEqual(min(y.latch_active(y.t >= 1.05 & y.t < 4)), 1, 'latch cleared by the SW reset alone');
            tc.verifyEqual(y.wheel_cmd_zero(end), 0);
        end

        function testSwResetRefusedWhileMoving(tc)
            % Verifies: SWR-HWI-008
            [t, U] = tc.base(3);
            U(t >= 1 & t < 1.05, 2) = 1;
            U(:, 6) = double(t < 0.5 | t >= 2.5);   % wheels turning between 0.5 s and 2.5 s
            U(t >= 2 & t < 2.05, 3) = 1;            % reset request while turning
            y = mbse.simModel(tc.Model, t, U);
            tc.verifyEqual(y.sw_coil(end), 1);
        end

        function testCpuWatchdogLossLatches(tc)
            % Verifies: SWR-HWI-012, SYS-SR-006
            [t, U] = tc.base(2);
            U(t >= 1, 7) = 0;
            y = mbse.simModel(tc.Model, t, U);
            tc.verifyEqual(y.latch_active(find(y.t >= 1, 1)), 1);
            tc.verifyEqual(y.contactor_engaged(end), 0);
        end

        function testSwEStopZeroCommandIdealLink(tc)
            % Verifies: SYS-SR-007, SWR-HWI-005, SWR-HWI-007
            % Ideal Modbus link: the only delay is the 10 Hz IO poll that refreshes the
            % cache write() reads (rover_safety_controller.cpp:486-492).
            lat = tc.swEStopLatencies(0);
            tc.verifyLessThanOrEqual(max(lat), 0.1 + 1e-9);
        end
    end

    methods (Test, TestTags = {'KnownDeviation'})
        function testSwEStopZeroCommandWorstCaseLink(tc)
            % Verifies: SYS-SR-007, SWR-HWI-007
            % Worst case: the coil write waits behind one in-flight Modbus transaction
            % (modbus_response_timeout) before the poll can see it.
            lat = tc.swEStopLatencies(mbse.param('modbus_response_timeout'));
            tc.verifyLessThanOrEqual(max(lat), 0.1 + 1e-9, ...
                sprintf('worst-case SW E-stop to zero command = %.0f ms', 1000 * max(lat)));
        end

        function testEStopStatePublishLatency(tc)
            % Verifies: SYS-SR-023, SWR-HWI-016
            % HW button change -> IO poll (10 Hz) -> safety_status publish (20 Hz). The
            % two timers are not phase-locked, so the publish offset is swept too.
            P = mbse.modelParam(tc.Model, 'P');
            lat = [];
            for phase = 0:0.01:0.04
                P.publish_phase = phase;
                for p = 1:10
                    press = 1 + (p - 1) * 0.01;
                    [t, U] = tc.base(1.5);
                    U(t >= press - 1e-9, 1) = 1;
                    y = mbse.simModel(tc.Model, t, U, struct('P', P));
                    lat(end + 1) = y.t(find(y.pub_hw_button > 0.5, 1)) - press; %#ok<AGROW>
                end
            end
            tc.verifyLessThanOrEqual(max(lat), 0.1 + 1e-9, ...
                sprintf('worst-case E-stop state publish latency = %.0f ms', 1000 * max(lat)));
        end

        function testLatchResetRefusedWhileMoving(tc)
            % Verifies: SWR-HWI-010
            % SAFETY_CHAIN.md section 5 says the zero-velocity invariant covers the latch;
            % resetEStopLatch() has no such check (emergency_stop.cpp:239-249).
            [t, U] = tc.base(4);
            U(t >= 1 & t < 1.3, 1) = 1;
            U(t >= 1, 5) = 0;              % non-zero commands
            U(t >= 1, 6) = 0;              % wheels turning
            U(t >= 2 & t < 2.05, 4) = 1;   % latch reset while moving
            y = mbse.simModel(tc.Model, t, U);
            tc.verifyEqual(y.latch_active(end), 1, 'latch reset accepted while moving');
        end
    end

    methods (Access = private)
        function [t, U] = base(tc, stopTime)
            t = (0:tc.Dt:stopTime)';
            U = zeros(numel(t), 7);
            U(:, 5) = 1;                       % zero command
            U(:, 6) = 1;                       % wheels at standstill
            U(:, 7) = 1;                       % CPU watchdog healthy
            U(t >= 0.2 & t < 0.25, 4) = 1;     % clear the start-up latch
        end

        function lat = swEStopLatencies(tc, linkDelay)
            % SW E-stop set at 10 phases across one IO poll period.
            P = mbse.modelParam(tc.Model, 'P');
            P.link_delay = linkDelay;
            lat = zeros(1, 10);
            for p = 1:10
                tSet = 1 + (p - 1) * 0.01;
                [t, U] = tc.base(2);
                U(t >= tSet - 1e-9 & t < tSet + 0.05, 2) = 1;
                y = mbse.simModel(tc.Model, t, U, struct('P', P));
                lat(p) = y.t(find(y.t >= tSet - 1e-9 & y.wheel_cmd_zero > 0.5, 1)) - tSet;
            end
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
