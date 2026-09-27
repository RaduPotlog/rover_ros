classdef tPlatformModeManager < matlab.unittest.TestCase
    %TPLATFORMMODEMANAGER Verifies the PROPOSED behaviour/PlatformModeManager.slx.
    %   Inputs: 1 ready, 2 e_stop, 3 fault, 4 reset_request, 5 active_source, 6 soc,
    %   7 charging. Modes: 0 Boot, 1 Idle, 2 Teleoperation, 3 Autonomous,
    %   4 EmergencyStop, 5 Fault, 6 LowBattery. The design is not implemented in
    %   rover_ros yet (SWR-SAF-018..022 are Proposed): these tests check the proposal.

    properties (Constant)
        Model = 'PlatformModeManager'
        Dt = 0.1
        % Proposed state -> rover_msgs/LedAnimation mapping (255 = none defined yet)
        LedOf = containers.Map({0, 1, 2, 3, 4, 5, 6}, {255, 1, 4, 12, 0, 2, 5})
    end

    methods (TestClassSetup)
        function loadModel(tc)
            setup_project();
            load_system(tc.Model);
            tc.addTeardown(@() close_system(tc.Model, 0));
        end
    end

    methods (Test)
        function testEveryStateReachable(tc)
            % Verifies: SYS-SR-010, SWR-SAF-018, SWR-SAF-019
            [t, U] = tc.tour();
            y = mbse.simModel(tc.Model, t, U);
            tc.verifyEqual(unique(y.mode)', 0:6);
        end

        function testLedMapping(tc)
            % Verifies: SYS-SR-009, SWR-SAF-022, SWR-LED-010
            [t, U] = tc.tour();
            y = mbse.simModel(tc.Model, t, U);
            for m = 1:6
                led = unique(y.led_animation(y.mode == m & y.safe_stop < 0.5));
                tc.verifyEqual(led, tc.LedOf(m), sprintf('LED for mode %d', m));
            end
            tc.verifyEqual(unique(y.led_animation(y.safe_stop > 0.5)), 6, 'CRITICAL_BATTERY in SafeStop');
        end

        function testTeleopAutonomousFollowActiveSource(tc)
            % Verifies: SYS-SR-010, SWR-SAF-019, SWR-MUX-020
            t = (0:tc.Dt:6)';
            U = tc.nominal(t);
            U(t >= 1 & t < 2, 5) = 1;     % ELRS
            U(t >= 2 & t < 3, 5) = 4;     % nav
            U(t >= 3 & t < 4, 5) = 3;     % driver UI preempts nav
            y = mbse.simModel(tc.Model, t, U);
            at = @(s) y.mode(find(y.t >= s, 1));
            tc.verifyEqual([at(0.9) at(1.9) at(2.9) at(3.9) at(5.9)], [1 2 3 2 1]);
        end

        function testEStopHasPriorityAndNeedsClear(tc)
            % Verifies: SYS-SR-008, SYS-SR-010, SWR-SAF-019
            t = (0:tc.Dt:6)';
            U = tc.nominal(t);
            U(t >= 1, 5) = 1;
            U(t >= 2 & t < 4, 2) = 1;      % E-stop (latch) active 2..4 s
            U(t >= 2 & t < 3, 3) = 1;      % and a fault at the same time
            y = mbse.simModel(tc.Model, t, U);
            during = y.t >= 2.1 & y.t < 4;
            tc.verifyEqual(unique(y.mode(during)), 4);
            tc.verifyEqual(max(y.motion_allowed(during)), 0);
            tc.verifyEqual(y.mode(end), 2, 'back to Teleoperation after the reset');
        end

        function testFaultNeedsReset(tc)
            % Verifies: SWR-SAF-021, SYS-SR-010
            t = (0:tc.Dt:6)';
            U = tc.nominal(t);
            U(t >= 1 & t < 2, 3) = 1;
            U(t >= 4 & t < 4.2, 4) = 1;
            y = mbse.simModel(tc.Model, t, U);
            tc.verifyEqual(unique(y.mode(y.t >= 1.1 & y.t < 4)), 5, 'left Fault without a reset');
            tc.verifyEqual(y.mode(end), 1);
        end

        function testLowBatteryWarningAndSafeStop(tc)
            % Verifies: SYS-SR-017, SWR-SAF-020
            low = mbse.param('led_battery_low');
            crit = mbse.param('led_battery_critical');
            t = (0:tc.Dt:8)';
            U = tc.nominal(t);
            U(t >= 2, 6) = low - 0.05;
            U(t >= 4, 6) = crit - 0.05;
            U(t >= 6, 7) = 1;              % charger connected
            y = mbse.simModel(tc.Model, t, U);
            at = @(s) find(y.t >= s, 1);
            tc.verifyEqual([y.mode(at(3)) y.motion_allowed(at(3))], [6 1], 'warning allows motion');
            tc.verifyEqual([y.mode(at(5)) y.motion_allowed(at(5)) y.safe_stop(at(5))], [6 0 1], 'safe stop');
            tc.verifyEqual(y.mode(end), 1, 'charging returns to Idle');
        end

        function testNoMotionOutsideOperationalStates(tc)
            % Verifies: SYS-SR-010, SWR-SAF-018
            [t, U] = tc.tour();
            y = mbse.simModel(tc.Model, t, U);
            blocked = ismember(y.mode, [0 4 5]);
            tc.verifyEqual(max(y.motion_allowed(blocked)), 0);
        end
    end

    methods (Test, TestTags = {'KnownDeviation'})
        function testLedDefinedForEveryState(tc)
            % Verifies: SYS-SR-009, SWR-LED-020, SWR-MSG-010
            % rover_msgs/LedAnimation has no Boot animation (Gap).
            [t, U] = tc.tour();
            y = mbse.simModel(tc.Model, t, U);
            tc.verifyTrue(all(y.led_animation ~= 255), 'a state has no LED animation defined');
        end
    end

    methods (Access = private)
        function U = nominal(~, t)
            U = zeros(numel(t), 7);
            U(:, 1) = t >= 0.5;   % bringup finished at 0.5 s
            U(:, 6) = 0.9;        % battery well charged
        end

        function [t, U] = tour(tc)
            % Boot -> Idle -> Teleop -> Autonomous -> E-stop -> Idle -> Fault -> Idle -> LowBattery
            t = (0:tc.Dt:12)';
            U = tc.nominal(t);
            U(t >= 1 & t < 2, 5) = 2;
            U(t >= 2 & t < 3, 5) = 4;
            U(t >= 3 & t < 4, 2) = 1;
            U(t >= 5 & t < 6, 3) = 1;
            U(t >= 7 & t < 7.2, 4) = 1;
            U(t >= 9, 6) = mbse.param('led_battery_low') - 0.05;
            U(t >= 11, 6) = mbse.param('led_battery_critical') - 0.05;
        end
    end
end
