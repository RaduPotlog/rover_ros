classdef tSkidSteerKinematics < matlab.unittest.TestCase
    %TSKIDSTEERKINEMATICS Verifies behaviour/SkidSteerKinematics.slx (as built) and
    %   the drive rate parameters. The model assumes ideal wheel tracking and no slip,
    %   so it verifies the kinematic design; SYS-SR-003's field test still applies.

    properties (Constant)
        Model = 'SkidSteerKinematics'
        Dt = 0.02
    end

    methods (TestClassSetup)
        function loadModel(tc)
            setup_project();
            load_system(tc.Model);
            tc.addTeardown(@() close_system(tc.Model, 0));
        end
    end

    methods (Test)
        function testStraightOdometryError(tc)
            % Verifies: SYS-SR-003, SWR-CTL-005, SWR-CTL-006
            % 10 m straight at 0.5 m/s: odometry distance error <= 5 %.
            v = 0.5;
            t = (0:tc.Dt:10 / v - tc.Dt)';
            y = mbse.simModel(tc.Model, t, [v * ones(numel(t), 1), zeros(numel(t), 1)]);
            truth = v * numel(t) * tc.Dt;
            tc.verifyLessThanOrEqual(abs(y.distance(end) - truth) / truth, 0.05);
            tc.verifyEqual(y.y(end), 0, 'AbsTol', 1e-9);
        end

        function testRotationRoundTrip(tc)
            % Verifies: SYS-SR-003, SWR-CTL-005
            t = (0:tc.Dt:2)';
            y = mbse.simModel(tc.Model, t, [0.3 * ones(numel(t), 1), 0.8 * ones(numel(t), 1)]);
            tc.verifyEqual(y.v_odom(end), 0.3, 'AbsTol', 1e-12);
            tc.verifyEqual(y.wz_odom(end), 0.8, 'AbsTol', 1e-12);
        end

        function testRimSpeedWithinJointLimit(tc)
            % Verifies: SYS-SR-004, SWR-CTL-011
            % Full linear plus full angular command must stay under the URDF joint limit.
            t = (0:tc.Dt:0.2)';
            vx = mbse.param('max_linear_velocity');
            wz = mbse.param('max_angular_velocity');
            y = mbse.simModel(tc.Model, t, [vx * ones(numel(t), 1), wz * ones(numel(t), 1)]);
            tc.verifyEqual(max(y.wheel_velocity_limited), 0);
            tc.verifyLessThanOrEqual(max(y.rim_speed_max), ...
                mbse.param('wheel_joint_velocity_limit') * mbse.param('wheel_radius'));
        end

        function testControlRates(tc)
            % Verifies: SYS-SR-002, SWR-CTL-001, SWR-CTL-016
            cm = mbse.param('cm_update_rate');
            dd = mbse.param('diff_drive_update_rate');
            tc.verifyGreaterThanOrEqual(cm, 50);
            tc.verifyGreaterThanOrEqual(dd, 50);
            tc.verifyEqual(mod(cm, dd), 0, 'diff_drive rate does not divide the controller_manager rate');
        end
    end

    methods (Test, TestTags = {'KnownDeviation'})
        function testEncoderFeedbackRate(tc)
            % Verifies: SYS-SR-002
            % The DCC1000 encoder state refreshes at driver_states_update_frequency
            % (rover_controller/README.md:57-58), below the 50 Hz control rate.
            tc.verifyGreaterThanOrEqual(mbse.param('driver_states_update_frequency'), 50);
        end

        function testImuPublishRate(tc)
            % Verifies: SYS-SR-019, SWR-CTL-015
            tc.verifyGreaterThanOrEqual(mbse.param('imu_broadcaster_rate'), 100);
        end
    end
end
