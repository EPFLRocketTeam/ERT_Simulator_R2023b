classdef massNonLinTest < matlab.unittest.TestCase
    % Unit tests for massNonLin.m

    properties
        AbsTol = 1e-10;
    end

    methods (Test)

        %% -------------------------------------------------------------
        %   HYBRID = 0  (STANDARD MOTOR)
        %% -------------------------------------------------------------

        function testNonLinear_nonHybrid_midBurn(testCase)
            R = rocketNonHybrid();
            t = R.Burn_Time/2;

            [M,dMdt] = massNonLin(t,R);

            tt = linspace(0,t,500);
            Current_Impulse = trapz(tt,Thrust(tt,R));
            exp_M = R.rocket_m + R.motor_mass - R.Thrust2dMass_Ratio*Current_Impulse;
            exp_dMdt = R.Thrust2dMass_Ratio*Thrust(t,R);

            testCase.verifyEqual(dMdt,exp_dMdt,'AbsTol',testCase.AbsTol);
            testCase.verifyEqual(M,exp_M,'AbsTol',testCase.AbsTol);
        end


        function testNonLinear_nonHybrid_afterBurn(testCase)
            R = rocketNonHybrid();
            t = R.Burn_Time + 1;

            [M,dMdt] = massNonLin(t,R);

            exp_M = R.rocket_m + R.motor_mass - R.propel_mass;

            testCase.verifyEqual(M,exp_M,'AbsTol',testCase.AbsTol);
            testCase.verifyEqual(dMdt,0,'AbsTol',testCase.AbsTol);
        end


        %% -------------------------------------------------------------
        %   HYBRID = 1  (HYBRID MOTOR)
        %% -------------------------------------------------------------

        function testNonLinear_hybrid_afterBurn(testCase)
            R = rocketHybrid();
            t = R.Burn_Time + 1;

            [M,dMdt] = massNonLin(t,R,'NonLinear');

            exp_M = R.rocket_m + R.casing_mass;

            testCase.verifyEqual(M,exp_M,'AbsTol',testCase.AbsTol);
            testCase.verifyEqual(dMdt,0,'AbsTol',testCase.AbsTol);
        end

        %% -------------------------------------------------------------
        %   GENERAL SANITY CHECKS
        %% -------------------------------------------------------------


        function testMass(testCase)
            R = rocketNonHybrid();
            t = linspace(0, R.Burn_Time, 1000);
            [M,dMdt,~,~,~,~,~,~] = massNonLin(t,R,'NonLinear');
            testCase.verifyGreaterThanOrEqual(M,0);
            testCase.verifyLessThanOrEqual(dMdt,0);
            testCase.verifyFinite(M)
        end

    end
end

%% ========================================================================
%   DUMMY ROCKET GENERATORS
%% ========================================================================

function R = rocketNonHybrid()
    R.isHybrid = 0;
    R.propel_mass = 10;
    R.Burn_Time = 5;
    R.rocket_m = 50;
    R.motor_mass = 20;
    R.casing_mass = 5;
    R.rocket_cm = 1.0;
    R.motor_length = 0.4;
    R.motor_dia = 0.12;
    R.L = 2.5;
    R.rocket_I = 0.3;
    R.Thrust2dMass_Ratio = 0.02;
end

function R = rocketHybrid()
    R.isHybrid = 1;
    R.propel_mass = 8;
    R.Burn_Time = 6;
    R.rocket_m = 55;
    R.casing_mass = 4;
    R.motor_mass = 18;
    R.rocket_cm = 1.0;
    R.motor_length = 0.45;
    R.motor_lengthP = 0.25;
    R.motor_lengthF = 0.15;
    R.motor_massP = 10;
    R.motor_massF = 8;
    R.propel_massP = 4;
    R.propel_massF = 4;
    R.intermotor_d = 0.02;
    R.motor_dia = 0.11;
    R.L = 2.6;
    R.rocket_I = 0.25;
    R.Thrust2dMass_Ratio = 0.02;
end

%% ========================================================================
%   DUMMY THRUST FUNCTION  (used for NonLinear tests)
%% ========================================================================
function y = Thrust(t, Rocket)
    % Deterministic thrust curve:  simply T = 1000*N*sin(t)
    if t<Rocket.Burn_Time
        y = 1000*sin(pi*t/Rocket.Burn_Time);
    else
        y=0;
    end
end