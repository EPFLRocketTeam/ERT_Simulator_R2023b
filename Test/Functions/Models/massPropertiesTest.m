classdef massPropertiesTest < matlab.unittest.TestCase
    % Unit tests for massProperties.m

    properties
        AbsTol = 1e-10;
        AddedPath
    end

    methods (Test)

        function addFunctionPath(testCase)
            % This function temporarily adds the directory of the
            % massProperties function to the MATLAB path so the tests can find it

            % 1. Get the path of the current test file directory
            % (e.g., ...\ERT_Simulator_R2023b\Test\Functions\Models)
            testDir = fileparts(mfilename('fullpath'));

            % 2. Move up two directories to reach the root folder
            % (e.g., ...\ERT_Simulator_R2023b)
            rootPath = fileparts(fileparts(fileparts(testDir)));

            % 3. Construct the path to the function file's directory
            % (e.g., ...\ERT_Simulator_R2023b\Src\Simulator_3D)
            functionPath = fullfile(rootPath, 'Src', 'Functions', 'Models');

            % 4. Add the paths to MATLAB's search path
            addpath(functionPath);

            % 5. Store the paths so we can remove it later in TestClassTeardown
            testCase.AddedPath = {functionPath};
        end

        %% -------------------------------------------------------------
        %   HYBRID = 0  (STANDARD MOTOR)
        %% -------------------------------------------------------------

        function testLinear_nonHybrid_t0(testCase)
            R = rocketNonHybrid();
            t = 0;
            [M,dMdt] = massProperties(t,R,'Linear');

            exp_dMdt = R.propelMass / R.burnTime;
            exp_M = R.emptyMass;

            testCase.verifyEqual(dMdt,exp_dMdt,'AbsTol',testCase.AbsTol);
            testCase.verifyEqual(M,exp_M,'AbsTol',testCase.AbsTol);
        end

        function testLinear_nonHybrid_midBurn(testCase)
            R = rocketNonHybrid();
            t = R.burnTime/2;

            [M,dMdt] = massProperties(t,R,'Linear');

            exp_dMdt = R.propelMass / R.burnTime;
            exp_M = R.emptyMass + R.motorMass - t*exp_dMdt;

            testCase.verifyEqual(dMdt,exp_dMdt,'AbsTol',testCase.AbsTol);
            testCase.verifyEqual(M,exp_M,'AbsTol',testCase.AbsTol);
        end

        function testLinear_nonHybrid_afterBurn(testCase)
            R = rocketNonHybrid();
            t = R.burnTime + 1;

            [M,dMdt] = massProperties(t,R,'Linear');

            exp_M = R.emptyMass + R.casingMass;

            testCase.verifyEqual(M,exp_M,'AbsTol',testCase.AbsTol);
            testCase.verifyEqual(dMdt,0,'AbsTol',testCase.AbsTol);
        end

        function testNonLinear_nonHybrid_afterBurn(testCase)
            R = rocketNonHybrid();
            t = R.burnTime + 1;

            [M,dMdt] = massProperties(t,R,'NonLinear');

            exp_M = R.emptyMass + R.motorMass - R.propelMass;

            testCase.verifyEqual(M,exp_M,'AbsTol',testCase.AbsTol);
            testCase.verifyEqual(dMdt,0,'AbsTol',testCase.AbsTol);
        end


        %% -------------------------------------------------------------
        %   HYBRID = 1  (HYBRID MOTOR)
        %% -------------------------------------------------------------

        function testLinear_hybrid_t0(testCase)
            R = rocketHybrid();
            t = 0;

            [M,dMdt] = massProperties(t,R,'Linear');

            exp_dMdt = R.propelMass / R.burnTime;
            exp_M    = R.emptyMass;

            testCase.verifyEqual(M,exp_M,'AbsTol',testCase.AbsTol);
            testCase.verifyEqual(dMdt,exp_dMdt,'AbsTol',testCase.AbsTol);
        end

        function testLinear_hybrid_afterBurn(testCase)
            R = rocketHybrid();
            t = R.burnTime + 1;

            [M,dMdt] = massProperties(t,R,'Linear');

            exp_M = R.emptyMass + R.casingMass;

            testCase.verifyEqual(M,exp_M,'AbsTol',testCase.AbsTol);
            testCase.verifyEqual(dMdt,0,'AbsTol',testCase.AbsTol);
        end

        function testNonLinear_hybrid_afterBurn(testCase)
            R = rocketHybrid();
            t = R.burnTime + 1;

            [M,dMdt] = massProperties(t,R,'NonLinear');

            exp_M = R.emptyMass + R.casingMass;

            testCase.verifyEqual(M,exp_M,'AbsTol',testCase.AbsTol);
            testCase.verifyEqual(dMdt,0,'AbsTol',testCase.AbsTol);
        end

        %% -------------------------------------------------------------
        %   GENERAL SANITY CHECKS
        %% -------------------------------------------------------------

        function testInertias_t0(testCase)
            R = rocketNonHybrid();
            t = 0;

            [~,~,~,~,I_L,dI_Ldt,I_R,dI_Rdt] = massProperties(t,R,'NonLinear');

            testCase.verifyGreaterThanOrEqual(I_L,0);
            testCase.verifyGreaterThanOrEqual(I_R,0);
            testCase.verifyLessThanOrEqual(dI_Rdt,0);
            testCase.verifyLessThanOrEqual(dI_Ldt,0);
        end

        function testInertias_MidBurn(testCase)
            R = rocketNonHybrid();
            t = R.burnTime/2;

            [~,~,~,~,I_L,dI_Ldt,I_R,dI_Rdt] = massProperties(t,R,'NonLinear');

            testCase.verifyGreaterThanOrEqual(I_L,0);
            testCase.verifyGreaterThanOrEqual(I_R,0);
            testCase.verifyLessThanOrEqual(dI_Rdt,0);
            testCase.verifyLessThanOrEqual(dI_Ldt,0);
        end

        function testMass(testCase)
            R = rocketNonHybrid();
            t = R.burnTime + 1;
            [M,dMdt,~,~,~,~,~,~] = massProperties(t,R,'NonLinear');
            testCase.verifyGreaterThanOrEqual(M,0);
            testCase.verifyLessThanOrEqual(dMdt,0);
            testCase.verifyFinite(M)
        end

        function testCm_t0(testCase)
            R = rocketNonHybrid();
            t = 0;
            [~,~,Cm,dCmdt,~,~,~,~] = massProperties(t,R,'NonLinear');
            testCase.verifyGreaterThanOrEqual(Cm,0);
            testCase.verifyLessThanOrEqual(Cm,R.L);
            testCase.verifyFinite(dCmdt)
        end

        function testCm_midBurn(testCase)
            R = rocketNonHybrid();
            t = R.burnTime/2;
            [~,~,Cm,dCmdt,~,~,~,~] = massProperties(t,R,'NonLinear');
            testCase.verifyGreaterThanOrEqual(Cm,0);
            testCase.verifyLessThanOrEqual(Cm,R.L);
            testCase.verifyFinite(dCmdt)
        end

        function testCm_afterBurn(testCase)
            R = rocketNonHybrid();
            t = R.burnTime + 1;
            [~,~,Cm,dCmdt,~,~,~,~] = massProperties(t,R,'NonLinear');
            testCase.verifyGreaterThanOrEqual(Cm,0);
            testCase.verifyLessThanOrEqual(Cm,R.L);
            testCase.verifyFinite(dCmdt)
        end

    end
end

%% ========================================================================
%   DUMMY ROCKET GENERATORS
%% ========================================================================

function R = rocketNonHybrid()
    R.isHybrid = 0;
    R.propelMass = 10;
    R.burnTime = 5;
    R.emptyMass = 50;
    R.motorMass = 20;
    R.casingMass = 5;
    R.emptyCenterOfMass = 1.0;
    R.motorLength = 0.4;
    R.motorDiameter = 0.12;
    R.length= 2.5;
    R.emptyInertia = 0.3;
    R.thrust2dMassRatio = 0.02;
    R.thrustTime = [0 1.5 1.6 4.5 5];
    R.thrustForce = [0 2500 2500 2400 1200];

end

function R = rocketHybrid()
    R.isHybrid = 1;
    R.propelMass = 8;
    R.burnTime = 6;
    R.emptyMass = 55;
    R.casingMass = 4;
    R.motorMass = 18;
    R.emptyCenterOfMass = 1.0;
    R.motorLength = 0.45;
    R.motorLengthPropel = 0.25;
    R.motorLengthFuel = 0.15;
    R.motorMassPropel = 10;
    R.motorMassFuel = 8;
    R.massPropel = 4;
    R.massFuel = 4;
    R.distanceInterMotors = 0.02;
    R.motorDiameter = 0.11;
    R.length= 2.6;
    R.emptyInertia = 0.25;
    R.thrust2dMassRatio = 0.02;
    R.thrustTime = [0 1.5 1.6 4.5 6];
    R.thrustForce = [0 2500 2500 2400 1200];
end

%% ========================================================================
%   DUMMY THRUST FUNCTION  (used for NonLinear tests)
%% ========================================================================
function y = Thrust(t, Rocket)
    % Deterministic thrust curve:  simply T = 1000*N*sin(t)
    if t<Rocket.burnTime
        y = 1000*sin(pi*t/Rocket.burnTime);
    else
        y=0;
    end
end