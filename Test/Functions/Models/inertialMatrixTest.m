classdef inertialMatrixTest < matlab.unittest.TestCase
    % Unit tests for inertialMatrtix.m

    properties
        AbsTol = 1e-10;
        AddedPath
    end
    
    properties (TestParameter)
        testParameter1 = struct("scalar",1,"vector",[1 1]);
    end

    methods (Test)

        function addFunctionPath(testCase)
            % This function temporarily adds the directory of the
            % inertialMatrix function to the MATLAB path so the tests can find it

            % 1. Get the path of the current test file directory
            % (e.g., ...\ERT_Simulator_R2023b\Test\Functions\Math)
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
        %   STANDARD MOTOR
        %% -------------------------------------------------------------

        function testSymetricMatrix_t0(testCase)
            R = dummyRocket();
            t = 0;
            matrixI = inertialMatrix(R, R.Cm, t);
            testCase.verifyEqual(matrixI,matrixI.','AbsTol',testCase.AbsTol);
        end

        function testSymetricMatrixMidBurn(testCase)
            R = dummyRocket();
            t = R.burnTime/2;
            matrixI = inertialMatrix(R, R.Cm, t);
            testCase.verifyEqual(matrixI,matrixI.','AbsTol',testCase.AbsTol);
        end

        function testSymetricMatrixAfterBurn(testCase)
            R = dummyRocket();
            t = R.burnTime+1;
            matrixI = inertialMatrix(R, R.Cm, t);
            testCase.verifyEqual(matrixI,matrixI.','AbsTol',testCase.AbsTol);
        end


        %% -------------------------------------------------------------
        %   GENERAL SANITY CHECKS
        %% -------------------------------------------------------------

        function testDiagPositve(testCase)
            R = dummyRocket();
            t = linspace(0, R.burnTime, 1000);

            MatrixI = inertialMatrix(R,R.Cm,t);

            testCase.verifyGreaterThanOrEqual(MatrixI(1,1),0);
            testCase.verifyGreaterThanOrEqual(MatrixI(2,2),0);
            testCase.verifyGreaterThanOrEqual(MatrixI(3,3),0);

        end

        function testFinite(testCase)
            R = dummyRocket();
            t = linspace(0, R.burnTime, 1000);
            MatrixI = inertialMatrix(R,R.Cm,t);
            testCase.verifyFinite(MatrixI)
        end
    end
end

%% ========================================================================
%   DUMMY ROCKET GENERATORS
%% ========================================================================

function R = dummyRocket()
R.tankZ = 1.2;
R.burnTime = 6;
R.tankL = 0.75;
R.propelMass = 2;
R.rocketInertia = 1e6; %iykyk
R.tankR = 0.5;
R.Cm = 1.8;
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