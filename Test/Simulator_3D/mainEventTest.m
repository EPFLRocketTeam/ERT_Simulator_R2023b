classdef mainEventTest < matlab.unittest.TestCase
    properties
        AddedPath
    end
    methods(Test)
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
            functionPath = fullfile(rootPath, 'Src', 'Simulator_3D');

            % 4. Add the paths to MATLAB's search path
            addpath(functionPath);

            % 5. Store the paths so we can remove it later in TestClassTeardown
            testCase.AddedPath = {functionPath};
        end

        
        function testParachuteDeployment(testCase)
            % Setup rocket parameters
            rocket.paraMainEvent = 450; % Deployment altitude
            
            % Mock states: [x; y; altitude]
            xAbove = [0; 0; 500];
            xBelow = [0; 0; 400];
            
            % Call the refactored function
            [valAbove, isTerminal, direction] = mainEvent(0, xAbove, rocket);
            [valBelow, ~, ~] = mainEvent(0, xBelow, rocket);
            
            % Assertions
            testCase.verifyGreaterThan(valAbove, 0);
            testCase.verifyLessThan(valBelow, 0);
            testCase.verifyEqual(isTerminal, 1);
            testCase.verifyEqual(direction, -1);
        end
    end
end