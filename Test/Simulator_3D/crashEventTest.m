classdef crashEventTest < matlab.unittest.TestCase
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



        function testGroundImpact(testCase)
            % Setup mock environment and state
            % x(3) is the vertical position
            environment.startAltitude = 0;
            
            % Mock positions: [x; y; z]
            xAbove = [10; 10; 50]; 
            xBelow = [10; 10; -5]; 
            
            % Call the refactored function
            [valAbove, isTerminal, direction] = crashEvent(0, xAbove, environment);
            [valBelow, ~, ~] = crashEvent(0, xBelow, environment);
            
            % Assertions
            testCase.verifyGreaterThan(valAbove, 0, 'Value should be positive when above ground');
            testCase.verifyLessThan(valBelow, 0, 'Value should be negative when below ground');
            testCase.verifyEqual(isTerminal, 1, 'isTerminal must be 1 to stop simulation');
            testCase.verifyEqual(direction, -1, 'Direction must be -1 to detect descent');
        end
    end
end