classdef railEventTest < matlab.unittest.TestCase
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

        function testRailExit(testCase)
            % Setup environment
            environment.railLength = 4.0; 
            
            % Mock states: [positionOnRail; velocity; altitude]
            xBefore = [3.5; 20; 0];
            xAfter  = [4.5; 25; 0];
            
            % Call the refactored function
            [valBefore, isTerminal, direction] = railEvent(0, xBefore, environment);
            [valAfter, ~, ~] = railEvent(0, xAfter, environment);
            
            % Assertions
            testCase.verifyLessThan(valBefore, 0, 'Value should be negative while on rail');
            testCase.verifyGreaterThan(valAfter, 0, 'Value should be positive after leaving rail');
            testCase.verifyEqual(isTerminal, 1);
            testCase.verifyEqual(direction, 1, 'Direction must be 1 for increasing position');
        end
    end
end