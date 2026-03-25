classdef ApogeeEventTest < matlab.unittest.TestCase

    % Private property to hold the paths that are added temporarily
    properties (Access = private)
        AddedPath;
    end
    
    methods (TestClassSetup) %
        function addFunctionPath(testCase)
            % This function temporarily adds the directory of the findAltitude 
            % function to the MATLAB path so the tests can find it
            
            % 1. Get the path of the current test file directory 
            % (e.g., ...\ERT_Simulator_R2023b\Test\Functions\Math)
            testDir = fileparts(mfilename('fullpath'));
            
            % 2. Move up two directories to reach the root folder 
            % (e.g., ...\ERT_Simulator_R2023b)
            rootPath = fileparts(fileparts(testDir));
            
            % 3. Construct the path to the function file's directory 
            % (e.g., ...\ERT_Simulator_R2023b\Src\Simulator_3D)
            functionPath = fullfile(rootPath, 'Src', 'Simulator_3D');
            
            % 4. Add the paths to MATLAB's search path
            addpath(functionPath);
            
            % 5. Store the paths so we can remove it later in TestClassTeardown
            testCase.AddedPath = functionPath;
        end
    end
    
    methods (TestClassTeardown) %
        function removeFunctionPath(testCase)
            % This removes the paths added in TestClassSetup, keeping the MATLAB environment clean.
            rmpath(testCase.AddedPath);
        end
    end

    % --- Existing Test Methods ---
    methods (Test)
        
        function testBasic(testCase)
            for i = 1:5  % test 5 random values
                % declare state, time
                X6 = rand(1);
                X = [0,0,0,0,0,X6];  % only 6th component matters
                T = 0;  % doesn't matter

                % declare expected output
                valueExpected = X6;

                % verify
                testCase.verifyEqual(ApogeeEvent(T,X), valueExpected);
            end
        end
    end
end