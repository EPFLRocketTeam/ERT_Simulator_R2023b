classdef find_altitude < matlab.unittest.TestCase

    % Private property to hold the paths that are added temporarily
    properties (Access = private)
        AddedPath;
        AddedPath2;
    end
    
    methods (TestClassSetup) %
        function addFunctionPath(testCase)
            % This function temporarily adds the directory of the find_altitude 
            % function to the MATLAB path so the tests can find it
            
            % 1. Get the path of the current test file directory 
            % (e.g., ...\ERT_Simulator_R2023b\Test\Functions\Math)
            testDir = fileparts(mfilename('fullpath'));
            
            % 2. Move up two directories to reach the root folder 
            % (e.g., ...\ERT_Simulator_R2023b)
            rootPath = fileparts(fileparts(fileparts(testDir))); 
            
            % 3. Construct the path to the function file's directory 
            % (e.g., ...\ERT_Simulator_R2023b\Src\Simulator_3D)
            functionPath = fullfile(rootPath, 'Src', 'Simulator_3D');
            % 3.1 Also add the path of useful utility functions
            % (e.g., the environnementReader function);
            functionPath2 = fullfile(rootPath, 'Src', 'Functions', 'Utilities')
            
            % 4. Add the paths to MATLAB's search path
            addpath(functionPath);
            addpath(functionPath2);
            
            % 5. Store the paths so we can remove it later in TestClassTeardown
            testCase.AddedPath = functionPath;
            testCase.AddedPath2 = functionPath2;
        end
    end
    
    methods (TestClassTeardown) %
        function removeFunctionPath(testCase)
            % This removes the paths added in TestClassSetup, keeping the MATLAB environment clean.
            rmpath(testCase.AddedPath);
            rmpath(testCase.AddedPath2);
        end
    end

    % --- Existing Test Methods ---
    methods (Test)
        
        function testBasicMap(testCase)
            % Test a standard 4x4x4 map, and standard 1x2 arrays for X,Y
            % Define Environment.map
            Environment.map_x =
            [1, 5, 9, 13;
             1, 5, 9, 13;
             1, 5, 9, 13;
             1, 5, 9, 13]
            Environment.map_y =
            [13,13,13,13;
             9, 9, 9, 9 ;
             5, 5, 5, 5 ;
             1, 1, 1, 1 ]
            Environment.map_z = 
            [1, 2, 3, 4 ;
             5, 6, 7, 8 ;
             9, 10,11,12;
             13,14,15,16]

            % Define X,Y and expected Z
            X = [4,10];
            Y = [6,12];
            expected_Z = [10,3];

            % Verify
            Z = find_altitude(X,Y,Environment)
            testCase.verifyThat(Z, ...
                matlab.unittest.constraints.IsEqualTo(expected_Z, ...
                'Within', matlab.unittest.constraints.AbsoluteTolerance(1e-10)));
        end
        
    end
end