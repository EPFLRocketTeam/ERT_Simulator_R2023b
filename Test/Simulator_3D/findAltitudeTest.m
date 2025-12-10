classdef findAltitudeTest < matlab.unittest.TestCase

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
        
        function testBasicMap(testCase)
            % Test a standard 4x4x4 map, and standard 1x2 arrays for X,Y
            % Define a standard Environment.map
            Environment.map_x =...
            [1, 5, 9, 13;
             1, 5, 9, 13;
             1, 5, 9, 13;
             1, 5, 9, 13];
            Environment.map_y =...
            [13,13,13,13;
             9, 9, 9, 9 ;
             5, 5, 5, 5 ;
             1, 1, 1, 1 ];
            Environment.map_z =...
            [1, 2, 3, 4 ;
             5, 6, 7, 8 ;
             9, 10,11,12;
             13,14,15,16];
            % Set the start altitude to 0
            Environment.startAltitude = 0;

            % Define X,Y and expected Z
            X = [4,10];
            Y = [6,12];
            expectedZ = [10,3]-Environment.startAltitude;

            % Verify
            Z = findAltitude(X,Y,Environment);
            testCase.verifyThat(Z, ...
                matlab.unittest.constraints.IsEqualTo(expectedZ, ...
                'Within', matlab.unittest.constraints.AbsoluteTolerance(0)));
        end

        function testEmptyMap(testCase)
            % Test whether it still works when the map is empty
            % Define Environment.map as an empty map
            Environment.map_x =...
            [];
            Environment.map_y =...
            [];
            Environment.map_z =...
            [];
            % Set the start altitude to 0
            Environment.startAltitude = 0;

            % Define X,Y and expected Z
            X = [4,10];
            Y = [6,12];
            expectedZ = [0,0]-Environment.startAltitude;

            % Verify
            Z = findAltitude(X,Y,Environment);
            testCase.verifyThat(Z, ...
                matlab.unittest.constraints.IsEqualTo(expectedZ, ...
                'Within', matlab.unittest.constraints.AbsoluteTolerance(0)));
        end
        
        function testEmptyXY(testCase)
            % Tests whether it still works when X,Y are empty arrays
            % Define a standard Environment.map
            Environment.map_x =...
            [1, 5, 9, 13;
             1, 5, 9, 13;
             1, 5, 9, 13;
             1, 5, 9, 13];
            Environment.map_y =...
            [13,13,13,13;
             9, 9, 9, 9 ;
             5, 5, 5, 5 ;
             1, 1, 1, 1 ];
            Environment.map_z =...
            [1, 2, 3, 4 ;
             5, 6, 7, 8 ;
             9, 10,11,12;
             13,14,15,16];
            % Set the start altitude to 0
            Environment.startAltitude = 0;

            % Define X,Y and expected Z
            X = [];
            Y = [];
            expectedZ = [];

            % Verify
            Z = findAltitude(X,Y,Environment);
            testCase.verifyThat(Z, ...
                matlab.unittest.constraints.IsEqualTo(expectedZ, ...
                'Within', matlab.unittest.constraints.AbsoluteTolerance(0)));
        end
        
        function testStartAltitude(testCase)
            % Test a non-zero starting altitude
            % Define a standard Environment.map
            Environment.map_x =...
            [1, 5, 9, 13;
             1, 5, 9, 13;
             1, 5, 9, 13;
             1, 5, 9, 13];
            Environment.map_y =...
            [13,13,13,13;
             9, 9, 9, 9 ;
             5, 5, 5, 5 ;
             1, 1, 1, 1 ];
            Environment.map_z =...
            [1, 2, 3, 4 ;
             5, 6, 7, 8 ;
             9, 10,11,12;
             13,14,15,16];
            % Set a non-zero start altitude
            Environment.startAltitude = 53;

            % Define X,Y and expected Z
            X = [4,10];
            Y = [6,12];
            expectedZ = [10,3]-Environment.startAltitude;

            % Verify
            Z = findAltitude(X,Y,Environment);
            testCase.verifyThat(Z, ...
                matlab.unittest.constraints.IsEqualTo(expectedZ, ...
                'Within', matlab.unittest.constraints.AbsoluteTolerance(0)));
        end
        
        function testDifferentShapeXY(testCase)
            % Test whether it gives the expected result
            % when X,Y are of different shape
            % Define a standard Environment.map
            Environment.map_x =...
            [1, 5, 9, 13;
             1, 5, 9, 13;
             1, 5, 9, 13;
             1, 5, 9, 13];
            Environment.map_y =...
            [13,13,13,13;
             9, 9, 9, 9 ;
             5, 5, 5, 5 ;
             1, 1, 1, 1 ];
            Environment.map_z =...
            [1, 2, 3, 4 ;
             5, 6, 7, 8 ;
             9, 10,11,12;
             13,14,15,16];
            % Set the start altitude to 0
            Environment.startAltitude = 0;

            % Define X,Y, where numel(X) > numel(Y)
            X = [4,8;10,2];
            Y = [6,12,13];
            % X should be truncated to [4,10,8]
            % Define expected Z
            expectedZ = [10,3,3]-Environment.startAltitude;

            % Verify
            Z = findAltitude(X,Y,Environment);
            testCase.verifyThat(Z, ...
                matlab.unittest.constraints.IsEqualTo(expectedZ, ...
                'Within', matlab.unittest.constraints.AbsoluteTolerance(0)));

            % Redefine Y, so that numel(X) < numel(Y)
            Y = [6,9,4;12,8,3;13,1,5];
            % The extra values from Y should be ignored;
            % X is treated as [4,10,8,2]
            % Y is treated as [6,12,13,9]
            % Redefine expected Z; this time, X is not truncated,
            % so Z will have the same shape as X (i.e. 2x2)
            expectedZ = [10,3;3,5];

            % Verify
            Z = findAltitude(X,Y,Environment);
            testCase.verifyThat(Z, ...
                matlab.unittest.constraints.IsEqualTo(expectedZ, ...
                'Within', matlab.unittest.constraints.AbsoluteTolerance(0)));
        end
        
        function testXYOutsideMap(testCase)
            % Test whether it still works when (X,Y) values
            % are outside the map
            % Define a standard Environment.map
            Environment.map_x =...
            [1, 5, 9, 13;
             1, 5, 9, 13;
             1, 5, 9, 13;
             1, 5, 9, 13];
            Environment.map_y =...
            [13,13,13,13;
             9, 9, 9, 9 ;
             5, 5, 5, 5 ;
             1, 1, 1, 1 ];
            Environment.map_z =...
            [1, 2, 3, 4 ;
             5, 6, 7, 8 ;
             9, 10,11,12;
             13,14,15,16];
            % Set the start altitude to 0
            Environment.startAltitude = 0;

            % Define X,Y with coordinates not within 1 of
            % values of the map
            X = [3,100];
            Y = [7,-100];
            % Define expected Z; since there are no matches for (X,Y)
            % within the map, the altitude is assume to be the same as
            % the start altitude; hence, the relative altitude will be 0
            expectedZ = [0,0];

            % Verify
            Z = findAltitude(X,Y,Environment);
            testCase.verifyThat(Z, ...
                matlab.unittest.constraints.IsEqualTo(expectedZ, ...
                'Within', matlab.unittest.constraints.AbsoluteTolerance(0)));

            % Verify with a different start altitude to make sure
            % we did not get lucky
            Environment.startAltitude = 53;
            Z = findAltitude(X,Y,Environment);
            testCase.verifyThat(Z, ...
                matlab.unittest.constraints.IsEqualTo(expectedZ, ...
                'Within', matlab.unittest.constraints.AbsoluteTolerance(0)));
        end

        function testMultipleMatches(testCase)
            % Test whether it still works when there are multiple
            % values of the map close to (X,Y)

            % Define a denser Environment.map
            Environment.map_x =...
            [1, 2, 3, 4 ;
             1, 2, 3, 4 ;
             1, 2, 3, 4 ;
             1, 2, 3, 4];
            Environment.map_y =...
            [4, 4, 4, 4 ;
             3, 3, 3, 3 ;
             2, 2, 2, 2 ;
             1, 1, 1, 1 ];
            Environment.map_z =...
            [1, 2, 3, 4 ;
             5, 6, 7, 8 ;
             9, 10,11,12;
             13,14,15,16];
            % Set the start altitude to 0
            Environment.startAltitude = 0;

            % Define X,Y and expected Z
            X = [2,3];
            Y = [1,4];
            expectedZ = [9,2];

            % Verify
            Z = findAltitude(X,Y,Environment);
            testCase.verifyThat(Z, ...
                matlab.unittest.constraints.IsEqualTo(expectedZ, ...
                'Within', matlab.unittest.constraints.AbsoluteTolerance(1e-10)));
        end
        
    end
end