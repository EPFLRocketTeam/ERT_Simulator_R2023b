classdef environnementReaderTest < matlab.unittest.TestCase

    % Private property to hold the paths that are added temporarily
    properties (Access = private)
        AddedPath;
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
            functionPath = fullfile(rootPath, 'Src', 'Functions', 'Utilities');
            snippetsPath = fullfile(rootPath, 'Src', 'Snippets');
            
            % 4. Add the paths to MATLAB's search path
            addpath(functionPath);
            addpath(snippetsPath);
            
            % 5. Store the paths so we can remove it later in TestClassTeardown
            testCase.AddedPath = {functionPath};
            testCase.AddedPath = [testCase.AddedPath,snippetsPath];
        end
    end
    
    methods (TestClassTeardown) %
        function removeFunctionPath(testCase)
            % This removes the paths added in TestClassSetup, keeping the MATLAB environment clean.
            for i = 1:numel(testCase.AddedPath)
                rmpath(testCase.AddedPath{i});
            end
        end
    end

    % --- Existing Test Methods ---
    methods (Test)
        
        function testBasicFile(testCase)
            % Path of file to be read by environnementReader
            file = 'Environment_Definition_test.txt';
            % Output of environnementReader
            Environment = environnementReader(file);
            
            % Expected output of environnementReader
            EnvironmentExpected.Temperature_Ground = 3;
            EnvironmentExpected.Pressure_Ground = 1;
            EnvironmentExpected.Humidity_Ground = 4;
            EnvironmentExpected.Start_Altitude = 1;
            EnvironmentExpected.Start_Latitude = 5;
            EnvironmentExpected.Start_Longitude = 9;
            EnvironmentExpected.dTdh = 2;
            EnvironmentExpected.V_inf = 6;
            EnvironmentExpected.V_Azimuth = 5;
            EnvironmentExpected.Turb_I = 3;
            EnvironmentExpected.Turb_model = 'Vroom';
            EnvironmentExpected.Rail_Length = 5;
            EnvironmentExpected.Rail_Angle = 8/180*pi;
            EnvironmentExpected.Rail_Azimuth = 9/180*pi;
            p_ws = exp(77.345+0.0057*EnvironmentExpected.Temperature_Ground-7235/EnvironmentExpected.Temperature_Ground)/EnvironmentExpected.Temperature_Ground^8.2;
            p_a = EnvironmentExpected.Pressure_Ground;
            EnvironmentExpected.Saturation_Vapor_Ratio = 0.62198*p_ws/(p_a-p_ws);
            EnvironmentExpected.V_dir = [cosd(EnvironmentExpected.V_Azimuth);sind(EnvironmentExpected.V_Azimuth); 0];

            % multilayerwind
            EnvironmentExpected.numberLayer = 3;
            layerHeight = [10,100,250];
            layerSpeed = [0.5,2,4];
            layerTurb = [0,0,0];
            axis = 0:10: 4000;
            EnvironmentExpected.Vspeed = interp1(layerHeight,layerSpeed,axis,'pchip','extrap');
            % I don't know how to test for random values, so I leave this untested for now
            EnvironmentExpected.Vazy = Environment.Vazy;
            EnvironmentExpected.Vturb = interp1(layerHeight,layerTurb,axis,'pchip','extrap');
            EnvironmentExpected.Vdirx = cosd(EnvironmentExpected.Vazy);
            EnvironmentExpected.Vdiry = sind(EnvironmentExpected.Vazy);
            EnvironmentExpected.Vdirz = 0*cosd(EnvironmentExpected.Vazy);
            EnvironmentExpected.isWindLayered = 1;

            % map
            [EnvironmentExpected.map_x,EnvironmentExpected.map_y,EnvironmentExpected.map_z] =...
                xyz2grid('maptest.xyz');
            EnvironmentExpected.map_x = EnvironmentExpected.map_x-2648540;
            EnvironmentExpected.map_y = EnvironmentExpected.map_y-1195050;
            EnvironmentExpected.map_z = EnvironmentExpected.map_z-EnvironmentExpected.Start_Altitude;

            % constants; no need to test
            EnvironmentExpected.T_Nu = Environment.T_Nu;
            EnvironmentExpected.Viscosity = Environment.Viscosity;
            
            testCase.verifyThat(Environment, ...
                matlab.unittest.constraints.IsEqualTo(EnvironmentExpected, ...
                'Within', matlab.unittest.constraints.AbsoluteTolerance(1e-10)));
        end

        function testNonexistentFile(testCase)
            % Verify that attempting to read a nonexistent file produces an error
            testCase.verifyError(@() environnementReader('nonexistent.txt'), ...
                'environnementReader:FileNotFound');
        end

        function testEmptyFile(testCase)
            % Test whether it can deal with empty files
            % "Default" values should be assigned
            % Create temporary empty file
            tempFile = 'temp_empty.txt';
            fid = fopen(tempFile, 'w');
            fclose(fid);

            % Read empty file
            Environment = environnementReader(tempFile);

            % Declare expected Environment struct, with default values
            EnvironmentExpected.Temperature_Ground = 289.15;
            EnvironmentExpected.Pressure_Ground = 102400;
            EnvironmentExpected.Humidity_Ground = 0.7;
            EnvironmentExpected.Start_Altitude = 154;
            EnvironmentExpected.Start_Latitude = 39.393564;
            EnvironmentExpected.Start_Longitude = -8.287676;
            EnvironmentExpected.dTdh = -9.5;
            EnvironmentExpected.V_inf = 2;
            EnvironmentExpected.V_Azimuth = 250;
            EnvironmentExpected.Turb_I = 0;
            EnvironmentExpected.Turb_model = 'None';
            EnvironmentExpected.Rail_Length = 12;
            EnvironmentExpected.Rail_Angle = 5/180*pi;
            EnvironmentExpected.Rail_Azimuth = 156/180*pi;
            p_ws = exp(77.345+0.0057*EnvironmentExpected.Temperature_Ground-7235/EnvironmentExpected.Temperature_Ground)/EnvironmentExpected.Temperature_Ground^8.2;
            p_a = EnvironmentExpected.Pressure_Ground;
            EnvironmentExpected.Saturation_Vapor_Ratio = 0.62198*p_ws/(p_a-p_ws);
            EnvironmentExpected.V_dir = [cosd(EnvironmentExpected.V_Azimuth);sind(EnvironmentExpected.V_Azimuth); 0];

            % Constant fields;
            EnvironmentExpected.T_Nu = Environment.T_Nu;
            EnvironmentExpected.Viscosity = Environment.Viscosity;

            % Verify
            testCase.verifyThat(Environment, ...
                matlab.unittest.constraints.IsEqualTo(EnvironmentExpected, ...
                'Within', matlab.unittest.constraints.AbsoluteTolerance(1e-10)));

            % Delete temporary file
            delete(tempFile);
        end
        
        function testMalformedData(testCase)
            % Test with malformed data lines
            tempFile = 'temp_malformed.txt';
            fid = fopen(tempFile, 'w');
            fprintf(fid, 'Temperature_Ground gibberish\n');
            fclose(fid);
            
            % Should handle parsing errors
            testCase.verifyError(@() environnementReader(tempFile), 'environnementReader:NaN');

            % Delete temporary file
            delete(tempFile);
        end
    end
end