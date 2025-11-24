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
            Environment_expected.Temperature_Ground = 3;
            Environment_expected.Pressure_Ground = 1;
            Environment_expected.Humidity_Ground = 4;
            Environment_expected.Start_Altitude = 1;
            Environment_expected.Start_Latitude = 5;
            Environment_expected.Start_Longitude = 9;
            Environment_expected.dTdh = 2;
            Environment_expected.V_inf = 6;
            Environment_expected.V_Azimuth = 5;
            Environment_expected.Turb_I = 3;
            Environment_expected.Turb_model = 'Vroom';
            Environment_expected.Rail_Length = 5;
            Environment_expected.Rail_Angle = 8/180*pi;
            Environment_expected.Rail_Azimuth = 9/180*pi;
            p_ws = exp(77.345+0.0057*Environment_expected.Temperature_Ground-7235/Environment_expected.Temperature_Ground)/Environment_expected.Temperature_Ground^8.2;
            p_a = Environment_expected.Pressure_Ground;
            Environment_expected.Saturation_Vapor_Ratio = 0.62198*p_ws/(p_a-p_ws);
            Environment_expected.V_dir = [cosd(Environment_expected.V_Azimuth);sind(Environment_expected.V_Azimuth); 0];

            % multilayerwind
            Environment_expected.numberLayer = 3;
            layerHeight = [10,100,250];
            layerSpeed = [0.5,2,4];
            layerTurb = [0,0,0];
            axis = 0:10: 4000;
            Environment_expected.Vspeed = interp1(layerHeight,layerSpeed,axis,'pchip','extrap');
            % I don't know how to test for random values, so I leave this untested for now
            Environment_expected.Vazy = Environment.Vazy;
            Environment_expected.Vturb = interp1(layerHeight,layerTurb,axis,'pchip','extrap');
            Environment_expected.Vdirx = cosd(Environment_expected.Vazy);
            Environment_expected.Vdiry = sind(Environment_expected.Vazy);
            Environment_expected.Vdirz = 0*cosd(Environment_expected.Vazy);
            Environment_expected.isWindLayered = 1;

            % map
            [Environment_expected.map_x,Environment_expected.map_y,Environment_expected.map_z] =...
                xyz2grid('maptest.xyz');
            Environment_expected.map_x = Environment_expected.map_x-2648540;
            Environment_expected.map_y = Environment_expected.map_y-1195050;
            Environment_expected.map_z = Environment_expected.map_z-Environment_expected.Start_Altitude;

            % constants; no need to test
            Environment_expected.T_Nu = Environment.T_Nu;
            Environment_expected.Viscosity = Environment.Viscosity;
            
            testCase.verifyThat(Environment, ...
                matlab.unittest.constraints.IsEqualTo(Environment_expected, ...
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
            Environment_expected.Temperature_Ground = 289.15;
            Environment_expected.Pressure_Ground = 102400;
            Environment_expected.Humidity_Ground = 0.7;
            Environment_expected.Start_Altitude = 154;
            Environment_expected.Start_Latitude = 39.393564;
            Environment_expected.Start_Longitude = -8.287676;
            Environment_expected.dTdh = -9.5;
            Environment_expected.V_inf = 2;
            Environment_expected.V_Azimuth = 250;
            Environment_expected.Turb_I = 0;
            Environment_expected.Turb_model = 'None';
            Environment_expected.Rail_Length = 12;
            Environment_expected.Rail_Angle = 5/180*pi;
            Environment_expected.Rail_Azimuth = 156/180*pi;
            p_ws = exp(77.345+0.0057*Environment_expected.Temperature_Ground-7235/Environment_expected.Temperature_Ground)/Environment_expected.Temperature_Ground^8.2;
            p_a = Environment_expected.Pressure_Ground;
            Environment_expected.Saturation_Vapor_Ratio = 0.62198*p_ws/(p_a-p_ws);
            Environment_expected.V_dir = [cosd(Environment_expected.V_Azimuth);sind(Environment_expected.V_Azimuth); 0];

            % Constant fields;
            Environment_expected.T_Nu = Environment.T_Nu;
            Environment_expected.Viscosity = Environment.Viscosity;

            % Verify
            testCase.verifyThat(Environment, ...
                matlab.unittest.constraints.IsEqualTo(Environment_expected, ...
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