classdef Simulator3DTest < matlab.unittest.TestCase
    %SIMULATOR3DTest class for Simulator3D rocket simulation
    
    properties
        TestRocket
        TestEnvironment  
        TestSimOutput
        AddedPath
    end
    
    methods(TestClassSetup)
        function setupPaths(testCase)
            % Add the path to the Simulator3D class
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            srcPath = fullfile(projectRoot, 'Src', 'Simulator_3D');
            addpath(srcPath);
            testCase.AddedPath = srcPath;
        end
        
        function createTestComponents(testCase)
            % Create test components
            testCase.TestRocket = struct();
            testCase.TestRocket.Burn_Time = 5;
            testCase.TestRocket.rocket_m = 50;
            testCase.TestRocket.motor_mass = 10;
            testCase.TestRocket.propel_mass = 8;
            testCase.TestRocket.casing_mass = 2;
            testCase.TestRocket.rocket_cm = 1.2;
            testCase.TestRocket.L = 2.0;
            testCase.TestRocket.motor_length = 0.5;
            testCase.TestRocket.motor_dia = 0.1;
            testCase.TestRocket.dm = 0.15;
            testCase.TestRocket.Sm = pi*(0.15/2)^2;
            testCase.TestRocket.rocket_inertia = eye(3) * 0.1;
            testCase.TestRocket.motor_fac = 1.0;
            testCase.TestRocket.CD_fac = 1.0;
            testCase.TestRocket.para_main_SCD = 10;
            testCase.TestRocket.para_drogue_SCD = 5;
            testCase.TestRocket.para_main_event = 500;
            testCase.TestRocket.pl_mass = 5;
            testCase.TestRocket.ab_phi = 0;
            testCase.TestRocket.cp_fac = 1.0;
            testCase.TestRocket.CNa_fac = 1.0;
            
            % Mock thrust curve
            testCase.TestRocket.Thrust_Time = [0, 2.5, 5];
            testCase.TestRocket.Thrust_Force = [0, 2000, 0];
            testCase.TestRocket.Thrust2dMass_Ratio = 0.001;
            
            % Create test environment
            testCase.TestEnvironment = struct();
            testCase.TestEnvironment.railLength = 10;
            testCase.TestEnvironment.railAngle = deg2rad(2);
            testCase.TestEnvironment.railAzimuth = deg2rad(0);
            testCase.TestEnvironment.startAltitude = 1000;
            testCase.TestEnvironment.groundTemperature = 288.15;
            testCase.TestEnvironment.V_inf = [0; 0; 0];
            testCase.TestEnvironment.V_dir = [1; 0; 0];
            testCase.TestEnvironment.Turb_I = 0;
            testCase.TestEnvironment.Turb_model = 'None';
            
            % Create simulation output configuration
            testCase.TestSimOutput = struct();
            testCase.TestSimOutput.Margin = true;
            testCase.TestSimOutput.Alpha = true;
            testCase.TestSimOutput.Cn_alpha = true;
            testCase.TestSimOutput.Xcp = true;
            testCase.TestSimOutput.Cd = true;
            testCase.TestSimOutput.Mass = true;
            testCase.TestSimOutput.CM = true;
            testCase.TestSimOutput.Il = true;
            testCase.TestSimOutput.Ir = true;
            testCase.TestSimOutput.Delta = true;
            testCase.TestSimOutput.Nose_Alpha = true;
            testCase.TestSimOutput.Nose_Delta = true;
        end
    end
    
    methods(TestClassTeardown)
        function cleanupPaths(testCase)
            if ~isempty(testCase.AddedPath)
                rmpath(testCase.AddedPath);
            end
        end
    end
    
    methods(Test)
        function testConstructor(testCase)
            % Test basic constructor functionality
            sim = Simulator3D(testCase.TestRocket, ...
                            testCase.TestEnvironment, ...
                            testCase.TestSimOutput);
            
            testCase.verifyEqual(sim.rocket, testCase.TestRocket);
            testCase.verifyEqual(sim.environment, testCase.TestEnvironment);
            testCase.verifyEqual(sim.simOutput, testCase.TestSimOutput);
            testCase.verifyTrue(isstruct(sim.simAuxResults));
            
            % Verify SimAuxResults is initialized correctly
            auxFields = {'stabilityMargin', 'angleOfAttack', 'normalForceCoefficientSlope', 'centerOfPressure', 'dragCoefficient', ...
                        'mass', 'centerOfMass', 'inertiaLong', 'inertiaRot', 'flightPathAngle', ...
                        'noseAngleOfAttack', 'noseFlightPathAngle'};
            for i = 1:length(auxFields)
                testCase.verifyTrue(isfield(sim.simAuxResults, auxFields{i}));
                testCase.verifyTrue(isempty(sim.simAuxResults.(auxFields{i})));
            end
        end
        
        function testConstructorNoArguments(testCase)
            % Test constructor with no arguments (should not error)
            sim = Simulator3D();
            
            % Verify properties are empty
            testCase.verifyTrue(isempty(sim.rocket));
            testCase.verifyTrue(isempty(sim.environment));
            testCase.verifyTrue(isempty(sim.simOutput));
            
            % Verify SimAuxResults is still initialized
            testCase.verifyTrue(isstruct(sim.simAuxResults));
            testCase.verifyTrue(isfield(sim.simAuxResults, 'stabilityMargin'));
        end
        
        % function testConstructorErrorHandling(testCase)
        %     % Test constructor error cases - FIXED VERSION
        %     % Use verifyError without checking the exact message
        %     testCase.verifyError(@() Simulator3D(1, 2), '');
            
        %     % Alternatively, verify that it throws an error
        %     % (the exact message might vary by MATLAB version)
        %     try
        %         Simulator3D(1, 2);
        %         testCase.verifyFail('Expected an error but none was thrown');
        %     catch ME
        %         % Verify the error contains the expected substring
        %         testCase.verifySubstring(ME.message, ...
        %             'either no arguments or 3 arguments can be given');
        %     end
        % end
        
        % function testConstructorErrorHandlingAlt(testCase)
        %     % Alternative test for constructor errors
        %     % Just verify that wrong number of arguments causes an error
        %     testCase.assertError(@() Simulator3D(1, 2), '');
            
        %     % Test with 1 argument
        %     testCase.assertError(@() Simulator3D(1), '');
            
        %     % Test with 4 arguments
        %     testCase.assertError(@() Simulator3D(1, 2, 3, 4), '');
        % end
        
        function testPropertyAssignment(testCase)
            % Test that we can assign to public properties
            sim = Simulator3D();
            
            % Assign new values
            newRocket = struct('test', 1);
            newEnvironment = struct('test', 2);
            newSimOutput = struct('test', 3);
            
            sim.rocket = newRocket;
            sim.environment = newEnvironment;
            sim.simOutput = newSimOutput;
            
            testCase.verifyEqual(sim.rocket, newRocket);
            testCase.verifyEqual(sim.environment, newEnvironment);
            testCase.verifyEqual(sim.simOutput, newSimOutput);
        end
        
        function testSimAuxResultsStructure(testCase)
            % Test SimAuxResults structure fields
            sim = Simulator3D(testCase.TestRocket, ...
                            testCase.TestEnvironment, ...
                            testCase.TestSimOutput);
            
            % Check all expected fields exist and are empty
            expectedFields = {'stabilityMargin', 'angleOfAttack', 'normalForceCoefficientSlope', 'centerOfPressure', 'dragCoefficient', ...
                             'mass', 'centerOfMass', 'inertiaLong', 'inertiaRot', 'flightPathAngle', ...
                             'noseAngleOfAttack', 'noseFlightPathAngle'};
            
            for i = 1:length(expectedFields)
                testCase.verifyTrue(isfield(sim.simAuxResults, expectedFields{i}), ...
                    sprintf('Field %s should exist in simAuxResults', expectedFields{i}));
                testCase.verifyTrue(isempty(sim.simAuxResults.(expectedFields{i})), ...
                    sprintf('Field %s should be initially empty', expectedFields{i}));
            end
        end
        
        % function testRailSimBasicExecution(testCase)
        %     % Test that RailSim executes without errors
        %     sim = Simulator3D(testCase.TestRocket, ...
        %                     testCase.TestEnvironment, ...
        %                     testCase.TestSimOutput);
            
        %     % Use assumeFail to skip if dependencies are missing
        %     try
        %         [T, S] = sim.RailSim();
                
        %         testCase.verifyNotEmpty(T, 'Time vector should not be empty');
        %         testCase.verifyNotEmpty(S, 'State vector should not be empty');
        %         testCase.verifyEqual(size(S, 2), 2, 'State should have 2 columns (position, velocity)');
                
        %         % Basic sanity checks
        %         testCase.verifyGreaterThanOrEqual(T(end), 0, 'Time should be non-negative');
        %         testCase.verifyGreaterThanOrEqual(S(end, 1), 0, 'Position should be non-negative');
        %     catch ME
        %         if contains(ME.message, 'undefined function', 'IgnoreCase', true)
        %             % Skip test if dependencies are missing
        %             testCase.assumeFail(['RailSim failed due to missing dependencies: ' ME.message]);
        %         else
        %             rethrow(ME);
        %         end
        %     end
        % end
        
        % function testFlightSimBasicExecution(testCase)
        %     % Test that FlightSim executes without errors
        %     sim = Simulator3D(testCase.TestRocket, ...
        %                     testCase.TestEnvironment, ...
        %                     testCase.TestSimOutput);
            
        %     tspan = [0, 0.1]; % Very short simulation
        %     initialVelocity = 50; % m/s
            
        %     try
        %         [T, S, TE, SE, IE] = sim.FlightSim(tspan, initialVelocity);
                
        %         testCase.verifyNotEmpty(T, 'Time vector should not be empty');
        %         testCase.verifyNotEmpty(S, 'State vector should not be empty');
        %         testCase.verifyEqual(size(S, 2), 13, 'State should have 13 columns (3 pos, 3 vel, 4 quat, 3 omega)');
        %     catch ME
        %         if contains(ME.message, 'undefined function', 'IgnoreCase', true)
        %             testCase.assumeFail(['FlightSim failed due to missing dependencies: ' ME.message]);
        %         else
        %             rethrow(ME);
        %         end
        %     end
        % end
    end
end