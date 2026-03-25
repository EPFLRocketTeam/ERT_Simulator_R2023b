classdef MainTest < matlab.unittest.TestCase
    % Comprehensive test suite for ERT Simulator components
    
    properties
        ProjectRoot
        OriginalWarningState
    end
    
    methods (TestClassSetup)
        function setup(testCase)
            % Get project root
            testDir = fileparts(mfilename('fullpath'));
            testCase.ProjectRoot = fileparts(testDir);  
            
            % Add Src paths
            addpath(genpath(fullfile(testCase.ProjectRoot, 'Src', 'Declarations')));
            addpath(genpath(fullfile(testCase.ProjectRoot, 'Src', 'Functions')));
            addpath(genpath(fullfile(testCase.ProjectRoot, 'Src', 'Snippets')));
            addpath(genpath(fullfile(testCase.ProjectRoot, 'Src', 'Simulator_3D')));
            
            % Store original warning state for restoration
            testCase.OriginalWarningState = warning('query');
            % Suppress warnings during tests to reduce noise
            warning('off', 'all');
        end
    end
    
    methods (TestClassTeardown)
        function teardown(testCase)
            % Restore original warning state
            warning(testCase.OriginalWarningState);
        end
    end
    
    methods (Test)
        function testConfigFilesExist(testCase)
            % Test that configuration files exist
            
            filesToCheck = {
                fullfile('Src', 'Declarations', 'Rocket', 'Nordend_EUROC.txt')
                fullfile('Src', 'Declarations', 'Environment', 'Environnement_Definition_EuRoC.txt')
                fullfile('Src', 'Declarations', 'Simulation', 'Simulation_outputs.txt')
                };
            
            for i = 1:length(filesToCheck)
                fullPath = fullfile(testCase.ProjectRoot, filesToCheck{i});
                testCase.verifyTrue(exist(fullPath, 'file') == 2, ...
                    sprintf('File does not exist: %s', filesToCheck{i}));
            end
        end
        
        function testCanLoadRocketConfig(testCase)
            % Test that rocket configuration can be loaded
            
            rocketFile = fullfile(testCase.ProjectRoot, 'Src', 'Declarations', 'Rocket', 'Nordend_EUROC.txt');
            
            try
                Rocket = rocketReader(rocketFile);
                testCase.verifyTrue(isstruct(Rocket), 'Rocket should be a struct');
                testCase.verifyTrue(isfield(Rocket, 'motorState'), 'Rocket should have motorState field');
                testCase.verifyTrue(isfield(Rocket, 'Burn_Time'), 'Rocket should have Burn_Time field');
            catch ME
                testCase.verifyFail(sprintf('Failed to load rocket: %s', ME.message));
            end
        end
        
        function testCanLoadEnvironmentConfig(testCase)
            % Test that environment configuration can be loaded
            
            envFile = fullfile(testCase.ProjectRoot, 'Src', 'Declarations', 'Environment', 'Environnement_Definition_EuRoC.txt');
            
            try
                Environment = environnementReader(envFile);
                testCase.verifyTrue(isstruct(Environment), 'Environment should be a struct');
                testCase.verifyTrue(isfield(Environment, 'startLatitude'), 'Environment should have startLatitude field');
            catch ME
                testCase.verifyFail(sprintf('Failed to load environment: %s', ME.message));
            end
        end
        
        function testMainScriptExists(testCase)
            % Test that main.m exists somewhere
            
            % Check common locations
            possiblePaths = {
                testCase.ProjectRoot
                fullfile(testCase.ProjectRoot, 'Src')
                fullfile(testCase.ProjectRoot, 'Test')
                };
            
            found = false;
            mainPath = '';
            for i = 1:length(possiblePaths)
                candidate = fullfile(possiblePaths{i}, 'main.m');
                if exist(candidate, 'file') == 2
                    found = true;
                    mainPath = candidate;
                    break;
                end
            end
            
            testCase.verifyTrue(found, 'main.m should exist in the project');
            if found
                fprintf('Found main.m at: %s\n', mainPath);
            end
        end
        
        function testMainScriptRunsWithoutError(testCase)
            % Smoke check for main.m; if it contains destructive commands,
            % run it in an isolated MATLAB process.
            
            mainPath = fullfile(testCase.ProjectRoot, 'Src', 'main.m');
            testCase.assumeTrue(exist(mainPath, 'file') == 2, 'main.m must exist for this test');
            
            mainSource = lower(fileread(mainPath));
            hasClearAll = contains(mainSource, 'clear all');
            hasCloseAll = contains(mainSource, 'close all');
            hasClc = contains(mainSource, 'clc');
            
            if hasClearAll || hasCloseAll || hasClc
                % Because script includes destructive/global commands, run as isolated process via a temp wrapper script
                scriptDir = fullfile(testCase.ProjectRoot, 'Src');
                tempMain = fullfile(scriptDir, 'main_auto.m');

                % Copy main into temporary file with pre-set non-interactive option
                originalMainCode = fileread(fullfile(scriptDir, 'main.m'));
                fid = fopen(tempMain, 'w');
                fprintf(fid, "plotShowAnswer = 'N';\n");
                fprintf(fid, '%s', originalMainCode);
                fclose(fid);

                try
                    batchCmd = sprintf('matlab -batch "cd(''%s''); addpath(genpath(''%s'')); run(''main_auto.m''); exit;"', scriptDir, scriptDir);
                    [errno, batchOutput] = system(batchCmd);
                    testCase.verifyEqual(errno, 0, sprintf('main.m failed in isolated batch: %s', batchOutput));
                catch ME
                    if exist(tempMain, 'file')
                        delete(tempMain);
                    end
                    rethrow(ME);
                end

                if exist(tempMain, 'file')
                    delete(tempMain);
                end
            else
                % No dangerous commands: can run in-process
                originalDir = pwd();
                cd(fullfile(testCase.ProjectRoot, 'Src'));
                try
                    run('main.m');
                catch ME
                    testCase.verifyFail(sprintf('main.m failed to execute in-process: %s', ME.message));
                end
                cd(originalDir);
            end
        end
        
        function testRocketHasRequiredFields(testCase)
            % Test that loaded rocket has all required fields for simulation
            
            rocketFile = fullfile(testCase.ProjectRoot, 'Src', 'Declarations', 'Rocket', 'Nordend_EUROC.txt');
            Rocket = rocketReader(rocketFile);
            
            requiredFields = {
                'motorState', 'numStages', 'stageDiameters', 'stagePositions', ...
                'numFins', 'finSpan', 'finRootChord', 'finTipChord', 'emptyMass', ...
                'emptyInertia', 'emptyCenterOfMass', 'maxDiameter', 'motorId', 'Burn_Time'
            };
            
            for i = 1:length(requiredFields)
                field = requiredFields{i};
                testCase.verifyTrue(isfield(Rocket, field), ...
                    sprintf('Rocket struct missing required field: %s', field));
            end
            
            % Test field types and reasonable ranges
            testCase.verifyTrue(ischar(Rocket.motorState), 'Rocket.motorState should be string');
            testCase.verifyTrue(Rocket.emptyMass > 0, 'Rocket.emptyMass should be positive');
            testCase.verifyTrue(Rocket.emptyCenterOfMass >= 0, 'Rocket.emptyCenterOfMass should be non-negative');
            testCase.verifyGreaterThan(Rocket.Burn_Time(end), 0, 'Burn_Time should be positive');
        end
        
        function testEnvironmentHasRequiredFields(testCase)
            % Test that loaded environment has all required fields
            
            envFile = fullfile(testCase.ProjectRoot, 'Src', 'Declarations', 'Environment', 'Environnement_Definition_EuRoC.txt');
            Environment = environnementReader(envFile);
            
            requiredFields = {
                'startLatitude', 'startLongitude', 'startAltitude', ...
                'groundTemperature', 'groundPressure', 'dTdh', 'V_inf'
            };
            
            for i = 1:length(requiredFields)
                field = requiredFields{i};
                testCase.verifyTrue(isfield(Environment, field), ...
                    sprintf('Environment struct missing required field: %s', field));
            end
            
            % Test reasonable ranges
            testCase.verifyTrue(Environment.startLatitude >= -90 && Environment.startLatitude <= 90, ...
                'Latitude should be between -90 and 90 degrees');
            testCase.verifyTrue(Environment.startAltitude >= 0, 'Altitude should be non-negative');
        end
        
        function testKeyFunctionsExist(testCase)
            % Test that key computational functions exist and are callable
            
            functionsToCheck = {
                'drag', 'normalLift', 'massProperties', 'findAltitude', ...
                'ApogeeEvent', 'RailEvent', 'Simulator3D'
            };
            
            for i = 1:length(functionsToCheck)
                funcName = functionsToCheck{i};
                testCase.verifyTrue(exist(funcName, 'file') == 2, ...
                    sprintf('Function %s should exist', funcName));
            end
        end
        
        function testDragFunctionBasic(testCase)
            % Test basic functionality of drag function
            
            rocketFile = fullfile(testCase.ProjectRoot, 'Src', 'Declarations', 'Rocket', 'Nordend_EUROC.txt');
            Rocket = rocketReader(rocketFile);
            
            % Test drag at zero angle of attack and low speed
            alpha = 0; % rad
            velocity = 10; % m/s
            kinematicViscosity = 1.5e-5;
            speedOfSound = 346;
            
            dragCoeff = drag(Rocket, alpha, velocity, kinematicViscosity, speedOfSound);
            
            testCase.verifyTrue(isscalar(dragCoeff), 'drag should return scalar');
            testCase.verifyTrue(dragCoeff >= 0, 'Drag coefficient should be non-negative');
            testCase.verifyTrue(dragCoeff < 10, 'Drag coefficient seems unreasonably high');
        end
        
        function testNormalLiftFunctionBasic(testCase)
            % Test basic functionality of normalLift function
            
            rocketFile = fullfile(testCase.ProjectRoot, 'Src', 'Declarations', 'Rocket', 'Nordend_EUROC.txt');
            Rocket = rocketReader(rocketFile);
            
            % Test normal lift at small angle of attack
            alpha = deg2rad(5); % 5 degrees
            airbrakeDeployment = 1.1;
            mach = 0.5;
            rollRate = 0;
            useNonlinear = 1;
            
            [cn, cp] = normalLift(Rocket, alpha, airbrakeDeployment, mach, rollRate, useNonlinear);
            
            testCase.verifyTrue(isscalar(cn), 'normalLift should return scalar cn');
            testCase.verifyTrue(isscalar(cp), 'normalLift should return scalar cp');
            testCase.verifyTrue(cp > 0, 'Center of pressure should be positive (from nose)');
        end
    end
end