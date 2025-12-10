classdef MainTest < matlab.unittest.TestCase
    % Minimal test suite for main.m
    
    properties
        ProjectRoot
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
            
            % Suppress warnings
            warning('off', 'all');
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
                testCase.verifyTrue(isfield(Rocket, 'motor_state'), 'Rocket should have motor_state field');
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
            for i = 1:length(possiblePaths)
                if exist(fullfile(possiblePaths{i}, 'main.m'), 'file') == 2
                    found = true;
                    break;
                end
            end
            
            testCase.verifyTrue(found, 'main.m should exist in the project');
        end
    end
end