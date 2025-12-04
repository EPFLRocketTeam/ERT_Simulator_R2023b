classdef rocketReaderTest < matlab.unittest.TestCase
    % Test class for the rocketReader function.
    % To run, type 'runtests('rocketReaderTest')' in the Command Window.

    properties (Access = private)
        AddedPath
        TestDataDir
        OriginalMotor2RocketReader
        OriginalMassProperties
    end
    
    methods (TestClassSetup)
        function setupTest(testCase)
            % This function temporarily adds the directory of the rocketReader 
            % function to the MATLAB path so the tests can find it.
            
            % 1. Get the path of the current test file directory 
            testDir = fileparts(mfilename('fullpath'));
            
            % 2. Move up directories to reach the root folder
            rootPath = fileparts(fileparts(fileparts(testDir))); 
            
            % 3. Construct the path to the function file's directory
            functionPath = fullfile(rootPath, 'Src', 'Functions', 'Utilities');
            
            % 4. Add the path to MATLAB's search path
            addpath(functionPath);
            testCase.AddedPath = functionPath;
            
            % 5. Create test data directory
            testCase.TestDataDir = testDir;
        end
    end
    
    methods (TestClassTeardown)
        function teardownTest(testCase)
            % This removes the path added in TestClassSetup
            rmpath(testCase.AddedPath);
        end
    end
    
    methods (TestMethodSetup)
        function setupMethod(testCase)
            % Save original functions and create mocks
            testCase.OriginalMotor2RocketReader = which('motor2RocketReader');
            testCase.OriginalMassProperties = which('Mass_Properties');
            
            % Get the path where rocketReader is located
            rocketReaderPath = which('rocketReader');
            [funcPath, ~, ~] = fileparts(rocketReaderPath);
            
            % Create a simple mock for motor2RocketReader that just returns Rocket unchanged
            mockFile = fullfile(funcPath, 'motor2RocketReader.m');
            fid = fopen(mockFile, 'w');
            fprintf(fid, 'function Rocket = motor2RocketReader(motorId, Rocket)\n');
            fprintf(fid, '    %% Mock function for testing - just return Rocket unchanged\n');
            fprintf(fid, '    %% Do nothing\n');
            fprintf(fid, 'end\n');
            fclose(fid);
            
            % Create a simple mock for Mass_Properties
            mockFile2 = fullfile(funcPath, 'Mass_Properties.m');
            fid = fopen(mockFile2, 'w');
            fprintf(fid, 'function [mass, cm, inertia, cp, longInertia, radInertia, Ixx, Iyy] = Mass_Properties(time, Rocket, mode)\n');
            fprintf(fid, '    %% Mock function for testing\n');
            fprintf(fid, '    longInertia = 1.0;\n');
            fprintf(fid, '    radInertia = 0.1;\n');
            fprintf(fid, '    mass = 0;\n');
            fprintf(fid, '    cm = 0;\n');
            fprintf(fid, '    inertia = 0;\n');
            fprintf(fid, '    cp = 0;\n');
            fprintf(fid, '    Ixx = 0;\n');
            fprintf(fid, '    Iyy = 0;\n');
            fprintf(fid, 'end\n');
            fclose(fid);
        end
    end
    
    methods (TestMethodTeardown)
        function teardownMethod(testCase)
            % Restore original functions
            rocketReaderPath = which('rocketReader');
            [funcPath, ~, ~] = fileparts(rocketReaderPath);
            
            % Remove mock files
            mockFile = fullfile(funcPath, 'motor2RocketReader.m');
            if exist(mockFile, 'file')
                delete(mockFile);
            end
            
            mockFile2 = fullfile(funcPath, 'Mass_Properties.m');
            if exist(mockFile2, 'file')
                delete(mockFile2);
            end
        end
    end

    methods (Test)
        
        function testBasicRocketFile(testCase)
            % Test reading a basic rocket configuration file
            
            % Create a test rocket file
            rocketFile = fullfile(testCase.TestDataDir, 'testBasicRocket.txt');
            
            % Define test data - minimal required fields
            testData = {
                'numStages 2'
                'stageDiameters 0 0.1'
                'stagePositions 0 1.0'
                'coneMode on'
                'numFins 4'
                'finRootPosition 0.6'
                'finSpan 0.05'
                'finRootChord 0.1'
                'finTipChord 0.05'
                'finThickness 0.003'
                'finSweepDistance 0.01'
                'numLaunchLugs 2'
                'lugSurfaceArea 0.0005'
                'emptyMass 10.0'
                'emptyInertia 0.3'
                'emptyCenterOfMass 0.5'
                'motorId testMotor.txt'
                'motorThrustFactor 1.0'
            };
            
            % Write test file
            fid = fopen(rocketFile, 'w');
            for i = 1:length(testData)
                fprintf(fid, '%s\n', testData{i});
            end
            fclose(fid);
            
            try
                % Read the rocket file
                Rocket = rocketReader(rocketFile);
                
                % Verify basic properties were read correctly
                testCase.verifyEqual(Rocket.numStages, 2);
                testCase.verifyEqual(Rocket.stageDiameters, [0 0.1]);
                testCase.verifyEqual(Rocket.stagePositions, [0 1.0]);
                testCase.verifyEqual(Rocket.coneMode, 'on');
                testCase.verifyEqual(Rocket.numFins, 4);
                testCase.verifyEqual(Rocket.finRootPosition, 0.6);
                testCase.verifyEqual(Rocket.finSpan, 0.05);
                testCase.verifyEqual(Rocket.finRootChord, 0.1);
                testCase.verifyEqual(Rocket.finTipChord, 0.05);
                testCase.verifyEqual(Rocket.finThickness, 0.003);
                testCase.verifyEqual(Rocket.finSweepDistance, 0.01);
                testCase.verifyEqual(Rocket.numLaunchLugs, 2);
                testCase.verifyEqual(Rocket.lugSurfaceArea, 0.0005);
                testCase.verifyEqual(Rocket.emptyMass, 10.0);
                testCase.verifyEqual(Rocket.emptyInertia, 0.3);
                testCase.verifyEqual(Rocket.emptyCenterOfMass, 0.5);
                testCase.verifyEqual(Rocket.motorId, 'testMotor.txt');
                testCase.verifyEqual(Rocket.motorThrustFactor, 1.0);
                
                % Verify derived parameters
                testCase.verifyEqual(Rocket.maxDiameter, 0.1);
                testCase.verifyEqual(Rocket.meanFinChord, (0.1 + 0.05)/2);
                testCase.verifyEqual(Rocket.totalLength, 1.0);
                
            catch ME
                % Clean up test file
                if exist(rocketFile, 'file')
                    delete(rocketFile);
                end
                rethrow(ME);
            end
            
            % Clean up test file
            if exist(rocketFile, 'file')
                delete(rocketFile);
            end
        end
        
        function testCompleteRocketFile(testCase)
            % Test reading a complete rocket configuration file
            
            % Create a test rocket file
            rocketFile = fullfile(testCase.TestDataDir, 'testCompleteRocket.txt');
            
            % Define test data - all fields
            testData = {
                'numStages 4'
                'stageDiameters 0 0.12 0.12 0.09'
                'stagePositions 0 0.6 1.8 2.2'
                'coneMode off'
                'numFins 3'
                'finRootPosition 1.4'
                'finSpan 0.1'
                'finRootChord 0.18'
                'finTipChord 0.09'
                'finThickness 0.004'
                'finSweepDistance 0.025'
                'finLeadingEdgeLength 0.19'
                'finTrailingEdgeLength 0.17'
                'numLaunchLugs 1'
                'lugSurfaceArea 0.0003'
                'emptyMass 12.5'
                'emptyInertia 0.45'
                'emptyCenterOfMass 1.0'
                'airbrakePosition 1.6'
                'numAirbrakes 4'
                'airbrakeAngle 30'
                'motorId CustomMotor_2024.txt'
                'motorThrustFactor 0.9'
                'payloadMass 1.8'
                'mainParachuteDragArea 0.6'
                'drogueParachuteDragArea 0.25'
                'mainParachuteDeploymentAltitude 600'
                'centerOfPressureFactor 1.1'
                'normalForceCoefficientFactor 0.95'
                'dragCoefficientFactor 1.05'
            };
            
            % Write test file
            fid = fopen(rocketFile, 'w');
            for i = 1:length(testData)
                fprintf(fid, '%s\n', testData{i});
            end
            fclose(fid);
            
            try
                % Read the rocket file
                Rocket = rocketReader(rocketFile);
                
                % Verify all parameters were read correctly
                testCase.verifyEqual(Rocket.numStages, 4);
                testCase.verifyEqual(Rocket.stageDiameters, [0 0.12 0.12 0.09]);
                testCase.verifyEqual(Rocket.stagePositions, [0 0.6 1.8 2.2]);
                testCase.verifyEqual(Rocket.coneMode, 'off');
                testCase.verifyEqual(Rocket.numFins, 3);
                testCase.verifyEqual(Rocket.finRootPosition, 1.4);
                testCase.verifyEqual(Rocket.finSpan, 0.1);
                testCase.verifyEqual(Rocket.finRootChord, 0.18);
                testCase.verifyEqual(Rocket.finTipChord, 0.09);
                testCase.verifyEqual(Rocket.finThickness, 0.004);
                testCase.verifyEqual(Rocket.finSweepDistance, 0.025);
                testCase.verifyEqual(Rocket.finLeadingEdgeLength, 0.19);
                testCase.verifyEqual(Rocket.finTrailingEdgeLength, 0.17);
                testCase.verifyEqual(Rocket.numLaunchLugs, 1);
                testCase.verifyEqual(Rocket.lugSurfaceArea, 0.0003);
                testCase.verifyEqual(Rocket.emptyMass, 12.5);
                testCase.verifyEqual(Rocket.emptyInertia, 0.45);
                testCase.verifyEqual(Rocket.emptyCenterOfMass, 1.0);
                testCase.verifyEqual(Rocket.airbrakePosition, 1.6);
                testCase.verifyEqual(Rocket.numAirbrakes, 4);
                testCase.verifyEqual(Rocket.airbrakeAngle, 30);
                testCase.verifyEqual(Rocket.motorId, 'CustomMotor_2024.txt');
                testCase.verifyEqual(Rocket.motorThrustFactor, 0.9);
                testCase.verifyEqual(Rocket.payloadMass, 1.8);
                testCase.verifyEqual(Rocket.mainParachuteDragArea, 0.6);
                testCase.verifyEqual(Rocket.drogueParachuteDragArea, 0.25);
                testCase.verifyEqual(Rocket.mainParachuteDeploymentAltitude, 600);
                testCase.verifyEqual(Rocket.centerOfPressureFactor, 1.1);
                testCase.verifyEqual(Rocket.normalForceCoefficientFactor, 0.95);
                testCase.verifyEqual(Rocket.dragCoefficientFactor, 1.05);
                
                % Verify derived parameters
                testCase.verifyEqual(Rocket.maxDiameter, 0.12);
                testCase.verifyEqual(Rocket.meanFinChord, (0.18 + 0.09)/2);
                testCase.verifyEqual(Rocket.totalLength, 2.2);
                
            catch ME
                % Clean up test file
                if exist(rocketFile, 'file')
                    delete(rocketFile);
                end
                rethrow(ME);
            end
            
            % Clean up test file
            if exist(rocketFile, 'file')
                delete(rocketFile);
            end
        end
        
        function testInvalidConeModeError(testCase)
            % Test that invalid coneMode throws an error
            
            % Create a test rocket file
            rocketFile = fullfile(testCase.TestDataDir, 'testInvalidCone.txt');
            
            % Define test data with invalid coneMode
            testData = {
                'numStages 2'
                'stageDiameters 0 0.1'
                'stagePositions 0 1.0'
                'coneMode maybe'  % Invalid value
                'numFins 4'
                'finRootPosition 0.6'
                'finSpan 0.05'
                'finRootChord 0.1'
                'finTipChord 0.05'
                'finThickness 0.003'
                'numLaunchLugs 2'
                'lugSurfaceArea 0.0005'
                'emptyMass 10.0'
                'emptyInertia 0.3'
                'emptyCenterOfMass 0.5'
                'motorId testMotor.txt'
                'motorThrustFactor 1.0'
            };
            
            % Write test file
            fid = fopen(rocketFile, 'w');
            for i = 1:length(testData)
                fprintf(fid, '%s\n', testData{i});
            end
            fclose(fid);
            
            try
                % Verify that an error is thrown
                testCase.verifyError(@() rocketReader(rocketFile), '');
                
            catch ME
                % Clean up test file
                if exist(rocketFile, 'file')
                    delete(rocketFile);
                end
                rethrow(ME);
            end
            
            % Clean up test file
            if exist(rocketFile, 'file')
                delete(rocketFile);
            end
        end
        
        function testFileNotFoundError(testCase)
            % Test error handling for non-existent file
            
            nonExistentFile = fullfile(testCase.TestDataDir, 'nonExistentFile.txt');
            
            % Verify that an error is thrown with the custom error message
            try
                rocketReader(nonExistentFile);
                testCase.verifyFail('Expected an error for non-existent file');
            catch ME
                % Verify the error contains the expected message
                testCase.verifySubstring(ME.message, 'ERROR: Rocket file not found');
            end
        end
        
        function testUnknownIdentifierWarning(testCase)
            % Test that unknown identifiers generate warnings
            
            rocketFile = fullfile(testCase.TestDataDir, 'testUnknownId.txt');
            
            % Define test data with an unknown identifier
            testData = {
                'numStages 2'
                'stageDiameters 0 0.1'
                'stagePositions 0 1.0'
                'coneMode on'
                'numFins 4'
                'finRootPosition 0.6'
                'finSpan 0.05'
                'finRootChord 0.1'
                'finTipChord 0.05'
                'finThickness 0.003'
                'unknownIdentifier someValue'  % This should generate a warning
                'numLaunchLugs 2'
                'lugSurfaceArea 0.0005'
                'emptyMass 10.0'
                'emptyInertia 0.3'
                'emptyCenterOfMass 0.5'
                'motorId testMotor.txt'
                'motorThrustFactor 1.0'
            };
            
            fid = fopen(rocketFile, 'w');
            for i = 1:length(testData)
                fprintf(fid, '%s\n', testData{i});
            end
            fclose(fid);
            
            try
                % Verify that a warning is issued
                testCase.verifyWarning(@() rocketReader(rocketFile), '');
                
            catch ME
                if exist(rocketFile, 'file')
                    delete(rocketFile);
                end
                rethrow(ME);
            end
            
            % Clean up
            if exist(rocketFile, 'file')
                delete(rocketFile);
            end
        end
        
        function testHybridRocketConfiguration(testCase)
            % Test reading a hybrid rocket configuration
            
            rocketFile = fullfile(testCase.TestDataDir, 'testHybridRocket.txt');
            
            testData = {
                'numStages 2'
                'stageDiameters 0 0.1'
                'stagePositions 0 1.0'
                'coneMode on'
                'numFins 4'
                'finRootPosition 0.6'
                'finSpan 0.05'
                'finRootChord 0.1'
                'finTipChord 0.05'
                'finThickness 0.003'
                'numLaunchLugs 2'
                'lugSurfaceArea 0.0005'
                'emptyMass 10.0'
                'emptyInertia 0.3'
                'emptyCenterOfMass 0.5'
                'motorId testMotor.txt'
                'hybrid fuelTank 0.2'
                'motorThrustFactor 1.0'
            };
            
            fid = fopen(rocketFile, 'w');
            for i = 1:length(testData)
                fprintf(fid, '%s\n', testData{i});
            end
            fclose(fid);
            
            try
                % Read the rocket file
                Rocket = rocketReader(rocketFile);
                
                % Verify hybrid properties
                testCase.verifyTrue(Rocket.isHybrid);
                testCase.verifyEqual(Rocket.fuelTankId, 'fuelTank');
                testCase.verifyEqual(Rocket.interMotorDistance, 0.2);
                
            catch ME
                if exist(rocketFile, 'file')
                    delete(rocketFile);
                end
                rethrow(ME);
            end
            
            % Clean up
            if exist(rocketFile, 'file')
                delete(rocketFile);
            end
        end
        
        function testOptionalFields(testCase)
            % Test reading optional fields
            
            rocketFile = fullfile(testCase.TestDataDir, 'testOptionalFields.txt');
            
            testData = {
                'numStages 2'
                'stageDiameters 0 0.1'
                'stagePositions 0 1.0'
                'coneMode on'
                'numFins 4'
                'finRootPosition 0.6'
                'finSpan 0.05'
                'finRootChord 0.1'
                'finTipChord 0.05'
                'finThickness 0.003'
                'numLaunchLugs 2'
                'lugSurfaceArea 0.0005'
                'emptyMass 10.0'
                'emptyInertia 0.3'
                'emptyCenterOfMass 0.5'
                'motorId testMotor.txt'
                'motorThrustFactor 1.0'
                'inertiaMatrix 0.5 0 0 0 0.5 0 0 0 0.1'
                'tankLength 0.5'
                'tankRadius 0.05'
                'tankPosition 0.8'
            };
            
            fid = fopen(rocketFile, 'w');
            for i = 1:length(testData)
                fprintf(fid, '%s\n', testData{i});
            end
            fclose(fid);
            
            try
                % Read the rocket file
                Rocket = rocketReader(rocketFile);
                
                % Verify optional fields
                expectedInertiaMatrix = [0.5 0 0; 0 0.5 0; 0 0 0.1];
                testCase.verifyEqual(Rocket.inertiaMatrix, expectedInertiaMatrix, 'AbsTol', 1e-10);
                testCase.verifyEqual(Rocket.tankLength, 0.5);
                testCase.verifyEqual(Rocket.tankRadius, 0.05);
                testCase.verifyEqual(Rocket.tankPosition, 0.8);
                
            catch ME
                if exist(rocketFile, 'file')
                    delete(rocketFile);
                end
                rethrow(ME);
            end
            
            % Clean up
            if exist(rocketFile, 'file')
                delete(rocketFile);
            end
        end
    end
end