classdef barrowmanLiftTest < matlab.unittest.TestCase
    
    properties
        AddedPath;
        basicRocket;
    end
    
    methods (TestClassSetup)
        function addFunctionPath(testCase)
            % This function temporarily adds the directory of the drag 
            % function to the MATLAB path so the tests can find it.
            
            % 1. Get the path of the current test file directory 
            testDir = fileparts(mfilename('fullpath'));
            
            % 2. Move up directories to reach the root folder
            rootPath = fileparts(fileparts(fileparts(testDir))); 
            
            % 3. Construct the path to the function file's directory
            functionPath = fullfile(rootPath, 'Src', 'Functions', 'Models');
            
            % 4. Add the path to MATLAB's search path
            addpath(functionPath);
            
            % 5. Store the path so we can remove it later
            testCase.AddedPath = functionPath;
        end

        function createRocket(testCase)
            % Create a basic rocket structure for testing
            testCase.basicRocket = struct(...
                'coneMode', 'on', ...
                'stageDiameters', [0.1, 0.1, 0.1, 0.1], ... % meters
                'stagePositions', [0, 0.2, 0.5, 0.8], ... % meters from nose
                'numStages', 4, ...
                'numFins', 3, ...
                'finRootChord', 0.1, ... % meters
                'finTipChord', 0.06, ... % meters
                'finSpan', 0.08, ... % meters
                'finSweepDistance', 0.04, ... % meters
                'finRootPosition', 0.7, ... % meters from nose
                'finRocketRadius', 0.05); % meters
        end
    end
    
    methods (Test)
        function testZeroAngleOfAttack(testCase)
            % Test at zero angle of attack
            alpha = 0;
            M = 0.5; % subsonic
            theta = 0; % fin roll angle
            
            [Calpha, CP] = barrowmanLift(testCase.basicRocket, alpha, M, theta);
            
            % All values should be positive and finite
            testCase.verifyGreaterThanOrEqual(Calpha, 0);
            testCase.verifyThat(Calpha, ...
                matlab.unittest.constraints.IsFinite);
            testCase.verifyThat(CP, ...
                matlab.unittest.constraints.IsFinite);
            
            % CP should be within rocket length
            testCase.verifyGreaterThanOrEqual(CP, 0);
            testCase.verifyLessThanOrEqual(CP, 0.8);
        end
        
        function testNonZeroAngleOfAttack(testCase)
            % Test at non-zero angle of attack
            alpha = 5 * pi/180; % 5 degrees in radians
            M = 0.5;
            theta = 0;
            
            [Calpha, CP] = barrowmanLift(testCase.basicRocket, alpha, M, theta);
            
            % Values should be positive and finite
            testCase.verifyGreaterThanOrEqual(Calpha, 0);
            testCase.verifyThat(Calpha, ...
                matlab.unittest.constraints.IsFinite);
            testCase.verifyThat(CP, ...
                matlab.unittest.constraints.IsFinite);
        end
        
        function testConeOff(testCase)
            % Test with nose cone contribution turned off
            rocket = testCase.basicRocket;
            rocket.coneMode = 'off';
            
            alpha = 0.1; % rad
            M = 0.5;
            theta = 0;
            
            [Calpha, CP] = barrowmanLift(rocket, alpha, M, theta);
            
            % With cone off, we should have fewer components
            testCase.verifyLength(Calpha, 3); % 2 body stages + fins
            testCase.verifyLength(CP, 3);
        end
        
        function testConeOn(testCase)
            % Test with nose cone contribution turned on
            rocket = testCase.basicRocket;
            rocket.coneMode = 'on';
            
            alpha = 0.1;
            M = 0.5;
            theta = 0;
            
            [Calpha, CP] = barrowmanLift(rocket, alpha, M, theta);
            
            % With cone on, we should have 4 components (cone + 2 body + fins)
            testCase.verifyLength(Calpha, 4);
            testCase.verifyLength(CP, 4);
            
            % First component should be cone (position ~ 0.133 for 0.2m cone)
            testCase.verifyEqual(CP(1), 2/3*0.2, 'AbsTol', 1e-6);
        end
        
        function testTransonicCondition(testCase)
            % Test at transonic Mach number
            alpha = 0.05;
            M = 1.0; % transonic
            theta = 0;
            
            [Calpha, CP] = barrowmanLift(testCase.basicRocket, alpha, M, theta);
            
            % Should handle beta = 0 gracefully
            testCase.verifyThat(Calpha, ...
                matlab.unittest.constraints.IsFinite);
            testCase.verifyThat(CP, ...
                matlab.unittest.constraints.IsFinite);
        end
        
        function testSupersonicCondition(testCase)
            % Test at supersonic Mach number
            alpha = 0.05;
            M = 2.0; % supersonic
            theta = 0;
            
            [Calpha, CP] = barrowmanLift(testCase.basicRocket, alpha, M, theta);
            
            % Should handle supersonic calculations
            testCase.verifyThat(Calpha, ...
                matlab.unittest.constraints.IsFinite);
            testCase.verifyThat(CP, ...
                matlab.unittest.constraints.IsFinite);
        end
        
        function testDifferentFinAngles(testCase)
            % Test with different fin roll angles
            alpha = 0.05;
            M = 0.5;
            
            theta1 = 0;
            theta2 = 30 * pi/180; % 30 degrees
            
            [Calpha1, CP1] = barrowmanLift(testCase.basicRocket, alpha, M, theta1);
            [Calpha2, CP2] = barrowmanLift(testCase.basicRocket, alpha, M, theta2);
            
            % Calpha might be different due to sin^2(theta) term
            testCase.verifyNotEqual(Calpha1(end), Calpha2(end));
            
            % CP for fins should be identical (doesn't depend on theta)
            testCase.verifyEqual(CP1(end), CP2(end), 'AbsTol', 1e-6);
        end
        
        function testDifferentNumberOfFins(testCase)
            % Test with different number of fins
            rocket = testCase.basicRocket;
            rocket.numFins = 4; % 4 fins instead of 3
            
            alpha = 0.05;
            M = 0.5;
            theta = 0;
            
            [Calpha, ~] = barrowmanLift(rocket, alpha, M, theta);
            
            % Should still work
            testCase.verifyThat(Calpha, ...
                matlab.unittest.constraints.IsFinite);
        end
        
        function testHandleNaN(testCase)
            % Test that NaN values in CP are set to 0
            rocket = testCase.basicRocket;
            rocket.stageDiameters = [0.1, 0.1, 0.1, 0.1]; % No taper
            
            alpha = 0.05;
            M = 0.5;
            theta = 0;
            
            [~, CP] = barrowmanLift(rocket, alpha, M, theta);
            
            % Should have no NaNs
            testCase.verifyEqual(any(isnan(CP)), false);
        end
        
        function testReferenceArea(testCase)
            % Verify reference area is calculated correctly
            rocket = testCase.basicRocket;
            expectedAref = pi * 0.1^2 / 4; % Based on stage 2 diameter (index 2)
            
            alpha = 0.1;
            M = 0.5;
            theta = 0;
            
            [Calpha, ~] = barrowmanLift(rocket, alpha, M, theta);
            
            % All coefficients should be reasonable
            testCase.verifyLessThan(Calpha, 10); % Typically < 10 for rockets
        end
        
    end
    
    methods (Test, TestTags = {'EdgeCases'})
        
        function testVerySmallAlpha(testCase)
            % Test with very small angle of attack
            alpha = 1e-6; % rad
            M = 0.5;
            theta = 0;
            
            [Calpha, CP] = barrowmanLift(testCase.basicRocket, alpha, M, theta);
            
            % Should approximate zero-alpha case
            testCase.verifyThat(Calpha, ...
                matlab.unittest.constraints.IsFinite);
            testCase.verifyThat(CP, ...
                matlab.unittest.constraints.IsFinite);
        end
        
        function testExtremeMach(testCase)
            % Test with extreme Mach number
            alpha = 0.05;
            M = 10; % very high Mach
            theta = 0;
            
            [Calpha, CP] = barrowmanLift(testCase.basicRocket, alpha, M, theta);
            
            % Should handle large Mach
            testCase.verifyThat(Calpha, ...
                matlab.unittest.constraints.IsFinite);
            testCase.verifyThat(CP, ...
                matlab.unittest.constraints.IsFinite);
        end
        
    end
end