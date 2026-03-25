%% File: normalLiftTest.m
classdef normalLiftTest < matlab.unittest.TestCase
    
    properties
        AddedPath;
        testRocket;
        testAlpha;
        testK;
        testM;
        testTheta;
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
    end
    
    methods (TestClassTeardown)
        function removeFunctionPath(testCase)
            % This removes the path added in TestClassSetup
            rmpath(testCase.AddedPath);
        end
    end

    methods (TestMethodSetup)
        function createRocket(testCase)
            % Create a standard test rocket with typical parameters
            testCase.testRocket = struct(...
                'coneMode', 'on', ...
                'stageDiameters', [0.1, 0.1, 0.08, 0.06], ...
                'stagePositions', [0, 0.2, 0.5, 0.8], ...
                'numStages', 4, ...
                'numFins', 3, ...
                'finRootChord', 0.1, ...
                'finTipChord', 0.06, ...
                'finSpan', 0.08, ...
                'finSweepDistance', 0.04, ...
                'finRootPosition', 0.7, ...
                'finRocketRadius', 0.05, ...
                'centerOfPressureFactor', 1.0, ...
                'normalForceCoefficientFactor', 1.0);
            
            testCase.testAlpha = 5 * pi/180; % 5 degrees
            testCase.testK = 1.0; % Nominal correction factor
            testCase.testM = 0.5; % Subsonic
            testCase.testTheta = 0; % No roll
        end
    end
    
    methods (Test)
        function testBasicFunctionality(testCase)
            % Test basic functionality without Galejs correction
            Galejs = 0;
            
            [CNa, Xp, CNa_barrowman, Xp_barrowman] = normalLift(...
                testCase.testRocket, testCase.testAlpha, testCase.testK, ...
                testCase.testM, testCase.testTheta, Galejs);
            
            % Verify outputs are finite and positive
            testCase.verifyGreaterThan(CNa, 0);
            testCase.verifyGreaterThan(Xp, 0);
            testCase.verifyThat(CNa, matlab.unittest.constraints.IsFinite);
            testCase.verifyThat(Xp, matlab.unittest.constraints.IsFinite);
            
            % Barrowman outputs should match structure
            testCase.verifyGreaterThan(sum(CNa_barrowman), 0);
            testCase.verifyGreaterThan(sum(Xp_barrowman), 0);
            
            % Without Galejs, CNa should equal sum of barrowman
            testCase.verifyEqual(CNa, sum(CNa_barrowman), 'AbsTol', 1e-10);
        end
        
        function testWithGalejsCorrection(testCase)
            % Test with Galejs correction enabled
            Galejs = 1;
            
            [CNa, Xp, CNa_barrowman, ~] = normalLift(...
                testCase.testRocket, testCase.testAlpha, testCase.testK, ...
                testCase.testM, testCase.testTheta, Galejs);
            
            % Verify outputs are finite
            testCase.verifyGreaterThan(CNa, 0);
            testCase.verifyGreaterThan(Xp, 0);
            testCase.verifyThat(CNa, matlab.unittest.constraints.IsFinite);
            testCase.verifyThat(Xp, matlab.unittest.constraints.IsFinite);
            
            % With Galejs, CNa should be different from pure Barrowman
            CNa_pure = sum(CNa_barrowman);
            testCase.verifyNotEqual(CNa, CNa_pure);
        end
        
        function testGalejsCorrectionFactor(testCase)
            % Test effect of different K factors with Galejs correction
            Galejs = 1;
            
            K_values = [0.5, 1.0, 1.5, 2.0];
            CNa_results = zeros(size(K_values));
            
            for i = 1:length(K_values)
                [CNa_results(i), ~, ~, ~] = normalLift(...
                    testCase.testRocket, testCase.testAlpha, K_values(i), ...
                    testCase.testM, testCase.testTheta, Galejs);
            end
            
            % Higher K should give different results
            testCase.verifyNotEqual(CNa_results(1), CNa_results(2));
            testCase.verifyNotEqual(CNa_results(2), CNa_results(3));
        end
        
        function testAngleOfAttackEffect(testCase)
            % Test effect of different angles of attack
            Galejs = 0;
            
            alpha_values = [0, 2, 5, 10, 15] * pi/180;
            CNa_results = zeros(size(alpha_values));
            
            for i = 1:length(alpha_values)
                [CNa_results(i), ~, ~, ~] = normalLift(...
                    testCase.testRocket, alpha_values(i), testCase.testK, ...
                    testCase.testM, testCase.testTheta, Galejs);
            end
            
            % CNa should vary with alpha
            testCase.verifyNotEqual(CNa_results(1), CNa_results(2));
            
            % At alpha=0, should be well-defined
            testCase.verifyThat(CNa_results(1), ...
                matlab.unittest.constraints.IsFinite);
        end
        
        function testMachEffect(testCase)
            % Test effect of different Mach numbers
            Galejs = 0;
            
            M_values = [0.3, 0.8, 1.2, 2.0];
            CNa_results = zeros(size(M_values));
            
            for i = 1:length(M_values)
                [CNa_results(i), ~, ~, ~] = normalLift(...
                    testCase.testRocket, testCase.testAlpha, testCase.testK, ...
                    M_values(i), testCase.testTheta, Galejs);
            end
            
            % Results should vary with Mach
            testCase.verifyNotEqual(CNa_results(1), CNa_results(2));
            testCase.verifyNotEqual(CNa_results(2), CNa_results(3));
        end
        
        function testConeOnOff(testCase)
            % Test with cone on vs off
            Galejs = 0;
            
            % Cone on
            [CNa_on, Xp_on, CNa_barrowman_on, ~] = normalLift(...
                testCase.testRocket, testCase.testAlpha, testCase.testK, ...
                testCase.testM, testCase.testTheta, Galejs);
            
            % Cone off
            rocket_no_cone = testCase.testRocket;
            rocket_no_cone.coneMode = 'off';
            
            [CNa_off, Xp_off, CNa_barrowman_off, ~] = normalLift(...
                rocket_no_cone, testCase.testAlpha, testCase.testK, ...
                testCase.testM, testCase.testTheta, Galejs);
            
            % Cone on should give higher CNa
            testCase.verifyGreaterThan(CNa_on, CNa_off);
            
            % Xp should be different (cone CP is forward)
            testCase.verifyNotEqual(Xp_on, Xp_off);
            
            % Barrowman outputs should have different lengths
            testCase.verifyLength(CNa_barrowman_on, 4); % cone + 2 body + fins
            testCase.verifyLength(CNa_barrowman_off, 3); % 2 body + fins
        end
        
    end
    
    methods (Test, TestTags = {'WeightedAverage'})
        
        function testWeightedAverageCalculation(testCase)
            % Test that center of pressure is correctly calculated as weighted average
            Galejs = 0;
            
            [~, Xp, CNa_barrowman, Xp_barrowman] = normalLift(...
                testCase.testRocket, testCase.testAlpha, testCase.testK, ...
                testCase.testM, testCase.testTheta, Galejs);
            
            % Manual weighted average calculation
            expected_Xp = sum(CNa_barrowman .* Xp_barrowman) / sum(CNa_barrowman);
            
            testCase.verifyEqual(Xp, expected_Xp, 'AbsTol', 1e-10);
        end
        
        function testWeightedAverageWithGalejs(testCase)
            % Test weighted average with Galejs correction
            Galejs = 1;
            
            % We need to mock robertGalejsLift for this test
            % Since we can't easily mock, we'll test the calculation logic
            % by checking that Xp is between component CPs
            
            [CNa, Xp, CNa_barrowman, ~] = normalLift(...
                testCase.testRocket, testCase.testAlpha, testCase.testK, ...
                testCase.testM, testCase.testTheta, Galejs);
            
            % Xp should be finite
            testCase.verifyThat(Xp, matlab.unittest.constraints.IsFinite);
            
            % CNa should be sum of all components
            testCase.verifyGreaterThan(CNa, sum(CNa_barrowman));
        end
        
    end
    
    methods (Test, TestTags = {'EdgeCases'})
        
        function testZeroAngleOfAttack(testCase)
            % Test with zero angle of attack
            Galejs = 0;
            
            [CNa, Xp, ~, ~] = normalLift(...
                testCase.testRocket, 0, testCase.testK, ...
                testCase.testM, testCase.testTheta, Galejs);
            
            % Should handle zero alpha
            testCase.verifyThat(CNa, matlab.unittest.constraints.IsFinite);
            testCase.verifyThat(Xp, matlab.unittest.constraints.IsFinite);
            testCase.verifyGreaterThan(CNa, 0);
        end
        
        function testZeroLiftComponents(testCase)
            % Test when some components have zero lift
            rocket_zero = testCase.testRocket;
            rocket_zero.stageDiameters = [0.1, 0.1, 0.1, 0.1]; % No taper
            
            Galejs = 0;
            
            [CNa, Xp, ~, ~] = normalLift(...
                rocket_zero, testCase.testAlpha, testCase.testK, ...
                testCase.testM, testCase.testTheta, Galejs);
            
            % Should still work (body contributions may be zero)
            testCase.verifyThat(CNa, matlab.unittest.constraints.IsFinite);
            testCase.verifyThat(Xp, matlab.unittest.constraints.IsFinite);
        end
        
        function testSupersonicMach(testCase)
            % Test at supersonic Mach numbers
            Galejs = 0;
            
            [CNa, Xp, ~, ~] = normalLift(...
                testCase.testRocket, testCase.testAlpha, testCase.testK, ...
                2.5, testCase.testTheta, Galejs);
            
            % Should handle supersonic flow
            testCase.verifyThat(CNa, matlab.unittest.constraints.IsFinite);
            testCase.verifyThat(Xp, matlab.unittest.constraints.IsFinite);
        end
        
        function testExtremeRollAngle(testCase)
            % Test with extreme roll angles
            Galejs = 0;
            
            theta_values = [-90, 0, 90, 180] * pi/180;
            
            for i = 1:length(theta_values)
                [CNa, Xp, ~, ~] = normalLift(...
                    testCase.testRocket, testCase.testAlpha, testCase.testK, ...
                    testCase.testM, theta_values(i), Galejs);
                
                testCase.verifyThat(CNa, matlab.unittest.constraints.IsFinite);
                testCase.verifyThat(Xp, matlab.unittest.constraints.IsFinite);
            end
        end
        
        function testZeroFins(testCase)
            % Test with zero fins (should still work)
            rocket_no_fins = testCase.testRocket;
            rocket_no_fins.numFins = 0;
            
            Galejs = 0;
            
            [CNa, Xp, ~, ~] = normalLift(...
                rocket_no_fins, testCase.testAlpha, testCase.testK, ...
                testCase.testM, testCase.testTheta, Galejs);
            
            % Should still have lift from body/cone
            testCase.verifyGreaterThan(CNa, 0);
            testCase.verifyThat(Xp, matlab.unittest.constraints.IsFinite);
        end
        
    end
    
    methods (Test, TestTags = {'Validation'})
        
        function testConsistencyWithBarrowman(testCase)
            % Test that Barrowman outputs are consistent
            Galejs = 0;
            
            [~, ~, CNa_barrowman, Xp_barrowman] = normalLift(...
                testCase.testRocket, testCase.testAlpha, testCase.testK, ...
                testCase.testM, testCase.testTheta, Galejs);
            
            % Direct call to barrowmanLift for comparison
            [CNa_direct, Xp_direct] = barrowmanLift(...
                testCase.testRocket, testCase.testAlpha, testCase.testM, ...
                testCase.testTheta);
            
            % Should match
            testCase.verifyEqual(CNa_barrowman, CNa_direct, 'AbsTol', 1e-10);
            testCase.verifyEqual(Xp_barrowman, Xp_direct, 'AbsTol', 1e-10);
        end
        
        function testOutputLengths(testCase)
            % Test that output arrays have correct lengths
            Galejs = 0;
            
            [~, ~, CNa_barrowman, Xp_barrowman] = normalLift(...
                testCase.testRocket, testCase.testAlpha, testCase.testK, ...
                testCase.testM, testCase.testTheta, Galejs);
            
            % Should have 4 components (cone + 2 body + fins)
            testCase.verifyLength(CNa_barrowman, 4);
            testCase.verifyLength(Xp_barrowman, 4);
            
            % Individual values should be positive
            testCase.verifyGreaterThan(Xp_barrowman, 0);
        end
        
    end
end