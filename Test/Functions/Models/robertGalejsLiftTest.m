classdef robertGalejsLiftTest < matlab.unittest.TestCase
    
    properties
        AddedPath;
        testRocket;
        tolerance = 1e-10;
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

        function createTestRocket(testCase)
            % Create a standard test rocket structure
            testCase.testRocket = struct();
            testCase.testRocket.coneMode = 'off';
            testCase.testRocket.numStages = 3;
            testCase.testRocket.stagePositions = [0, 1, 2];
            testCase.testRocket.stageDiameters = [0.1, 0.2, 0.1];
        end
    end
    
    methods (Test)
        
        function testBasicFunctionality(testCase)
            % Test basic functionality with coneMode off
            rocket = testCase.testRocket;
            angleOfAttack = 0.1;
            galejsFactor = 1.0;
            
            [Calpha2, centerOfPressure] = robertGalejsLift(rocket, angleOfAttack, galejsFactor);
            
            % Verify outputs are not empty
            testCase.assertNotEmpty(Calpha2);
            testCase.assertNotEmpty(centerOfPressure);
            
            % Verify correct number of outputs
            testCase.verifyEqual(length(centerOfPressure), rocket.numStages-2);
        end
        
        function testWithConeModeOn(testCase)
            % Test with cone mode enabled
            rocket = testCase.testRocket;
            rocket.coneMode = 'on';
            angleOfAttack = 0.1;
            galejsFactor = 1.0;
            
            [~, centerOfPressure] = robertGalejsLift(rocket, angleOfAttack, galejsFactor);
            
            % Should have one more element with cone mode on
            testCase.verifyEqual(length(centerOfPressure), rocket.numStages-1);
        end
        
        function testZeroAlpha(testCase)
            % Test with angleOfAttack = 0
            rocket = testCase.testRocket;
            angleOfAttack = 0;
            galejsFactor = 1.0;
            
            [Calpha2, ~] = robertGalejsLift(rocket, angleOfAttack, galejsFactor);
            
            % Calpha2 should be zero when angleOfAttack is zero
            testCase.verifyEqual(Calpha2, 0, 'AbsTol', testCase.tolerance);
        end
        
        function testZeroK(testCase)
            % Test with galejsFactor = 0
            rocket = testCase.testRocket;
            angleOfAttack = 0.1;
            galejsFactor = 0;
            
            [Calpha2, ~] = robertGalejsLift(rocket, angleOfAttack, galejsFactor);
            
            % Calpha2 should be zero when galejsFactor is zero
            testCase.verifyEqual(Calpha2, 0, 'AbsTol', testCase.tolerance);
        end
        
        function testApCalculation(testCase)
            % Verify the Ap calculations are correct
            rocket = testCase.testRocket;
            rocket.numStages = 4;
            rocket.stagePositions = [0, 1, 2, 3];
            rocket.stageDiameters = [0.1, 0.2, 0.15, 0.1];
            angleOfAttack = 0.1;
            galejsFactor = 1.0;
            
            [Calpha2, ~] = robertGalejsLift(rocket, angleOfAttack, galejsFactor);
            
            % Manually calculate expected Ap for first stage transition
            expectedAp1 = (0.2 + 0.15)/2 * (2 - 1);  % (D2 + D3)/2 * (pos3 - pos2)
            expectedAp2 = (0.15 + 0.1)/2 * (3 - 2);  % (D3 + D4)/2 * (pos4 - pos3)
            
            % Calculate expected Calpha2
            expectedCalpha2 = 4/pi/rocket.stageDiameters(2)^2 * galejsFactor * ...
                              angleOfAttack * [expectedAp1, expectedAp2];
            
            testCase.verifyEqual(Calpha2, expectedCalpha2, 'AbsTol', testCase.tolerance);
        end
        
        function testXpCalculation(testCase)
            % Verify the Xp calculations are correct for a simple case
            rocket = testCase.testRocket;
            rocket.numStages = 4;
            rocket.stagePositions = [0, 1, 3, 4];
            rocket.stageDiameters = [0.1, 0.2, 0.1, 0.05];
            angleOfAttack = 0.1;
            galejsFactor = 1.0;
            
            [~, centerOfPressure] = robertGalejsLift(rocket, angleOfAttack, galejsFactor);
            
            % Manually calculate expected Xp for first stage
            D1 = 0.2; D2 = 0.1;
            L = 2; % 3-1
            expectedCenterOfPressure1 = 1 + 1/3 * L * (D1 + 2*D2)/(D1 + D2);
            
            testCase.verifyEqual(centerOfPressure(1), expectedCenterOfPressure1, 'AbsTol', testCase.tolerance);
        end
        
        function testConeModeXpCalculation(testCase)
            % Test Xp calculation with cone mode on
            rocket = testCase.testRocket;
            rocket.coneMode = 'on';
            rocket.numStages = 3;
            rocket.stagePositions = [0, 2, 3];
            rocket.stageDiameters = [0.1, 0.3, 0.1];
            angleOfAttack = 0.1;
            galejsFactor = 1.0;
            
            [~, centerOfPressure] = robertGalejsLift(rocket, angleOfAttack, galejsFactor);
            
            % Expected Xp for cone
            expectedCenterOfPressureCone = 2/3 * rocket.stagePositions(2);
            
            testCase.verifyEqual(centerOfPressure(1), expectedCenterOfPressureCone, 'AbsTol', testCase.tolerance);
        end
        
        function testMultipleStages(testCase)
            % Test with more stages
            rocket = testCase.testRocket;
            rocket.numStages = 5;
            rocket.stagePositions = [0, 0.5, 1.5, 2.5, 3.5];
            rocket.stageDiameters = [0.1, 0.2, 0.15, 0.12, 0.08];
            angleOfAttack = 0.05;
            galejsFactor = 2.5;
            
            [Calpha2, centerOfPressure] = robertGalejsLift(rocket, angleOfAttack, galejsFactor);
            
            % Verify number of outputs
            testCase.verifyEqual(length(centerOfPressure), rocket.numStages-2);
            
            % Verify Calpha2 is positive
            testCase.verifyGreaterThan(Calpha2, 0);
        end
        
        function testLargeValues(testCase)
            % Test with large input values
            rocket = testCase.testRocket;
            rocket.stageDiameters = [1, 10, 1];
            angleOfAttack = 10;
            galejsFactor = 1000;
            
            [Calpha2, ~] = robertGalejsLift(rocket, angleOfAttack, galejsFactor);
            
            % Verify no overflow/underflow issues
            testCase.verifyTrue(isfinite(Calpha2));
            testCase.verifyGreaterThan(Calpha2, 0);
        end
        
        function testSmallValues(testCase)
            % Test with very small input values
            rocket = testCase.testRocket;
            rocket.stageDiameters = [1e-6, 1e-5, 1e-6];
            angleOfAttack = 1e-6;
            galejsFactor = 1e-6;
            
            [Calpha2, ~] = robertGalejsLift(rocket, angleOfAttack, galejsFactor);
            
            % Verify no underflow issues
            testCase.verifyTrue(isfinite(Calpha2));
        end
        
        function testInvalidInputs(testCase)
            % Test error handling for invalid inputs
            rocket = testCase.testRocket;
            
            % Test with empty rocket
            testCase.verifyError(@() robertGalejsLift([], 0.1, 1), 'MATLAB:structRefFromNonStruct');
            
            % Test with negative angleOfAttack
            [Calpha2, ~] = robertGalejsLift(rocket, -0.1, 1);
            testCase.verifyLessThan(Calpha2, 0);
        end
        
        function testNumericPrecision(testCase)
            % Test numeric precision
            rocket = testCase.testRocket;
            rocket.stagePositions = [0, 1/3, 2/3];
            rocket.stageDiameters = [0.1, 0.2, 0.1];
            angleOfAttack = 1/3;
            galejsFactor = 1/3;
            
            [Calpha2, centerOfPressure] = robertGalejsLift(rocket, angleOfAttack, galejsFactor);
            
            % Verify results are numeric and finite
            testCase.verifyTrue(isnumeric(Calpha2));
            testCase.verifyTrue(isnumeric(centerOfPressure));
            testCase.verifyTrue(all(isfinite([Calpha2, centerOfPressure])));
        end
        
        function testConsistencyWithDifferentParameters(testCase)
            % Test consistency when changing parameters
            rocket = testCase.testRocket;
            
            [Calpha2_1, centerOfPressure1] = robertGalejsLift(rocket, 0.1, 1);
            [Calpha2_2, centerOfPressure2] = robertGalejsLift(rocket, 0.2, 1);
            
            % Doubling angleOfAttack should double Calpha2
            testCase.verifyEqual(Calpha2_2, 2*Calpha2_1, 'AbsTol', testCase.tolerance);
            
            % Xp should remain the same
            testCase.verifyEqual(centerOfPressure1, centerOfPressure2, 'AbsTol', testCase.tolerance);
        end
    end
end