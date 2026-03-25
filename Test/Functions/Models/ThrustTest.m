classdef thrustTest < matlab.unittest.TestCase
    % Unit tests for thrust function

    properties
        AddedPath;
        AddedPath2;
        AddedPath3;
        TestData;
    end
    
    properties (TestParameter)
        % Test parameters for different time scenarios
        timeValues = {0, 0.5, 1.5, 2.5, 5.0, 8.0, 9.5, 10.223};
    end
    
    methods (TestClassSetup)
        function addFunctionPath(testCase)
            % This function temporarily adds the directory of the drag 
            % function to the MATLAB path so the tests can find it.
            
            % 1. Get the path of the current test file directory 
            testDir = fileparts(mfilename('fullpath'));
            
            % 2. Move up directories to reach the root folder
            rootPath = fileparts(fileparts(fileparts(testDir))); 
            
            % 3.a Construct the path to the function file's directory
            functionPath = fullfile(rootPath, 'Src', 'Functions', 'Models');
            
            % 4.a Add the path to MATLAB's search path
            addpath(functionPath);
            
            % 5.a Store the path so we can remove it later
            testCase.AddedPath = functionPath;

            % 3.b Construct the path to motorReader.m's directory
            functionPath = fullfile(rootPath, 'Src', 'Functions', 'Utilities');
            
            % 4.b Add the path to MATLAB's search path
            addpath(functionPath);
            
            % 5.b Store the path so we can remove it later
            testCase.AddedPath2 = functionPath;
        end

        function setupTestEnvironment(testCase)
            % Set up the test environment
            % This runs once before all tests
            
            % Load motor data from file
            motorFile = 'motorTestFile.txt';
            [time, thrust, ~] = motorReader(motorFile);
            
            % Create Rocket structure with preprocessed data
            testCase.TestData.rawTime = time;
            testCase.TestData.rawThrust = thrust;
            if time(1) > 0
                time = [0,time];
                thrust = [0,thrust];
            end
            testCase.TestData.Rocket.thrustTime = time;
            testCase.TestData.Rocket.thrustForce = thrust;
            testCase.TestData.Rocket.burnTime = testCase.TestData.Rocket.thrustTime(end);
        end
    end
    
    methods (TestClassTeardown)
        function removeFunctionPath(testCase)
            % This removes the paths added in TestClassSetup
            rmpath(testCase.AddedPath);
            rmpath(testCase.AddedPath2);
        end
    end
    
    methods (Test) 
        function testBasicInterpolation(testCase)
            % Test basic interpolation at various time points
            
            % Test at exact time points from the data
            testTimes = [0, 0.5, 1.0, 2.0, 5.0, 8.0, 10.0];
            
            for t = testTimes
                % Find closest index in raw data
                [~, idx] = min(abs(testCase.TestData.rawTime - t));
                expectedThrust = testCase.TestData.rawThrust(idx);
                
                % Get interpolated thrust
                actualThrust = thrust(t, testCase.TestData.Rocket);
                
                % Should match closely (interpolation might give slightly different values)
                testCase.verifyEqual(actualThrust, expectedThrust, 'RelTol', 0.01, ...
                    sprintf('Thrust at t=%.3f should match data', t));
            end
        end
        
        function testLinearInterpolation(testCase)
            % Test that interpolation is linear between points
            
            % Pick two consecutive points from the data
            t1 = 1.0;
            t2 = 1.001; % Next time point (0.001s spacing)
            
            % Get thrust at these points
            T1 = thrust(t1, testCase.TestData.Rocket);
            T2 = thrust(t2, testCase.TestData.Rocket);
            
            % Test at midpoint
            timeMid = (t1 + t2) / 2;
            thrustMid = thrust(timeMid, testCase.TestData.Rocket);
            
            % Linear interpolation should give average at midpoint
            expectedMid = (T1 + T2) / 2;
            testCase.verifyEqual(thrustMid, expectedMid, 'RelTol', 0.01, ...
                'Interpolation should be linear between points');
        end
        
        function testAfterBurnTime(testCase)
            % Test that thrust is zero after burn time
            burnTime = testCase.TestData.Rocket.burnTime;
            
            % Test at times after burn
            testTimes = [burnTime + 0.001, burnTime + 1, burnTime + 10, burnTime + 100];
            
            for t = testTimes
                actualThrust = thrust(t, testCase.TestData.Rocket);
                testCase.verifyEqual(actualThrust, 0, 'AbsTol', 1e-6, ...
                    sprintf('Thrust should be 0 at t=%.3f (after burn time)', t));
            end
        end
        
        function testAtBurnTime(testCase)
            % Test thrust exactly at burn time
            burnTime = testCase.TestData.Rocket.burnTime;
            
            % Get last thrust value from data
            expectedThrust = testCase.TestData.Rocket.thrustForce(end);
            
            actualThrust = thrust(burnTime, testCase.TestData.Rocket);
            
            testCase.verifyEqual(actualThrust, expectedThrust, 'RelTol', 1e-4, ...
                'Thrust at burn time should match last data point');
        end
        
        function testBeforeZero(testCase)
            % Test that thrust is zero for negative time
            testTimes = [-10, -5, -1, -0.5, -0.001, -1e-6];
            
            for t = testTimes
                actualThrust = thrust(t, testCase.TestData.Rocket);
                testCase.verifyEqual(actualThrust, 0, 'AbsTol', 1e-6, ...
                    sprintf('Thrust should be 0 at t=%.3f (negative time)', t));
            end
        end
        
        function testExtrapolationBehavior(testCase)
            % Test that function doesn't extrapolate (should return 0 after burn time)
            
            % Times well beyond burn time
            testTimes = [testCase.TestData.Rocket.burnTime + 1, ...
                        testCase.TestData.Rocket.burnTime + 10, ...
                        testCase.TestData.Rocket.burnTime + 100];
            
            for t = testTimes
                actualThrust = thrust(t, testCase.TestData.Rocket);
                testCase.verifyEqual(actualThrust, 0, 'AbsTol', 1e-6, ...
                    'Should not extrapolate beyond burn time');
            end
        end
        
        function testMonotonicBehavior(testCase)
            % Test that thrust is non-negative and physically reasonable
            
            % Test at various points throughout the burn
            testTimes = 0:0.1:testCase.TestData.Rocket.burnTime;
            
            for t = testTimes
                actualThrust = thrust(t, testCase.TestData.Rocket);
                testCase.verifyTrue(actualThrust >= 0, ...
                    sprintf('Thrust should be non-negative at t=%.3f', t));
                testCase.verifyTrue(isfinite(actualThrust), ...
                    sprintf('Thrust should be finite at t=%.3f', t));
            end
        end
        
        function testRocketStructureInput(testCase)
            % Test that function correctly handles Rocket structure
            
            % Create a minimal valid Rocket structure
            minimalRocket.thrustTime = [0, 0.1, 0.2, 0.3];
            minimalRocket.thrustForce = [0, 10, 20, 15];
            minimalRocket.burnTime = 0.3;
            
            % Test interpolation
            actualThrust = thrust(0.15, minimalRocket);
            expectedThrust = 15; % Linear interpolation between 10 and 20 at midpoint
            testCase.verifyEqual(actualThrust, expectedThrust, 'RelTol', 0.01, ...
                'Should work with minimal Rocket structure');
            
            % Test after burn time
            actualThrust = thrust(0.5, minimalRocket);
            testCase.verifyEqual(actualThrust, 0, 'AbsTol', 1e-6, ...
                'Should return 0 after burn time');
        end
    end
    
    methods (Test, ParameterCombination = 'sequential')
        function testMultipleTimePoints(testCase, timeValues)
            % Parameterized test for various time points
            actualThrust = thrust(timeValues, testCase.TestData.Rocket);
            
            if timeValues > testCase.TestData.Rocket.burnTime || timeValues < 0
                testCase.verifyEqual(actualThrust, 0, 'AbsTol', 1e-6, ...
                    sprintf('Thrust should be 0 at t=%.3f', timeValues));
            else
                testCase.verifyTrue(actualThrust >= 0, ...
                    sprintf('Thrust should be non-negative at t=%.3f', timeValues));
                testCase.verifyTrue(isfinite(actualThrust), ...
                    sprintf('Thrust should be finite at t=%.3f', timeValues));
            end
        end
    end
end