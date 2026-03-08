classdef RocketInertiaTest < matlab.unittest.TestCase
    
    properties
        AddedPath;
        testRocket;
        tolerance = 1e-10;
        timeVector;
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
            
            % Mass properties
            testCase.testRocket.emptyMass = 10; % kg
            testCase.testRocket.casing_mass = 2; % kg
            testCase.testRocket.motor_mass = 8; % kg
            testCase.testRocket.propel_mass = 5; % kg
            testCase.testRocket.emptyInertia = 0.5; % kg*m^2
            
            % Geometric properties
            testCase.testRocket.totalLength = 2; % m
            testCase.testRocket.motor_length = 1; % m
            testCase.testRocket.motor_dia = 0.1; % m
            testCase.testRocket.emptyCenterOfMass = 1; % m from tip
            
            % Time properties
            testCase.testRocket.Burn_Time = 5; % s
            
            % Thrust properties (for non-linear model)
            testCase.testRocket.Thrust2dMass_Ratio = 0.001; % kg/N/s
            
            % Time vector for testing
            testCase.timeVector = [0, 2, 4, 5, 6, 10];
        end
    end
    
    methods (TestClassTeardown)
        function removeFunctionPath(testCase)
            % This removes the path added in TestClassSetup
            rmpath(testCase.AddedPath);
        end
    end
    
    methods (Test)
        
        function testLinearMassModelOutputs(testCase)
            % Test linear mass model returns correct number and type of outputs
            t = 0;
            
            [mass, massRate, centerOfMass, inertiaMoment, inertiaRate] = RocketInertia(t, testCase.testRocket, 0);
            
            % Verify all outputs exist and are numeric
            testCase.verifyTrue(isnumeric(mass));
            testCase.verifyTrue(isnumeric(massRate));
            testCase.verifyTrue(isnumeric(centerOfMass));
            testCase.verifyTrue(isnumeric(inertiaMoment));
            testCase.verifyTrue(isnumeric(inertiaRate));
            
            % Verify sizes
            testCase.verifySize(mass, [1, 1]);
            testCase.verifySize(massRate, [1, 1]);
            testCase.verifySize(centerOfMass, [1, 1]);
            testCase.verifySize(inertiaMoment, [1, 2]); % Currently I_L and 1 placeholder
        end
        
        function testNonLinearMassModelOutputs(testCase)
            % Test non-linear mass model returns correct number and type of outputs
            t = 0;
            
            [mass, massRate, centerOfMass, inertiaMoment, inertiaRate] = RocketInertia(t, testCase.testRocket, 1);
            
            % Verify all outputs exist and are numeric
            testCase.verifyTrue(isnumeric(mass));
            testCase.verifyTrue(isnumeric(massRate));
            testCase.verifyTrue(isnumeric(centerOfMass));
            testCase.verifyTrue(isnumeric(inertiaMoment));
            testCase.verifyTrue(isnumeric(inertiaRate));
            
            % Verify sizes
            testCase.verifySize(mass, [1, 1]);
            testCase.verifySize(massRate, [1, 1]);
            testCase.verifySize(centerOfMass, [1, 1]);
            testCase.verifySize(inertiaMoment, [1, 2]);
        end
        
        function testMassPositive(testCase)
            % Test that mass is always positive
            for t = testCase.timeVector
                [mass, ~, ~, ~, ~] = RocketInertia(t, testCase.testRocket, 0);
                testCase.verifyGreaterThan(mass, 0);
                
                [mass, ~, ~, ~, ~] = RocketInertia(t, testCase.testRocket, 1);
                testCase.verifyGreaterThan(mass, 0);
            end
        end
        
        function testCMBounds(testCase)
            % Test that center of mass is within rocket length
            for t = testCase.timeVector
                [~, ~, centerOfMass, ~, ~] = RocketInertia(t, testCase.testRocket, 0);
                
                % CM should be between 0 and total length
                testCase.verifyGreaterThanOrEqual(centerOfMass, 0);
                testCase.verifyLessThanOrEqual(centerOfMass, testCase.testRocket.totalLength);
            end
        end
        
        function testEmptyRocketState(testCase)
            % Test at time when rocket is empty (mass = emptyMass)
            % Note: This assumes mass model returns emptyMass at some time
            % You may need to adjust this based on actual mass model behavior
            
            % For linear model at t=0
            [mass, ~, ~, inertiaMoment, ~] = RocketInertia(0, testCase.testRocket, 0);
            
            % Verify mass is at least empty mass
            testCase.verifyGreaterThanOrEqual(mass, testCase.testRocket.emptyMass);
            
            % Verify inertia is at least empty inertia
            testCase.verifyGreaterThanOrEqual(inertiaMoment(1), testCase.testRocket.emptyInertia);
        end
        
        function testMassDerivativeSign(testCase)
            % Test that mass derivative is negative (mass decreasing) or zero
            for t = testCase.timeVector
                [~, massRateLin, ~, ~, ~] = RocketInertia(t, testCase.testRocket, 0);
                [~, massRateNonlin, ~, ~, ~] = RocketInertia(t, testCase.testRocket, 1);
                
                % Mass should be decreasing or constant
                testCase.verifyLessThanOrEqual(massRateLin, 0);
                testCase.verifyLessThanOrEqual(massRateNonlin, 0);
            end
        end
        
        function testInertiaTensorStructure(testCase)
            % Test that inertia tensor has correct structure
            [~, ~, ~, inertiaMoment, ~] = RocketInertia(0, testCase.testRocket, 0);
            
            % Currently inertiaMoment is [I_L, 1] - first element is longitudinal inertia
            testCase.verifyTrue(isfinite(inertiaMoment(1))); % I_L
            testCase.verifyEqual(inertiaMoment(2), 1); % Placeholder
        end
        
        function testInertiaDerivativeInitial(testCase)
            % Test inertia derivative placeholder
            [~, ~, ~, ~, inertiaRate] = RocketInertia(0, testCase.testRocket, 0);
            
            % Currently placeholder value
            testCase.verifyEqual(inertiaRate, 0);
        end
        
        function testCMCalculation(testCase)
            % Verify center of mass calculation formula
            t = 0;
            [mass, ~, centerOfMass, ~, ~] = RocketInertia(t, testCase.testRocket, 0);
            
            % Manually calculate expected CM
            propellantMass = mass - testCase.testRocket.emptyMass;
            expectedCenterOfMass = (testCase.testRocket.emptyCenterOfMass * ...
                testCase.testRocket.emptyMass + ...
                propellantMass * (testCase.testRocket.totalLength - ...
                testCase.testRocket.motor_length/2)) / mass;
            
            testCase.verifyEqual(centerOfMass, expectedCenterOfMass, 'AbsTol', testCase.tolerance);
        end
        
        function testLongitudinalInertiaCalculation(testCase)
            % Verify longitudinal inertia calculation
            t = 0;
            [mass, ~, centerOfMass, inertiaMoment, ~] = RocketInertia(t, testCase.testRocket, 0);
            
            % Calculate expected values
            R_e = testCase.testRocket.motor_dia/2;
            R_i = 0.005; % Internal grain diameter (hardcoded in function)
            grainMass = mass - testCase.testRocket.emptyMass - testCase.testRocket.casing_mass;
            
            % Casing inertia
            I_L_casing_expected = testCase.testRocket.casing_mass * ...
                (testCase.testRocket.motor_length^2/12 + R_e^2/2);
            
            % Grain inertia
            I_L_grain_expected = grainMass * ...
                (testCase.testRocket.motor_length^2/12 + (R_e^2 + R_i^2)/4);
            
            % Steiner term
            steiner_expected = (grainMass + testCase.testRocket.casing_mass) * ...
                (testCase.testRocket.totalLength - centerOfMass - testCase.testRocket.motor_length/2);
            
            I_L_expected = testCase.testRocket.emptyInertia + I_L_casing_expected + ...
                I_L_grain_expected + steiner_expected;
            
            testCase.verifyEqual(inertiaMoment(1), I_L_expected, 'AbsTol', testCase.tolerance);
        end
        
        function testMassModelSwitch(testCase)
            % Test that mass model switch works correctly
            t = 1;
            
            % Get results from both models
            [massLin, massRateLin, ~, ~, ~] = ...
                RocketInertia(t, testCase.testRocket, 0);
            [massNonlin, massRateNonlin, ~, ~, ~] = ...
                RocketInertia(t, testCase.testRocket, 1);
            
            % Results should be different for different mass models
            % (unless mass models return same values at this time)
            testCase.verifyNotEqual(massLin, massNonlin);
            testCase.verifyNotEqual(massRateLin, massRateNonlin);
        end
        
        function testInvalidMassModel(testCase)
            % Test with invalid mass model value
            t = 0;
            
            % Should still work with any numeric input? Let's test
            [mass, massRate, ~, ~, ~] = RocketInertia(t, testCase.testRocket, 2);
            
            % Should still produce outputs (may use default behavior)
            testCase.verifyTrue(isnumeric(mass));
            testCase.verifyTrue(isnumeric(massRate));
        end
        
        function testTimeDependence(testCase)
            % Test that outputs change with time
            resultsMass = zeros(length(testCase.timeVector), 1);
            resultsCenterOfMass = zeros(length(testCase.timeVector), 1);
            resultsInertia = zeros(length(testCase.timeVector), 1);
            
            for i = 1:length(testCase.timeVector)
                [mass, ~, centerOfMass, inertiaMoment, ~] = RocketInertia(...
                    testCase.timeVector(i), testCase.testRocket, 0);
                resultsMass(i) = mass;
                resultsCenterOfMass(i) = centerOfMass;
                resultsInertia(i) = inertiaMoment(1);
            end
            
            % Verify that values change (not all identical)
            testCase.verifyTrue(std(resultsMass) > 0 || ...
                any(diff(resultsMass) ~= 0));
        end
        
        function testSteinerTermPositive(testCase)
            % Test that Steiner term is positive (contributes positively to inertia)
            t = 0;
            [mass, ~, ~, inertiaMoment, ~] = RocketInertia(t, testCase.testRocket, 0);
            
            % Calculate inertia without Steiner term
            R_e = testCase.testRocket.motor_dia/2;
            R_i = 0.005;
            grainMass = mass - testCase.testRocket.emptyMass - testCase.testRocket.casing_mass;
            
            I_L_casing = testCase.testRocket.casing_mass * ...
                (testCase.testRocket.motor_length^2/12 + R_e^2/2);
            I_L_grain = grainMass * ...
                (testCase.testRocket.motor_length^2/12 + (R_e^2 + R_i^2)/4);
            
            I_L_no_steiner = testCase.testRocket.emptyInertia + I_L_casing + I_L_grain;
            
            % Inertia with Steiner should be larger
            testCase.verifyGreaterThan(inertiaMoment(1), I_L_no_steiner);
        end
        
        function testMultipleTimePoints(testCase)
            % Test function works for multiple time points sequentially
            for t = testCase.timeVector
                testCase.verifyWarningFree(@() ...
                    RocketInertia(t, testCase.testRocket, 0));
            end
        end
        
        function testExtremeTimeValues(testCase)
            % Test with extreme time values
            extremeTimes = [-1000, 0, 1000];
            
            for t = extremeTimes
                [mass, massRate, centerOfMass, inertiaMoment, inertiaRate] = RocketInertia(t, testCase.testRocket, 0);
                
                % Should still produce finite outputs
                testCase.verifyTrue(isfinite(mass));
                testCase.verifyTrue(isfinite(massRate));
                testCase.verifyTrue(isfinite(centerOfMass));
                testCase.verifyTrue(all(isfinite(inertiaMoment)));
                testCase.verifyTrue(isfinite(inertiaRate));
            end
        end
        
        function testZeroMotorLength(testCase)
            % Test edge case with zero motor length
            rocket = testCase.testRocket;
            rocket.motor_length = 0;
            
            [mass, ~, centerOfMass, inertiaMoment, ~] = RocketInertia(0, rocket, 0);
            
            % Should still compute without division by zero
            testCase.verifyTrue(isfinite(mass));
            testCase.verifyTrue(isfinite(centerOfMass));
            testCase.verifyTrue(all(isfinite(inertiaMoment)));
        end
    end
end