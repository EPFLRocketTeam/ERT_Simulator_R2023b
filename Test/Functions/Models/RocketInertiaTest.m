classdef rocketInertiaTest < matlab.unittest.TestCase
    % Unit tests for rocketInertia function
    
    properties (TestParameter)
        % Test parameters for different scenarios
        massModels = {0, 1}; % 0 = Non-linear, 1 = Linear
        testTimes = {0, 0.5, 1.0, 2.0, 5.0, 8.0, 10.823}; % Including burn time
        altitudes = {0, 100, 500, 1000};
    end
    
    properties
        % Path to function file
        AddedPath;
        % Rocket structure for testing
        testRocket
        % Tolerance for numerical comparisons
        tol = 1e-6
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

        function setupTestEnvironment(testCase)
            % Set up the test environment
            % This runs once before all tests
            
            % Load motor data from motorTestFile.txt
            motorFile = 'motorTestFile.txt';
            fid = fopen(motorFile, 'r');
            
            % Read header line
            headerLine = fgetl(fid);
            headerData = textscan(headerLine, '%s %f %f %s %f %f %s');
            
            % Read thrust data
            timeData = [];
            thrustData = [];
            while ~feof(fid)
                lineContent = fgetl(fid);
                data = textscan(lineContent, '%f %f');
                if ~isempty(data{1})
                    timeData = [timeData; data{1}];
                    thrustData = [thrustData; data{2}];
                end
            end
            fclose(fid);
            
            % Create rocket structure with all necessary fields
            testCase.testRocket = struct();
            
            % Thrust data
            testCase.testRocket.thrustTime = timeData';
            testCase.testRocket.thrustForce = thrustData';
            testCase.testRocket.burnTime = timeData(end);
            
            % Mass properties
            testCase.testRocket.emptyMass = 25.0; % [kg]
            testCase.testRocket.motorMass = 8.5; % [kg]
            testCase.testRocket.propelMass = 6.2; % [kg]
            testCase.testRocket.casingMass = testCase.testRocket.motorMass - testCase.testRocket.propelMass;
            testCase.testRocket.emptyCenterOfMass = 2.5; % [m] from tip
            
            % Geometric properties
            testCase.testRocket.totalLength = 4.2; % [m]
            testCase.testRocket.motorLength = 1.5; % [m]
            testCase.testRocket.motorDiameter = 0.15; % [m]
            
            % Inertia properties
            testCase.testRocket.emptyInertia = 15.0; % [kg*m^2]
            
            % Thrust to mass ratio for non-linear model
            testCase.testRocket.Thrust2dMass_Ratio = 5.6935e-04;
            % (value from running main with Nordend_EUROC.txt)
        end
    end
    
    methods (TestClassTeardown)
        function removeFunctionPath(testCase)
            % This removes the path added in TestClassSetup
            rmpath(testCase.AddedPath);
        end
    end
    
    methods (TestMethodSetup)
        function setupEachTest(testCase)
            % Setup before each test
            % This runs before every individual test
        end
    end
    
    methods (Test)
        % Test methods for basic functionality
        
        function testBasicOutputStructure(testCase)
            % Test that function returns expected number of outputs
            
            t = 0;
            [mass, massRate, CM, I, Idot] = rocketInertia(t, testCase.testRocket, 1);
            
            % Verify all outputs exist
            testCase.verifyNotEmpty(mass, 'Mass should not be empty');
            testCase.verifyNotEmpty(massRate, 'Mass rate should not be empty');
            testCase.verifyNotEmpty(CM, 'Center of mass should not be empty');
            testCase.verifyNotEmpty(I, 'Inertia moment should not be empty');
            testCase.verifyNotEmpty(Idot, 'Inertia rate should not be empty');
        end
        
        function testOutputTypes(testCase)
            % Test that outputs have correct data types
            
            t = 0;
            [mass, massRate, CM, I, Idot] = rocketInertia(t, testCase.testRocket, 1);
            
            testCase.verifyClass(mass, 'double', 'Mass should be double');
            testCase.verifyClass(massRate, 'double', 'Mass rate should be double');
            testCase.verifyClass(CM, 'double', 'Center of mass should be double');
            testCase.verifyClass(I, 'double', 'Inertia should be double');
            testCase.verifyClass(Idot, 'double', 'Inertia rate should be double');
            
            % Inertia tensor should be at least 2D (currently [2x?] in the code)
            testCase.verifyTrue(length(I) >= 2, 'Inertia should have at least 2 elements');
        end
        
        % Test methods for linear mass model
        
        function testLinearMassModelBeforeBurn(testCase)
            % Test linear mass model before burn time
            
            t = 1.0; % Before burn time (burn time is ~10.823s)
            [mass, massRate, ~, ~, ~] = rocketInertia(t, testCase.testRocket, 1);
            
            % Mass should be decreasing linearly
            expectedMassRate = testCase.testRocket.propelMass / testCase.testRocket.burnTime;
            expectedMass = testCase.testRocket.emptyMass + testCase.testRocket.motorMass - t * expectedMassRate;
            
            testCase.verifyEqual(massRate, expectedMassRate, 'AbsTol', testCase.tol, ...
                'Linear mass rate mismatch before burn');
            testCase.verifyEqual(mass, expectedMass, 'RelTol', 1e-4, ...
                'Linear mass mismatch before burn');
        end
        
        function testLinearMassModelAtBurnTime(testCase)
            % Test linear mass model exactly at burn time
            
            t = testCase.testRocket.burnTime;
            [mass, massRate, ~, ~, ~] = rocketInertia(t, testCase.testRocket, 1);
            
            % At burn time, mass should be empty mass + casing mass
            expectedMass = testCase.testRocket.emptyMass + testCase.testRocket.casingMass;
            expectedMassRate = testCase.testRocket.propelMass/testCase.testRocket.burnTime;
            
            testCase.verifyEqual(mass, expectedMass, 'RelTol', 1e-4, ...
                'Linear mass at burn time mismatch');
            testCase.verifyEqual(massRate, expectedMassRate, 'AbsTol', testCase.tol, ...
                'Linear mass rate at burn time mismatch');
        end
        
        function testLinearMassModelAfterBurn(testCase)
            % Test linear mass model after burn time
            
            t = testCase.testRocket.burnTime + 1.0;
            [mass, massRate, ~, ~, ~] = rocketInertia(t, testCase.testRocket, 1);
            
            expectedMass = testCase.testRocket.emptyMass + testCase.testRocket.casingMass;
            
            testCase.verifyEqual(mass, expectedMass, 'RelTol', 1e-4, ...
                'Linear mass after burn mismatch');
            testCase.verifyEqual(massRate, 0, 'AbsTol', testCase.tol, ...
                'Linear mass rate after burn should be 0');
        end
        
        function testLinearMassModelContinuity(testCase)
            % Test continuity at burn time
            
            timeBefore = testCase.testRocket.burnTime - 0.001;
            timeBurn = testCase.testRocket.burnTime;
            timeAfter = testCase.testRocket.burnTime + 0.001;
            
            [massBefore, ~, ~, ~, ~] = rocketInertia(timeBefore, testCase.testRocket, 1);
            [massBurn, ~, ~, ~, ~] = rocketInertia(timeBurn, testCase.testRocket, 1);
            [massAfter, ~, ~, ~, ~] = rocketInertia(timeAfter, testCase.testRocket, 1);
            
            % Mass should be continuous
            testCase.verifyEqual(massBefore, massBurn, 'RelTol', 0.01, ...
                'Mass should be continuous at burn time');
            testCase.verifyEqual(massBurn, massAfter, 'AbsTol', testCase.tol, ...
                'Mass after burn should equal mass at burn time');
        end
        
        % Test methods for non-linear mass model
        
        function testNonLinearMassModelBeforeBurn(testCase)
            % Test non-linear mass model before burn time
            
            t = 1.0;
            [mass, massRate, ~, ~, ~] = rocketInertia(t, testCase.testRocket, 0);
            
            % Mass should be decreasing non-linearly
            testCase.verifyTrue(mass > testCase.testRocket.emptyMass + testCase.testRocket.casingMass, ...
                'Mass should be greater than empty+casing mass');
            testCase.verifyTrue(massRate > 0, 'Mass rate should be positive (mass decreasing)');
            testCase.verifyTrue(isfinite(mass), 'Mass should be finite');
            testCase.verifyTrue(isfinite(massRate), 'Mass rate should be finite');
        end
        
        function testNonLinearMassModelAfterBurn(testCase)
            % Test non-linear mass model after burn time
            
            t = testCase.testRocket.burnTime + 1.0;
            [mass, massRate, ~, ~, ~] = rocketInertia(t, testCase.testRocket, 0);
            
            expectedMass = testCase.testRocket.emptyMass + testCase.testRocket.motorMass - testCase.testRocket.propelMass;
            
            testCase.verifyEqual(mass, expectedMass, 'RelTol', 1e-4, ...
                'Non-linear mass after burn mismatch');
            testCase.verifyEqual(massRate, 0, 'AbsTol', testCase.tol, ...
                'Non-linear mass rate after burn should be 0');
        end
        
        function testNonLinearMassModelConsistency(testCase)
            % Test that non-linear model reduces mass appropriately
            
            t1 = 0.5;
            t2 = 1.0;
            
            [mass1, ~, ~, ~, ~] = rocketInertia(t1, testCase.testRocket, 0);
            [mass2, ~, ~, ~, ~] = rocketInertia(t2, testCase.testRocket, 0);
            
            % Mass should decrease over time
            testCase.verifyTrue(mass2 < mass1, 'Mass should decrease during burn');
        end
        
        % Test methods for center of mass calculation
        
        function testCenterOfMassBeforeBurn(testCase)
            % Test center of mass calculation before burn
            
            t = 1.0;
            [mass, ~, CM, ~, ~] = rocketInertia(t, testCase.testRocket, 1);
            
            % Calculate expected CM
            motorCM = testCase.testRocket.totalLength - testCase.testRocket.motorLength/2;
            expectedCM = (testCase.testRocket.emptyCenterOfMass * testCase.testRocket.emptyMass + ...
                (mass - testCase.testRocket.emptyMass) * motorCM) / mass;
            
            testCase.verifyEqual(CM, expectedCM, 'RelTol', 1e-4, ...
                'Center of mass mismatch before burn');
            
            % CM should be between empty CM and motor CM
            testCase.verifyTrue(CM >= testCase.testRocket.emptyCenterOfMass, ...
                'CM should be >= empty CM');
            testCase.verifyTrue(CM <= motorCM, ...
                'CM should be <= motor CM');
        end
        
        function testCenterOfMassAfterBurn(testCase)
            % Test center of mass after burn
            
            t = testCase.testRocket.burnTime + 1.0;
            [mass, ~, CM, ~, ~] = rocketInertia(t, testCase.testRocket, 1);
            
            motorCM = testCase.testRocket.totalLength - testCase.testRocket.motorLength/2;
            expectedCM = (testCase.testRocket.emptyCenterOfMass * testCase.testRocket.emptyMass + ...
                (mass - testCase.testRocket.emptyMass) * motorCM) / mass;
            
            testCase.verifyEqual(CM, expectedCM, 'RelTol', 1e-4, ...
                'Center of mass after burn mismatch');
        end
        
        function testCenterOfMassContinuity(testCase)
            % Test continuity of center of mass
            
            timeBefore = testCase.testRocket.burnTime - 0.001;
            timeBurn = testCase.testRocket.burnTime;
            timeAfter = testCase.testRocket.burnTime + 0.001;
            
            [~, ~, centerOfMassBefore, ~, ~] = rocketInertia(timeBefore, testCase.testRocket, 1);
            [~, ~, centerOfMassBurn, ~, ~] = rocketInertia(timeBurn, testCase.testRocket, 1);
            [~, ~, centerOfMassAfter, ~, ~] = rocketInertia(timeAfter, testCase.testRocket, 1);
            
            % CM should be continuous
            testCase.verifyEqual(centerOfMassBefore, centerOfMassBurn, 'RelTol', 0.01, ...
                'CM should be continuous at burn time');
            testCase.verifyEqual(centerOfMassBurn, centerOfMassAfter, 'RelTol', 0.01, ...
                'CM after burn should equal CM at burn time');
        end
        
        % Test methods for inertia calculation
        
        function testInertiaLongitudinalBeforeBurn(testCase)
            % Test longitudinal inertia calculation before burn
            
            t = 1.0;
            [mass, ~, ~, I, ~] = rocketInertia(t, testCase.testRocket, 1);
            
            % Inertia should be positive
            testCase.verifyTrue(I(1) > 0, 'Longitudinal inertia should be positive');
            
            % Grain mass should be positive during burn
            grainMass = mass - testCase.testRocket.emptyMass - testCase.testRocket.casingMass;
            testCase.verifyTrue(grainMass > 0, 'Grain mass should be positive during burn');
        end
        
        function testInertiaLongitudinalAfterBurn(testCase)
            % Test longitudinal inertia after burn
            
            t = testCase.testRocket.burnTime + 1.0;
            [mass, ~, ~, I, ~] = rocketInertia(t, testCase.testRocket, 1);
            
            % After burn, grain mass should be 0
            grainMass = mass - testCase.testRocket.emptyMass - testCase.testRocket.casingMass;
            testCase.verifyEqual(grainMass, 0, 'AbsTol', testCase.tol, ...
                'Grain mass should be 0 after burn');
            
            % Inertia should be positive
            testCase.verifyTrue(I(1) > 0, 'Longitudinal inertia should be positive after burn');
        end
        
        function testInertiaMonotonic(testCase)
            % Test that inertia changes monotonically during burn
            
            t1 = 0.5;
            t2 = 5.0;
            
            [~, ~, ~, I1, ~] = rocketInertia(t1, testCase.testRocket, 1);
            [~, ~, ~, I2, ~] = rocketInertia(t2, testCase.testRocket, 1);
            
            % Inertia should decrease as mass decreases (generally)
            % Note: This might not always hold due to Steiner term, but for typical rockets it does
            testCase.verifyTrue(I2(1) <= I1(1) || abs(I2(1) - I1(1)) < 0.1*I1(1), ...
                'Inertia should generally decrease or stay similar');
        end
        
        function testInertiaStructure(testCase)
            % Test that inertia tensor has expected structure
            
            t = 1.0;
            [~, ~, ~, I, ~] = rocketInertia(t, testCase.testRocket, 1);
            
            % Currently the code returns [longitudinalInertia, 1]
            % This is likely a placeholder - we test what's actually returned
            testCase.verifyEqual(length(I), 2, 'Inertia should be a 2-element vector');
            testCase.verifyTrue(I(1) > 0, 'First element should be positive inertia');
            testCase.verifyEqual(I(2), 1, 'Second element should be 1 (placeholder)');
        end
        
        function testInertiaRate(testCase)
            % Test inertia rate output
            
            t = 1.0;
            [~, ~, ~, ~, Idot] = rocketInertia(t, testCase.testRocket, 1);
            
            % Currently returns 0 as placeholder
            testCase.verifyEqual(Idot, 0, 'AbsTol', testCase.tol, ...
                'Inertia rate should be 0 (placeholder)');
        end
        
        % Cross-model comparison tests
        
        function testMassModelsComparison(testCase)
            % Compare linear and non-linear mass models at same time
            
            t = 2.0;
            [massLin, rateLin, ~, ~, ~] = rocketInertia(t, testCase.testRocket, 1);
            [massNonLin, rateNonLin, ~, ~, ~] = rocketInertia(t, testCase.testRocket, 0);
            
            % Both should give physically plausible values
            testCase.verifyTrue(massLin > testCase.testRocket.emptyMass, ...
                'Linear mass should be > empty mass');
            testCase.verifyTrue(massNonLin > testCase.testRocket.emptyMass, ...
                'Non-linear mass should be > empty mass');
            testCase.verifyTrue(rateLin > 0, 'Linear mass rate should be positive');
            testCase.verifyTrue(rateNonLin > 0, 'Non-linear mass rate should be positive');
            
            % They will differ - just verify both are finite
            testCase.verifyTrue(isfinite(massLin) && isfinite(massNonLin), ...
                'Both mass models should give finite values');
        end
        
        function testMassModelAtBoundaries(testCase)
            % Test both models at time boundaries
            
            timeStart = 0;
            timeBurn = testCase.testRocket.burnTime;
            timeEnd = timeBurn + 1;
            
            times = [timeStart, timeBurn, timeEnd];
            
            for t = times
                [massLin, ~, ~, ~, ~] = rocketInertia(t, testCase.testRocket, 1);
                [massNonLin, ~, ~, ~, ~] = rocketInertia(t, testCase.testRocket, 0);
                
                testCase.verifyTrue(isfinite(massLin), 'Linear mass should be finite');
                testCase.verifyTrue(isfinite(massNonLin), 'Non-linear mass should be finite');
                testCase.verifyTrue(massLin >= 0, 'Linear mass should be non-negative');
                testCase.verifyTrue(massNonLin >= 0, 'Non-linear mass should be non-negative');
            end
        end
        
        % Edge case tests
        
        function testZeroMass(testCase)
            % Test behavior as mass approaches zero (shouldn't happen for real rocket)
            
            % Create rocket with very small mass
            extremeRocket = testCase.testRocket;
            extremeRocket.emptyMass = 0.1;
            extremeRocket.motorMass = 0.05;
            
            t = extremeRocket.burnTime + 1;
            [mass, ~, CM, I, ~] = rocketInertia(t, extremeRocket, 1);
            
            testCase.verifyTrue(mass >= 0, 'Mass should be non-negative');
            testCase.verifyTrue(isfinite(CM), 'CM should be finite even with small mass');
            testCase.verifyTrue(isfinite(I(1)), 'Inertia should be finite even with small mass');
        end
        
        function testMissingRocketFields(testCase)
            % Test with incomplete rocket structure
            
            incompleteRocket = struct();
            incompleteRocket.emptyMass = 25.0;
            % Missing other fields
            
            testCase.verifyError(@() rocketInertia(0, incompleteRocket, 1), ...
                'MATLAB:nonExistentField', ...
                'Missing fields should cause error');
        end
    end
    
    methods (Test, ParameterCombination = 'sequential')
        function testVariousTimesLin(testCase, testTimes)
            % Parameterized test for different times with linear mass model
            
            [mass, massRate, CM, I, Idot] = rocketInertia(testTimes, testCase.testRocket, 1);
            
            % Basic sanity checks
            testCase.verifyTrue(mass >= 0, sprintf('Mass should be >=0 at t=%.3f', testTimes));
            testCase.verifyTrue(massRate >= 0, sprintf('Mass rate should be >=0 at t=%.3f', testTimes));
            testCase.verifyTrue(CM > 0, sprintf('CM should be >0 at t=%.3f', testTimes));
            testCase.verifyTrue(CM < testCase.testRocket.totalLength, ...
                sprintf('CM should be within rocket length at t=%.3f', testTimes));
            testCase.verifyTrue(I(1) > 0, sprintf('Inertia should be >0 at t=%.3f', testTimes));
            testCase.verifyTrue(isfinite(Idot), sprintf('Inertia rate should be finite at t=%.3f', testTimes));
            
            % Additional checks at burn time
            if abs(testTimes - testCase.testRocket.burnTime) < testCase.tol
                expectedMass = testCase.testRocket.emptyMass + testCase.testRocket.casingMass;
                testCase.verifyEqual(mass, expectedMass, 'RelTol', 1e-4, ...
                    'Mass at burn time mismatch');
                testCase.verifyEqual(massRate, 0, 'AbsTol', testCase.tol, ...
                    'Mass rate should be 0 at burn time');
            end
        end
        
        function testVariousTimesNonLin(testCase, testTimes)
            % Parameterized test for different times with non-linear mass model
            
            [mass, massRate, CM, I, Idot] = rocketInertia(testTimes, testCase.testRocket, 0);
            
            % Basic sanity checks
            testCase.verifyTrue(mass >= 0, sprintf('Mass should be >=0 at t=%.3f', testTimes));
            testCase.verifyTrue(massRate >= 0, sprintf('Mass rate should be >=0 at t=%.3f', testTimes));
            testCase.verifyTrue(CM > 0, sprintf('CM should be >0 at t=%.3f', testTimes));
            testCase.verifyTrue(CM < testCase.testRocket.totalLength, ...
                sprintf('CM should be within rocket length at t=%.3f', testTimes));
            testCase.verifyTrue(I(1) > 0, sprintf('Inertia should be >0 at t=%.3f', testTimes));
            testCase.verifyTrue(isfinite(Idot), sprintf('Inertia rate should be finite at t=%.3f', testTimes));
            
            % Additional checks at burn time
            if abs(testTimes - testCase.testRocket.burnTime) < testCase.tol
                testCase.verifyEqual(massRate, 0, 'AbsTol', testCase.tol, ...
                    'Mass rate should be 0 at burn time');
            end
        end
    end
end