classdef pitchDampingMomentTest < matlab.unittest.TestCase
    
    properties
        AddedPath;
        testRocket;
        testRho;
        testCalpha;
        testCP;
        testCM;
        testdMdt;
        testW;
        testV;
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
            % Create a standard test rocket
            testCase.testRocket = struct(...
                'totalLength', 2.0, ... % meters
                'maxCrossSectionArea', pi*0.1^2, ... % m^2
                'stagePositions', [0, 0.2, 0.5, 0.8, 2.0], ...
                'numFins', 3, ...
                'exposedFinArea', 0.01, ... % m^2
                'maxDiameter', 0.2, ... % meters
                'finRootPosition', 1.7); % meters from nose
            
            % Test conditions
            testCase.testRho = 1.225; % kg/m^3 (air density at sea level)
            testCase.testCalpha = [0.5, 0.3, 1.2]; % Component lift coefficients
            testCase.testCP = [0.1, 0.4, 1.8]; % Component center of pressure [m]
            testCase.testCM = 1.2; % Center of mass [m]
            testCase.testdMdt = 0; % No mass change
            testCase.testW = 10; % Angular velocity [rad/s]
            testCase.testV = 100; % Velocity [m/s]
        end
    end
    
    methods (Test)
        function testBasicFunctionality(testCase)
            % Test basic functionality with nominal inputs
            CDM = pitchDampingMoment(testCase.testRocket, testCase.testRho, ...
                testCase.testCalpha, testCase.testCP, testCase.testdMdt, ...
                testCase.testCM, testCase.testW, testCase.testV);
            
            % Verify output is finite
            testCase.verifyThat(CDM, matlab.unittest.constraints.IsFinite);
            
            % Damping coefficient should be positive
            testCase.verifyGreaterThan(CDM, 0);
        end
        
        function testZeroVelocity(testCase)
            % Test with zero velocity
            CDM = pitchDampingMoment(testCase.testRocket, testCase.testRho, ...
                testCase.testCalpha, testCase.testCP, testCase.testdMdt, ...
                testCase.testCM, testCase.testW, 0);
            
            % Should return zero
            testCase.verifyEqual(CDM, 0);
        end
        
        function testAerodynamicDampingOnly(testCase)
            % Test with no mass change (aerodynamic damping only)
            CDM = pitchDampingMoment(testCase.testRocket, testCase.testRho, ...
                testCase.testCalpha, testCase.testCP, 0, ...
                testCase.testCM, testCase.testW, testCase.testV);
            
            % Should be positive
            testCase.verifyGreaterThan(CDM, 0);
            
            % Calculate expected value manually
            CNa_Total = sum(testCase.testCalpha .* (testCase.testCP - testCase.testCM).^2);
            expected_CDM = CNa_Total * testCase.testW / testCase.testV;
            
            testCase.verifyEqual(CDM, expected_CDM, 'AbsTol', 1e-10);
        end
        
        function testThrustDampingOnly(testCase)
            % Test with thrust damping only (no aerodynamic damping)
            % Note: Can't have zero aerodynamic damping, but we can make it small
            Calpha_small = [1e-6, 1e-6, 1e-6];
            dMdt = 10; % Positive mass change (burning)
            
            CDM = pitchDampingMoment(testCase.testRocket, testCase.testRho, ...
                Calpha_small, testCase.testCP, dMdt, ...
                testCase.testCM, testCase.testW, testCase.testV);
            
            % Thrust damping should be positive (destabilizing)
            testCase.verifyGreaterThan(CDM, 0);
        end
        
        function testBothDampingComponents(testCase)
            % Test with both aerodynamic and thrust damping
            dMdt = 5; % Positive mass change
            
            CDM = pitchDampingMoment(testCase.testRocket, testCase.testRho, ...
                testCase.testCalpha, testCase.testCP, dMdt, ...
                testCase.testCM, testCase.testW, testCase.testV);
            
            % Calculate components
            CNa_Total = sum(testCase.testCalpha .* (testCase.testCP - testCase.testCM).^2);
            CDM_aero = CNa_Total * testCase.testW / testCase.testV;
            
            CDM_thrust = dMdt * (testCase.testRocket.totalLength - testCase.testCM)^2 * ...
                testCase.testW * 2 / testCase.testV^2 / testCase.testRho / ...
                testCase.testRocket.maxCrossSectionArea;
            
            expected_CDM = CDM_aero + CDM_thrust;
            
            testCase.verifyEqual(CDM, expected_CDM, 'AbsTol', 1e-10);
        end
        
    end
    
    methods (Test, TestTags = {'AerodynamicDamping'})
        
        function testAerodynamicDampingScalingWithW(testCase)
            % Test that aerodynamic damping scales linearly with angular velocity
            w_values = [5, 10, 20, 50];
            CDM_results = zeros(size(w_values));
            
            for i = 1:length(w_values)
                CDM_results(i) = pitchDampingMoment(testCase.testRocket, ...
                    testCase.testRho, testCase.testCalpha, testCase.testCP, 0, ...
                    testCase.testCM, w_values(i), testCase.testV);
            end
            
            % Check linear scaling
            for i = 2:length(w_values)
                ratio = CDM_results(i) / CDM_results(1);
                expected_ratio = w_values(i) / w_values(1);
                testCase.verifyEqual(ratio, expected_ratio, 'AbsTol', 1e-6);
            end
        end
        
        function testAerodynamicDampingScalingWithV(testCase)
            % Test that aerodynamic damping scales inversely with velocity
            v_values = [50, 100, 200, 400];
            CDM_results = zeros(size(v_values));
            
            for i = 1:length(v_values)
                CDM_results(i) = pitchDampingMoment(testCase.testRocket, ...
                    testCase.testRho, testCase.testCalpha, testCase.testCP, 0, ...
                    testCase.testCM, testCase.testW, v_values(i));
            end
            
            % Check inverse scaling
            for i = 2:length(v_values)
                ratio = CDM_results(i) / CDM_results(1);
                expected_ratio = v_values(1) / v_values(i);
                testCase.verifyEqual(ratio, expected_ratio, 'AbsTol', 1e-6);
            end
        end
        
        function testAerodynamicDampingMomentArm(testCase)
            % Test effect of moment arm (CP - CM)
            Calpha_test = [1.0];
            
            % Test with different CP positions
            CP_positions = [0.5, 1.0, 1.5, 2.0];
            CDM_results = zeros(size(CP_positions));
            
            for i = 1:length(CP_positions)
                CDM_results(i) = pitchDampingMoment(testCase.testRocket, ...
                    testCase.testRho, Calpha_test, CP_positions(i), 0, ...
                    1.0, testCase.testW, testCase.testV);
            end
            
            % Should scale with (CP-CM)^2
            moment_arms = CP_positions - 1.0;
            for i = 2:length(CP_positions)
                ratio = CDM_results(i) / CDM_results(1);
                expected_ratio = (moment_arms(i)^2) / (moment_arms(1)^2);
                testCase.verifyEqual(ratio, expected_ratio, 'AbsTol', 1e-6);
            end
        end
        
        function testMultipleComponents(testCase)
            % Test with multiple components contributing to damping
            Calpha_multi = [0.5, 0.3, 0.2];
            CP_multi = [0.8, 1.2, 1.6];
            CM = 1.0;
            
            CDM = pitchDampingMoment(testCase.testRocket, testCase.testRho, ...
                Calpha_multi, CP_multi, 0, CM, testCase.testW, testCase.testV);
            
            % Calculate expected (sum of squares)
            CNa_Total = sum(Calpha_multi .* (CP_multi - CM).^2);
            expected = CNa_Total * testCase.testW / testCase.testV;
            
            testCase.verifyEqual(CDM, expected, 'AbsTol', 1e-10);
        end
        
    end
    
    methods (Test, TestTags = {'ThrustDamping'})
        
        function testThrustDampingScaling(testCase)
            % Test thrust damping scaling with various parameters
            Calpha_small = [1e-6, 1e-6, 1e-6]; % Negligible aero damping
            
            % Test scaling with dMdt
            dMdt_values = [5, 10, 20, 40];
            CDM_results = zeros(size(dMdt_values));
            
            for i = 1:length(dMdt_values)
                CDM_results(i) = pitchDampingMoment(testCase.testRocket, ...
                    testCase.testRho, Calpha_small, testCase.testCP, dMdt_values(i), ...
                    testCase.testCM, testCase.testW, testCase.testV);
            end
            
            % Should scale linearly with dMdt
            for i = 2:length(dMdt_values)
                ratio = CDM_results(i) / CDM_results(1);
                expected_ratio = dMdt_values(i) / dMdt_values(1);
                testCase.verifyEqual(ratio, expected_ratio, 'AbsTol', 1e-5);
            end
        end
        
        function testThrustDampingMomentArm(testCase)
            % Test thrust damping scaling with moment arm (totalLength - CM)
            Calpha_small = [1e-6, 1e-6, 1e-6];
            dMdt = 10;
            
            % Test with different CM positions
            CM_positions = [0.5, 1.0, 1.5, 1.8];
            CDM_results = zeros(size(CM_positions));
            
            for i = 1:length(CM_positions)
                CDM_results(i) = pitchDampingMoment(testCase.testRocket, ...
                    testCase.testRho, Calpha_small, testCase.testCP, dMdt, ...
                    CM_positions(i), testCase.testW, testCase.testV);
            end
            
            % Should scale with (totalLength - CM)^2
            moment_arms = testCase.testRocket.totalLength - CM_positions;
            for i = 2:length(CM_positions)
                ratio = CDM_results(i) / CDM_results(1);
                expected_ratio = (moment_arms(i)^2) / (moment_arms(1)^2);
                testCase.verifyEqual(ratio, expected_ratio, 'AbsTol', 1e-6);
            end
        end
        
        function testThrustDampingVelocityScaling(testCase)
            % Test thrust damping scales with 1/V^2
            Calpha_small = [1e-6, 1e-6, 1e-6];
            dMdt = 10;
            
            v_values = [50, 100, 200, 400];
            CDM_results = zeros(size(v_values));
            
            for i = 1:length(v_values)
                CDM_results(i) = pitchDampingMoment(testCase.testRocket, ...
                    testCase.testRho, Calpha_small, testCase.testCP, dMdt, ...
                    testCase.testCM, testCase.testW, v_values(i));
            end
            
            % Should scale with 1/V^2
            for i = 2:length(v_values)
                ratio = CDM_results(i) / CDM_results(1);
                expected_ratio = (v_values(1)^2) / (v_values(i)^2);
                testCase.verifyEqual(ratio, expected_ratio, 'AbsTol', 1e-6);
            end
        end
        
        function testThrustDampingSign(testCase)
            % Test sign of thrust damping for positive and negative mass change
            Calpha_small = [1e-6, 1e-6, 1e-6];
            
            % Positive dMdt (burning) -> positive damping (destabilizing)
            CDM_pos = pitchDampingMoment(testCase.testRocket, testCase.testRho, ...
                Calpha_small, testCase.testCP, 10, testCase.testCM, ...
                testCase.testW, testCase.testV);
            testCase.verifyGreaterThan(CDM_pos, 0);
            
            % Negative dMdt (jettison) -> negative damping (stabilizing)
            CDM_neg = pitchDampingMoment(testCase.testRocket, testCase.testRho, ...
                Calpha_small, testCase.testCP, -10, testCase.testCM, ...
                testCase.testW, testCase.testV);
            testCase.verifyLessThan(CDM_neg, 0);
        end
        
        function testThrustDampingDensityScaling(testCase)
            % Test thrust damping scales with 1/rho
            Calpha_small = [1e-6, 1e-6, 1e-6];
            dMdt = 10;
            
            rho_values = [0.5, 1.0, 1.225, 2.0];
            CDM_results = zeros(size(rho_values));
            
            for i = 1:length(rho_values)
                CDM_results(i) = pitchDampingMoment(testCase.testRocket, ...
                    rho_values(i), Calpha_small, testCase.testCP, dMdt, ...
                    testCase.testCM, testCase.testW, testCase.testV);
            end
            
            % Should scale inversely with rho
            for i = 2:length(rho_values)
                ratio = CDM_results(i) / CDM_results(1);
                expected_ratio = rho_values(1) / rho_values(i);
                testCase.verifyEqual(ratio, expected_ratio, 'AbsTol', 1e-6);
            end
        end
        
    end
    
    methods (Test, TestTags = {'EdgeCases'})
        
        function testNegativeAngularVelocity(testCase)
            % Test with negative angular velocity
            CDM_pos = pitchDampingMoment(testCase.testRocket, testCase.testRho, ...
                testCase.testCalpha, testCase.testCP, 0, ...
                testCase.testCM, 10, testCase.testV);
            
            CDM_neg = pitchDampingMoment(testCase.testRocket, testCase.testRho, ...
                testCase.testCalpha, testCase.testCP, 0, ...
                testCase.testCM, -10, testCase.testV);
            
            % Should be opposite signs
            testCase.verifyEqual(CDM_pos, -CDM_neg, 'AbsTol', 1e-10);
        end
        
        function testCenterOfMassOutsideRocket(testCase)
            % Test with CM outside rocket bounds
            CM_outside = 2.5; % Beyond rocket length
            
            CDM = pitchDampingMoment(testCase.testRocket, testCase.testRho, ...
                testCase.testCalpha, testCase.testCP, testCase.testdMdt, ...
                CM_outside, testCase.testW, testCase.testV);
            
            % Should still work (though physically unrealistic)
            testCase.verifyThat(CDM, matlab.unittest.constraints.IsFinite);
        end
        
        function testAllComponentsAtCM(testCase)
            % Test when all CPs are at CM (should give zero aero damping)
            CP_at_CM = [testCase.testCM, testCase.testCM, testCase.testCM];
            
            CDM = pitchDampingMoment(testCase.testRocket, testCase.testRho, ...
                testCase.testCalpha, CP_at_CM, 0, ...
                testCase.testCM, testCase.testW, testCase.testV);
            
            % Aero damping should be zero
            testCase.verifyEqual(CDM, 0, 'AbsTol', 1e-10);
        end
        
        function testExtremeDensity(testCase)
            % Test with extreme density values
            rho_values = [0.01, 100];
            
            for i = 1:length(rho_values)
                CDM = pitchDampingMoment(testCase.testRocket, rho_values(i), ...
                    testCase.testCalpha, testCase.testCP, testCase.testdMdt, ...
                    testCase.testCM, testCase.testW, testCase.testV);
                
                testCase.verifyThat(CDM, matlab.unittest.constraints.IsFinite);
            end
        end
        
        function testExtremeAngularVelocity(testCase)
            % Test with extreme angular velocity
            w_values = [0.1, 1000];
            
            for i = 1:length(w_values)
                CDM = pitchDampingMoment(testCase.testRocket, testCase.testRho, ...
                    testCase.testCalpha, testCase.testCP, testCase.testdMdt, ...
                    testCase.testCM, w_values(i), testCase.testV);
                
                testCase.verifyThat(CDM, matlab.unittest.constraints.IsFinite);
            end
        end
        
        function testZeroCalpha(testCase)
            % Test with zero lift coefficients
            Calpha_zero = [0, 0, 0];
            
            CDM = pitchDampingMoment(testCase.testRocket, testCase.testRho, ...
                Calpha_zero, testCase.testCP, testCase.testdMdt, ...
                testCase.testCM, testCase.testW, testCase.testV);
            
            % Should have only thrust damping
            CNa_Total = sum(Calpha_zero .* (testCase.testCP - testCase.testCM).^2);
            expected_aero = -CNa_Total * testCase.testW / testCase.testV;
            
            expected_thrust = testCase.testdMdt * ...
                (testCase.testRocket.totalLength - testCase.testCM)^2 * ...
                testCase.testW * 2 / testCase.testV^2 / testCase.testRho / ...
                testCase.testRocket.maxCrossSectionArea;
            
            expected = expected_aero + expected_thrust;
            
            testCase.verifyEqual(CDM, expected, 'AbsTol', 1e-10);
        end
        
    end
end