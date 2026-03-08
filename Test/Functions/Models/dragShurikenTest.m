classdef dragShurikenTest < matlab.unittest.TestCase
    
    properties
        AddedPath;
        testRocket;
        testTheta;
        testAlpha;
        testUinf;
        testNu;
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
            % Create a standard test rocket
            testCase.testRocket = struct(...
                'airbrakePosition', 0.5, ... % meters from nose
                'numAirbrakes', 3, ...
                'maxCrossSectionArea', pi*0.05^2); % m^2 (based on 0.1m diameter)
            
            testCase.testTheta = -116; % half open (mid-range)
            testCase.testAlpha = 0; % 0 deg angle of attack
            testCase.testUinf = 50; % m/s
            testCase.testNu = 1.5e-5; % m^2/s (typical air at sea level)
        end
    end
    
    methods (Test)
        function testBasicFunctionality(testCase)
            % Test basic functionality with nominal inputs
            CD = dragShuriken(testCase.testRocket, testCase.testTheta, ...
                               testCase.testAlpha, testCase.testUinf, testCase.testNu);
            
            % Verify output is positive and finite
            testCase.verifyGreaterThan(CD, 0);
            testCase.verifyThat(CD, matlab.unittest.constraints.IsFinite);
            
            % Drag coefficient should be reasonable (typically < 2 for airbrakes)
            testCase.verifyLessThan(CD, 2);
        end
        
        function testClosedPosition(testCase)
            % Test with airbrakes fully closed
            theta_closed = -232; % closed position
            
            CD = dragShuriken(testCase.testRocket, theta_closed, ...
                               testCase.testAlpha, testCase.testUinf, testCase.testNu);
            
            % When closed, drag should be very small (near zero)
            testCase.verifyLessThan(CD, 1e-3);
        end
        
        function testOpenPosition(testCase)
            % Test with airbrakes fully open
            theta_open = 0.9; % open position
            
            CD = dragShuriken(testCase.testRocket, theta_open, ...
                               testCase.testAlpha, testCase.testUinf, testCase.testNu);
            
            % Open position should produce maximum drag
            testCase.verifyGreaterThan(CD, 0.01);
        end
        
        function testAngleOfAttackEffect(testCase)
            % Test effect of angle of attack
            theta = -116; % half open
            
            % Test with different angles of attack
            CD_0deg = dragShuriken(testCase.testRocket, theta, 0, ...
                                     testCase.testUinf, testCase.testNu);
            CD_10deg = dragShuriken(testCase.testRocket, theta, 10*pi/180, ...
                                      testCase.testUinf, testCase.testNu);
            
            % Drag should decrease with angle of attack (cos(alpha) factor)
            testCase.verifyLessThan(CD_10deg, CD_0deg);
            
            % Check approximate relationship
            expected_ratio = cos(10*pi/180);
            actual_ratio = CD_10deg / CD_0deg;
            testCase.verifyEqual(actual_ratio, expected_ratio, 'AbsTol', 0.02);
        end
        
        function testVelocityEffect(testCase)
            % Test effect of velocity on boundary layer
            theta = -116;
            alpha = 0;
            
            % Test at different velocities
            CD_low = dragShuriken(testCase.testRocket, theta, alpha, 20, testCase.testNu);
            CD_high = dragShuriken(testCase.testRocket, theta, alpha, 100, testCase.testNu);
            
            % Both should be finite
            testCase.verifyThat(CD_low, matlab.unittest.constraints.IsFinite);
            testCase.verifyThat(CD_high, matlab.unittest.constraints.IsFinite);
            
            % Higher velocity should give different drag due to Rex effect
            testCase.verifyNotEqual(CD_low, CD_high);
        end
        
        function testNumberOfAirbrakes(testCase)
            % Test scaling with number of airbrakes
            theta = -116;
            alpha = 0;
            
            rocket_2brakes = testCase.testRocket;
            rocket_2brakes.numAirbrakes = 2;
            
            rocket_4brakes = testCase.testRocket;
            rocket_4brakes.numAirbrakes = 4;
            
            CD_2 = dragShuriken(rocket_2brakes, theta, alpha, testCase.testUinf, testCase.testNu);
            CD_4 = dragShuriken(rocket_4brakes, theta, alpha, testCase.testUinf, testCase.testNu);
            
            % Should scale linearly with number of airbrakes
            testCase.verifyEqual(CD_4 / CD_2, 2, 'AbsTol', 1e-6);
        end
        
        function testReferenceAreaScaling(testCase)
            % Test scaling with reference area
            theta = -116;
            alpha = 0;
            
            rocket_large = testCase.testRocket;
            rocket_large.maxCrossSectionArea = pi*0.1^2; % Double diameter
            
            rocket_small = testCase.testRocket;
            rocket_small.maxCrossSectionArea = pi*0.025^2; % Half diameter
            
            CD_large = dragShuriken(rocket_large, theta, alpha, testCase.testUinf, testCase.testNu);
            CD_small = dragShuriken(rocket_small, theta, alpha, testCase.testUinf, testCase.testNu);
            
            % Should be inversely proportional to reference area
            expected_ratio = rocket_large.maxCrossSectionArea / rocket_small.maxCrossSectionArea;
            testCase.verifyEqual(CD_small / CD_large, expected_ratio, 'AbsTol', 1e-6);
        end
        
    end
    
    methods (Test, TestTags = {'BoundaryLayer'})
        
        function testBoundaryLayerTransition(testCase)
            % Test the boundary layer transition behavior
            theta = -116; % Fixed position
            alpha = 0;
            
            % Values that give h < delta and h > delta cases
            % Need to manipulate Rex to get different delta values
            U_low = 1; % m/s - gives larger delta
            U_high = 200; % m/s - gives smaller delta
            
            CD_lowU = dragShuriken(testCase.testRocket, theta, alpha, U_low, testCase.testNu);
            CD_highU = dragShuriken(testCase.testRocket, theta, alpha, U_high, testCase.testNu);
            
            % Both should be finite
            testCase.verifyThat(CD_lowU, ...
                matlab.unittest.constraints.IsFinite);
            testCase.verifyThat(CD_highU, ...
                matlab.unittest.constraints.IsFinite);
        end
        
        function testViscosityEffect(testCase)
            % Test effect of different viscosities
            theta = -116;
            alpha = 0;
            
            % Test at different viscosities
            CD_lowNu = dragShuriken(testCase.testRocket, theta, alpha, ...
                                      testCase.testUinf, 1e-6); % Low viscosity
            CD_highNu = dragShuriken(testCase.testRocket, theta, alpha, ...
                                       testCase.testUinf, 2e-5); % High viscosity
            
            testCase.verifyThat(CD_lowNu, ...
                matlab.unittest.constraints.IsFinite);
            testCase.verifyThat(CD_highNu, ...
                matlab.unittest.constraints.IsFinite);
        end
        
    end
    
    methods (Test, TestTags = {'Interpolation'})
        
        function testSurfaceInterpolation(testCase)
            % Test the surface interpolation function indirectly
            % by checking various theta values
            
            % Test at known interpolation points (should match tabulated)
            theta_closed = -232; % maps to angle = 0
            theta_open = 0.9;    % maps to angle = 73
            
            CD_closed = dragShuriken(testCase.testRocket, theta_closed, ...
                                       testCase.testAlpha, testCase.testUinf, testCase.testNu);
            CD_open = dragShuriken(testCase.testRocket, theta_open, ...
                                     testCase.testAlpha, testCase.testUinf, testCase.testNu);
            
            % Open should have much higher drag than closed
            testCase.verifyGreaterThan(CD_open / CD_closed, 100);
        end
        
        function testInterpolationMonotonic(testCase)
            % Test that drag increases monotonically with theta
            theta_values = linspace(-232, 0.9, 10);
            CD_previous = 0;
            
            for i = 1:length(theta_values)
                CD = dragShuriken(testCase.testRocket, theta_values(i), ...
                                   testCase.testAlpha, testCase.testUinf, testCase.testNu);
                
                if i > 1
                    testCase.verifyGreaterThanOrEqual(CD, CD_previous);
                end
                CD_previous = CD;
            end
        end
        
    end
    
    methods (Test, TestTags = {'EdgeCases'})
        
        function testZeroVelocity(testCase)
            % Test with zero velocity
            theta = -116;
            alpha = 0;
            
            CD = dragShuriken(testCase.testRocket, theta, alpha, 0, testCase.testNu);
            
            % Should handle zero velocity (Rex = 0)
            testCase.verifyThat(CD, matlab.unittest.constraints.IsFinite);
        end
        
        function testExtremeViscosity(testCase)
            % Test with extreme viscosity values
            theta = -116;
            alpha = 0;
            
            % Very high viscosity
            CD_highNu = dragShuriken(testCase.testRocket, theta, alpha, ...
                                       testCase.testUinf, 1);
            testCase.verifyThat(CD_highNu, ...
                matlab.unittest.constraints.IsFinite);
            
            % Very low viscosity
            CD_lowNu = dragShuriken(testCase.testRocket, theta, alpha, ...
                                      testCase.testUinf, 1e-10);
            testCase.verifyThat(CD_lowNu, ...
                matlab.unittest.constraints.IsFinite);
        end
        
        function testExtremeAirbrakePosition(testCase)
            % Test with extreme airbrake positions
            rocket_near = testCase.testRocket;
            rocket_near.airbrakePosition = 0.01; % Very close to nose
            
            rocket_far = testCase.testRocket;
            rocket_far.airbrakePosition = 10; % Very far from nose
            
            theta = -116;
            alpha = 0;
            
            CD_near = dragShuriken(rocket_near, theta, alpha, testCase.testUinf, testCase.testNu);
            CD_far = dragShuriken(rocket_far, theta, alpha, testCase.testUinf, testCase.testNu);
            
            % Both should be finite
            testCase.verifyThat(CD_near, matlab.unittest.constraints.IsFinite);
            testCase.verifyThat(CD_far, matlab.unittest.constraints.IsFinite);
        end
        
    end
    
    methods (Test, TestTags = {'Validation'})
        
        function testSymmetry(testCase)
            % Test symmetry around zero angle of attack
            theta = -116;
            
            CD_positive = dragShuriken(testCase.testRocket, theta, 5*pi/180, ...
                                         testCase.testUinf, testCase.testNu);
            CD_negative = dragShuriken(testCase.testRocket, theta, -5*pi/180, ...
                                         testCase.testUinf, testCase.testNu);
            
            % Should be symmetric (cos is even)
            testCase.verifyEqual(CD_positive, CD_negative, 'AbsTol', 1e-6);
        end
        
        function testContinuousTransition(testCase)
            % Test that the function is continuous across theta range
            theta = linspace(-232, 0.9, 100);
            CD_values = zeros(size(theta));
            
            for i = 1:length(theta)
                CD_values(i) = dragShuriken(testCase.testRocket, theta(i), ...
                                             testCase.testAlpha, testCase.testUinf, testCase.testNu);
            end
            
            % Check for jumps (should be smooth)
            differences = abs(diff(CD_values));
            max_jump = max(differences);
            mean_jump = mean(differences);
            
            % No individual jump should be more than 10x the mean
            testCase.verifyLessThan(max_jump, 10 * mean_jump);
        end
        
    end
end