classdef dragTransonicTest < matlab.unittest.TestCase
    % Test class for the dragTransonic function.
    % To run, type 'runtests('dragTransonicTest')' in the Command Window.

    % Private properties to hold the path and test data
    properties (Access = private)
        AddedPath
        TestRocket
        KinematicViscosity
        SpeedOfSound
    end
    
    methods (TestClassSetup)
        function setupTestEnvironment(testCase)
            % This function sets up the test environment by adding the function path
            % and creating test data
            
            % 1. Get the path of the current test file directory
            testDir = fileparts(mfilename('fullpath'));
            
            % 2. Move up to the root folder
            rootPath = fileparts(fileparts(fileparts(testDir)));
            
            % 3. Construct the path to the function file's directory
            functionPath = fullfile(rootPath, 'Src', 'Functions', 'Models');
            
            % 4. Add the path to MATLAB's search path
            addpath(functionPath);
            
            % 5. Store the path so we can remove it later
            testCase.AddedPath = functionPath;
            
            % 6. Create a standard test rocket configuration
            testCase.TestRocket = testCase.createStandardRocket();
            
            % 7. Set standard atmospheric properties
            testCase.KinematicViscosity = 1.81e-5; % m²/s
            testCase.SpeedOfSound = 343; % m/s
        end
    end
    
    methods (TestClassTeardown)
        function teardownTestEnvironment(testCase)
            % This removes the path added in TestClassSetup
            rmpath(testCase.AddedPath);
        end
    end

    % --- Test Methods ---
    methods (Test)
        
        function testSubsonicCase(testCase)
            % Test subsonic regime (M < drag divergence Mach)
            freestreamVelocity = 200; % m/s (M ≈ 0.58)
            angleOfAttack = 0;
            
            dragCoefficient = dragTransonic(testCase.TestRocket, angleOfAttack, ...
                freestreamVelocity, testCase.KinematicViscosity, testCase.SpeedOfSound);
            
            % Verify reasonable drag coefficient for subsonic flow
            testCase.verifyGreaterThan(dragCoefficient, 0.1, ...
                'Subsonic drag should be positive');
            testCase.verifyLessThan(dragCoefficient, 0.8, ...
                'Subsonic drag should be reasonable');
        end
        
        function testTransonicCase(testCase)
            % Test transonic regime (M between drag divergence and final transonic Mach)
            freestreamVelocity = 320; % m/s (M ≈ 0.93)
            angleOfAttack = 0;
            
            dragCoefficient = dragTransonic(testCase.TestRocket, angleOfAttack, ...
                freestreamVelocity, testCase.KinematicViscosity, testCase.SpeedOfSound);
            
            % Verify transonic drag is higher than subsonic
            testCase.verifyGreaterThan(dragCoefficient, 0.2, ...
                'Transonic drag should be significant');
        end
        
        function testSupersonicCase(testCase)
            % Test supersonic regime (M > final transonic Mach)
            freestreamVelocity = 600; % m/s (M ≈ 1.75)
            angleOfAttack = 0;
            
            dragCoefficient = dragTransonic(testCase.TestRocket, angleOfAttack, ...
                freestreamVelocity, testCase.KinematicViscosity, testCase.SpeedOfSound);
            
            % Verify supersonic drag coefficient
            testCase.verifyGreaterThan(dragCoefficient, 0.1, ...
                'Supersonic drag should be positive');
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsFinite, ...
                'Drag coefficient should be finite');
        end
        
        function testZeroAngleOfAttack(testCase)
            % Test with zero angle of attack
            freestreamVelocity = 300; % m/s
            angleOfAttack = 0;
            
            dragCoefficient = dragTransonic(testCase.TestRocket, angleOfAttack, ...
                freestreamVelocity, testCase.KinematicViscosity, testCase.SpeedOfSound);
            
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsFinite, ...
                'Zero AoA drag should be finite');
        end
        
        function testNonZeroAngleOfAttack(testCase)
            % Test with non-zero angle of attack
            freestreamVelocity = 300; % m/s
            angleOfAttack = 5 * pi/180; % 5 degrees in radians
            
            dragCoefficient = dragTransonic(testCase.TestRocket, angleOfAttack, ...
                freestreamVelocity, testCase.KinematicViscosity, testCase.SpeedOfSound);
            
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsFinite, ...
                'Non-zero AoA drag should be finite');
        end
        
        function testLowVelocityCutoff(testCase)
            % Test the low velocity cutoff (Uinf < 0.1 m/s)
            freestreamVelocity = 0.05; % m/s (below cutoff)
            angleOfAttack = 0;
            
            dragCoefficient = dragTransonic(testCase.TestRocket, angleOfAttack, ...
                freestreamVelocity, testCase.KinematicViscosity, testCase.SpeedOfSound);
            
            % Should use 0.1 m/s internally, so result should be finite
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsFinite, ...
                'Low velocity case should handle cutoff');
        end
        
        function testBoundaryLayerTransition(testCase)
            % Test cases that trigger different boundary layer regimes
            % Use very low Reynolds number to force laminar flow
            highViscosity = 1.0; % Very high kinematic viscosity
            freestreamVelocity = 100; % m/s
            angleOfAttack = 0;
            
            dragCoefficient = dragTransonic(testCase.TestRocket, angleOfAttack, ...
                freestreamVelocity, highViscosity, testCase.SpeedOfSound);
            
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsFinite, ...
                'Laminar flow case should be finite');
        end
        
        function testRocketWithLongNose(testCase)
            % Test rocket configuration that triggers the warning (Ln/Le > 0.6)
            longNoseRocket = testCase.createLongNoseRocket();
            freestreamVelocity = 300; % m/s
            angleOfAttack = 0;
            
            % This should trigger the warning about bad approximation
            dragCoefficient = dragTransonic(longNoseRocket, angleOfAttack, ...
                freestreamVelocity, testCase.KinematicViscosity, testCase.SpeedOfSound);
            
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsFinite, ...
                'Long nose rocket should still compute finite drag');
        end
        
        function testRocketWithShortBody(testCase)
            % Test rocket with short body (affects wave drag calculation)
            shortRocket = testCase.createShortRocket();
            freestreamVelocity = 400; % m/s
            angleOfAttack = 0;
            
            dragCoefficient = dragTransonic(shortRocket, angleOfAttack, ...
                freestreamVelocity, testCase.KinematicViscosity, testCase.SpeedOfSound);
            
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsFinite, ...
                'Short rocket should compute finite drag');
        end
        
        function testExtremeMachNumbers(testCase)
            % Test very high Mach number
            freestreamVelocity = 1000; % m/s (M ≈ 2.92)
            angleOfAttack = 0;
            
            dragCoefficient = dragTransonic(testCase.TestRocket, angleOfAttack, ...
                freestreamVelocity, testCase.KinematicViscosity, testCase.SpeedOfSound);
            
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsFinite, ...
                'High Mach number should be finite');
        end
        
        function testNegativeAngleOfAttack(testCase)
            % Test with negative angle of attack (should use absolute value)
            freestreamVelocity = 300; % m/s
            angleOfAttack = -5 * pi/180; % -5 degrees in radians
            
            dragCoefficient = dragTransonic(testCase.TestRocket, angleOfAttack, ...
                freestreamVelocity, testCase.KinematicViscosity, testCase.SpeedOfSound);
            
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsFinite, ...
                'Negative AoA should be handled');
        end
        
        function testMultipleAngleOfAttackValues(testCase)
            % Test consistency across different angles of attack
            freestreamVelocity = 300; % m/s
            angles = [0, 2, 5, 10] * pi/180; % Various angles in radians
            
            dragCoefficients = zeros(size(angles));
            for i = 1:length(angles)
                dragCoefficients(i) = dragTransonic(testCase.TestRocket, angles(i), ...
                    freestreamVelocity, testCase.KinematicViscosity, testCase.SpeedOfSound);
            end
            
            % Drag should generally increase with angle of attack
            for i = 2:length(dragCoefficients)
                testCase.verifyGreaterThanOrEqual(dragCoefficients(i), dragCoefficients(1), ...
                    'Drag should not decrease with increasing AoA');
            end
            
            % All values should be finite
            % testCase.verifyThat(dragCoefficients, ...
            %     matlab.unittest.constraints.EveryElementOf(...
            %     matlab.unittest.constraints.IsFinite), ...
            %     'All AoA cases should produce finite drag');
            for i = 1:length(dragCoefficients)
                testCase.verifyThat(dragCoefficients(i), ...
                    matlab.unittest.constraints.IsFinite, ...
                    sprintf('AoA case %d should produce finite drag', i));
            end
        end
        
        function testDifferentReynoldsNumbers(testCase)
            % Test sensitivity to Reynolds number (via kinematic viscosity)
            freestreamVelocity = 300; % m/s
            angleOfAttack = 0;
            
            % Test with different viscosities
            lowViscosity = 1e-6; % Very low viscosity (high Re)
            highViscosity = 1e-1; % Very high viscosity (low Re)
            
            dragLowVisc = dragTransonic(testCase.TestRocket, angleOfAttack, ...
                freestreamVelocity, lowViscosity, testCase.SpeedOfSound);
            
            dragHighVisc = dragTransonic(testCase.TestRocket, angleOfAttack, ...
                freestreamVelocity, highViscosity, testCase.SpeedOfSound);
            
            testCase.verifyThat(dragLowVisc, ...
                matlab.unittest.constraints.IsFinite, ...
                'Low viscosity case should be finite');
            testCase.verifyThat(dragHighVisc, ...
                matlab.unittest.constraints.IsFinite, ...
                'High viscosity case should be finite');
        end
        
        function testMachNumberBoundaries(testCase)
            % Test Mach numbers near critical boundaries
            % Calculate the drag divergence Mach for the test rocket
            noseLength = testCase.TestRocket.stagePositions(2);
            maxDiameter = testCase.TestRocket.maxDiameter;
            dragDivergenceMach = -0.0156 * (noseLength / maxDiameter)^2 + ...
                0.136 * (noseLength / maxDiameter) + 0.6817;
            
            % Test just below and just above drag divergence
            machBelow = dragDivergenceMach - 0.05;
            machAbove = dragDivergenceMach + 0.05;
            
            velocityBelow = machBelow * testCase.SpeedOfSound;
            velocityAbove = machAbove * testCase.SpeedOfSound;
            angleOfAttack = 0;
            
            dragBelow = dragTransonic(testCase.TestRocket, angleOfAttack, ...
                velocityBelow, testCase.KinematicViscosity, testCase.SpeedOfSound);
            
            dragAbove = dragTransonic(testCase.TestRocket, angleOfAttack, ...
                velocityAbove, testCase.KinematicViscosity, testCase.SpeedOfSound);
            
            testCase.verifyThat(dragBelow, ...
                matlab.unittest.constraints.IsFinite, ...
                'Below divergence Mach should be finite');
            testCase.verifyThat(dragAbove, ...
                matlab.unittest.constraints.IsFinite, ...
                'Above divergence Mach should be finite');
        end
    end
    
    % --- Helper Methods for Creating Test Rockets ---
    methods (Static, Access = private)
        function rocket = createStandardRocket()
            % Create a standard rocket configuration for testing
            rocket = struct();
            
            % Body geometry
            rocket.stagePositions = [0, 0.5, 2.0, 2.2, 2.3]; % Stage positions [m]
            rocket.maxDiameter = 0.1; % Maximum diameter [m]
            rocket.stageDiameters = [0, rocket.maxDiameter, rocket.maxDiameter, 0.08, 0.07]; % Diameters at stages [m]
            
            % Fin geometry
            rocket.finRootChord = 0.15; % Root chord [m]
            rocket.finTipChord = 0.10; % Tip chord [m]
            rocket.numFins = 4; % Number of fins
            rocket.finThickness = 0.003; % Fin thickness [m]
            rocket.finLeadingEdgeLength = 0.05; % Distance to max thickness [m]
            rocket.virtualFinArea = 0.06; % Fin wetted area [m²]
            rocket.finRootPosition = 1.8; % Fin leading edge position [m]
        end
        
        function rocket = createLongNoseRocket()
            % Create a rocket with long nose (Ln/Le > 0.6)
            rocket = dragTransonicTest.createStandardRocket();
            rocket.stagePositions = [0, 1.8, 2.0, 2.2, 2.3]; % Very long nose
            rocket.stageDiameters = [0, rocket.maxDiameter, rocket.maxDiameter, 0.08, 0.07];
        end
        
        function rocket = createShortRocket()
            % Create a rocket with short body
            rocket = dragTransonicTest.createStandardRocket();
            rocket.stagePositions = [0, 0.3, 0.8, 1.0, 1.1]; % Short rocket
            rocket.stageDiameters = [0, rocket.maxDiameter, rocket.maxDiameter, 0.08, 0.07];
        end
    end
end