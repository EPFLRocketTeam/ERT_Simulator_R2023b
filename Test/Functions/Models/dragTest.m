classdef dragTest < matlab.unittest.TestCase
    % Test class for the drag function.
    % To run, type 'runtests('dragTest')' in the Command Window.

    % Private property to hold the path that is added temporarily
    properties (Access = private)
        AddedPath;
        TestRocket;
        StandardKinematicViscosity = 1.5e-5; % Standard air at 20°C [m2/s]
        StandardSpeedOfSound = 343; % Standard air at 20°C [m/s]
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
            
            % 6. Create a standard test rocket configuration
            testCase.createStandardTestRocket();
        end
    end
    
    methods (TestClassTeardown)
        function removeFunctionPath(testCase)
            % This removes the path added in TestClassSetup
            rmpath(testCase.AddedPath);
        end
    end
    
    methods (Access = private)
        function createStandardTestRocket(testCase)
            % Create a standard rocket configuration for testing
            rocket.maxDiameter = 0.1; % maximum diameter [m]
            rocket.maxCrossSectionArea = pi * (rocket.maxDiameter/2)^2; % maximum cross-sectional area [m2]
            rocket.meanFinChord = 0.15; % fin chord [m]
            rocket.exposedFinArea = 0.02; % exposed planform fin area [m2]
            rocket.finBodyDiameter = 0.1; % body diameter at fin station [m]
            rocket.virtualFinArea = 0.025; % virtual fin planform area [m2]
            rocket.stagePositions = [0, 0.3, 1.0, 1.5]; % stage positions [m]
            rocket.stageDiameters = [0.05, 0.1, 0.1, 0.08]; % stageDiameters at stagePositions [m]
            rocket.numFins = 4; % number of fins
            rocket.finThickness = 0.003; % fin thickness [m]
            rocket.finSpan = 0.05; % fin span [m]
            rocket.finRootChord = 0.1; % fin root chord [m]
            rocket.finTipChord = 0.08; % fin tip chord [m]
            rocket.finLeadingEdgeLength = 0.12; % fin leading edge length [m]
            rocket.finTrailingEdgeLength = 0.1; % fin trailing edge length [m]
            rocket.finRootPosition = 1.2; % fin position from nose [m]
            rocket.numLaunchLugs = 2; % number of launch lugs
            rocket.lugSurfaceArea = 0.001; % launch lug area [m2]
            rocket.coneMode = 'on'; % nosecone mode
            rocket.motor_state = 'off'; % motor state
            rocket.motor_dia = 0.04; % motor diameter [m]
            
            testCase.TestRocket = rocket;
        end
    end

    methods (Test)
        
        function testZeroAngleOfAttackSubsonic(testCase)
            % Test with zero angle of attack and subsonic velocity
            rocket = testCase.TestRocket;
            angleOfAttack = 0; % [rad]
            freestreamVelocity = 50; % [m/s], subsonic
            kinematicViscosity = testCase.StandardKinematicViscosity;
            speedOfSound = testCase.StandardSpeedOfSound;
            
            dragCoefficient = drag(rocket, angleOfAttack, freestreamVelocity, kinematicViscosity, speedOfSound);
            
            % Verify the result is a positive scalar
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsGreaterThan(0));
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsFinite);
            testCase.verifyNumElements(dragCoefficient, 1);
        end
        
        function testNonZeroAngleOfAttackSubsonic(testCase)
            % Test with non-zero angle of attack and subsonic velocity
            rocket = testCase.TestRocket;
            angleOfAttack = deg2rad(10); % 10 degrees [rad]
            freestreamVelocity = 80; % [m/s], subsonic
            kinematicViscosity = testCase.StandardKinematicViscosity;
            speedOfSound = testCase.StandardSpeedOfSound;
            
            dragCoefficient = drag(rocket, angleOfAttack, freestreamVelocity, kinematicViscosity, speedOfSound);
            
            % Verify the result is a positive scalar
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsGreaterThan(0));
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsFinite);
        end
        
        function testVeryLowVelocity(testCase)
            % Test the divergence condition for very low velocity
            rocket = testCase.TestRocket;
            angleOfAttack = 0; % [rad]
            freestreamVelocity = 0.05; % [m/s], below threshold
            kinematicViscosity = testCase.StandardKinematicViscosity;
            speedOfSound = testCase.StandardSpeedOfSound;
            
            dragCoefficient = drag(rocket, angleOfAttack, freestreamVelocity, kinematicViscosity, speedOfSound);
            
            % Should still return a valid result due to divergence handling
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsGreaterThan(0));
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsFinite);
        end
        
        function testTransonicRegion(testCase)
            % Test in transonic region
            rocket = testCase.TestRocket;
            angleOfAttack = deg2rad(5); % [rad]
            freestreamVelocity = 300; % [m/s], transonic
            kinematicViscosity = testCase.StandardKinematicViscosity;
            speedOfSound = testCase.StandardSpeedOfSound;
            
            dragCoefficient = drag(rocket, angleOfAttack, freestreamVelocity, kinematicViscosity, speedOfSound);
            
            % Verify the result is a positive scalar
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsGreaterThan(0));
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsFinite);
        end
        
        function testSupersonicRegion(testCase)
            % Test in supersonic region
            rocket = testCase.TestRocket;
            angleOfAttack = deg2rad(2); % [rad]
            freestreamVelocity = 500; % [m/s], supersonic
            kinematicViscosity = testCase.StandardKinematicViscosity;
            speedOfSound = testCase.StandardSpeedOfSound;
            
            dragCoefficient = drag(rocket, angleOfAttack, freestreamVelocity, kinematicViscosity, speedOfSound);
            
            % Verify the result is a positive scalar
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsGreaterThan(0));
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsFinite);
        end
        
        function testNegativeAngleOfAttack(testCase)
            % Test with negative angle of attack (should use absolute value)
            rocket = testCase.TestRocket;
            angleOfAttack = deg2rad(-15); % [rad]
            freestreamVelocity = 100; % [m/s]
            kinematicViscosity = testCase.StandardKinematicViscosity;
            speedOfSound = testCase.StandardSpeedOfSound;
            
            dragCoefficient = drag(rocket, angleOfAttack, freestreamVelocity, kinematicViscosity, speedOfSound);
            
            % Verify the result is a positive scalar
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsGreaterThan(0));
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsFinite);
        end
        
        function testDifferentRocketConfigurations(testCase)
            % Test with different rocket configurations
            rocket = testCase.TestRocket;
            
            % Test with different number of fins
            rocketVariants = cell(3, 1);
            
            % Variant 1: 3 fins
            rocket1 = rocket;
            rocket1.numFins = 3;
            rocketVariants{1} = rocket1;
            
            % Variant 2: 6 fins  
            rocket2 = rocket;
            rocket2.numFins = 6;
            rocketVariants{2} = rocket2;
            
            % Variant 3: No nosecone
            rocket3 = rocket;
            rocket3.coneMode = 'off';
            rocketVariants{3} = rocket3;
            
            angleOfAttack = deg2rad(5);
            freestreamVelocity = 100;
            kinematicViscosity = testCase.StandardKinematicViscosity;
            speedOfSound = testCase.StandardSpeedOfSound;
            
            for i = 1:length(rocketVariants)
                dragCoefficient = drag(rocketVariants{i}, angleOfAttack, freestreamVelocity, kinematicViscosity, speedOfSound);
                
                testCase.verifyThat(dragCoefficient, ...
                    matlab.unittest.constraints.IsGreaterThan(0));
                testCase.verifyThat(dragCoefficient, ...
                    matlab.unittest.constraints.IsFinite);
            end
        end
        
        function testMotorOnState(testCase)
            % Test with motor on (affects base area calculation)
            rocket = testCase.TestRocket;
            rocket.motor_state = 'on';
            
            angleOfAttack = 0;
            freestreamVelocity = 200;
            kinematicViscosity = testCase.StandardKinematicViscosity;
            speedOfSound = testCase.StandardSpeedOfSound;
            
            dragCoefficient = drag(rocket, angleOfAttack, freestreamVelocity, kinematicViscosity, speedOfSound);
            
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsGreaterThan(0));
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsFinite);
        end
        
        function testBoundaryConditions(testCase)
            % Test boundary conditions for Reynolds number transitions
            rocket = testCase.TestRocket;
            angleOfAttack = 0;
            
            % Test with very low kinematic viscosity (high Reynolds number)
            lowKinematicViscosity = 1e-7; % [m2/s]
            freestreamVelocity = 50;
            speedOfSound = testCase.StandardSpeedOfSound;
            
            dragCoefficient = drag(rocket, angleOfAttack, freestreamVelocity, lowKinematicViscosity, speedOfSound);
            
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsGreaterThan(0));
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsFinite);
        end
        
        function testConsistencyAcrossVelocities(testCase)
            % Test that drag coefficient changes consistently with velocity
            rocket = testCase.TestRocket;
            angleOfAttack = deg2rad(10);
            kinematicViscosity = testCase.StandardKinematicViscosity;
            speedOfSound = testCase.StandardSpeedOfSound;
            
            velocities = [50, 100, 200, 300]; % [m/s]
            dragCoefficients = zeros(size(velocities));
            
            for i = 1:length(velocities)
                dragCoefficients(i) = drag(rocket, angleOfAttack, velocities(i), kinematicViscosity, speedOfSound);
            end
            
            for i = 1:length(dragCoefficients)
                testCase.verifyThat(dragCoefficients(i), ...
                    matlab.unittest.constraints.IsGreaterThan(0));
                testCase.verifyThat(dragCoefficients(i), ...
                    matlab.unittest.constraints.IsFinite);
            end
            
            % Should not be all the same (drag changes with velocity)
            % Use a simple check that they're not identical
            isNotConstant = ~all(abs(diff(dragCoefficients)) < 1e-10);
            testCase.verifyTrue(isNotConstant, ...
                'Drag coefficients should vary with velocity');
            % All should be positive and finite
            % testCase.verifyThat(dragCoefficients, ...
            %     matlab.unittest.constraints.EveryElementOf(...
            %     matlab.unittest.constraints.IsGreaterThan(0)));
            % testCase.verifyThat(dragCoefficients, ...
            %     matlab.unittest.constraints.EveryElementOf(...
            %     matlab.unittest.constraints.IsFinite));
            % 
            % % Should not be all the same (drag changes with velocity)
            % testCase.verifyThat(dragCoefficients, ...
            %     ~matlab.unittest.constraints.IsEqualTo(...
            %     dragCoefficients(1) * ones(size(dragCoefficients)), ...
            %     'Within', matlab.unittest.constraints.RelativeTolerance(0.01)));
        end
        
        function testExtremeAngleOfAttack(testCase)
            % Test with extreme angle of attack
            rocket = testCase.TestRocket;
            angleOfAttack = deg2rad(45); % Large angle of attack [rad]
            freestreamVelocity = 100; % [m/s]
            kinematicViscosity = testCase.StandardKinematicViscosity;
            speedOfSound = testCase.StandardSpeedOfSound;
            
            dragCoefficient = drag(rocket, angleOfAttack, freestreamVelocity, kinematicViscosity, speedOfSound);
            
            % Should be limited by tumbling drag coefficient
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsGreaterThan(0));
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsFinite);
        end
        
        function testDifferentAtmosphericConditions(testCase)
            % Test with different atmospheric conditions (viscosity and speed of sound)
            rocket = testCase.TestRocket;
            angleOfAttack = deg2rad(5);
            freestreamVelocity = 150;
            
            % High altitude conditions (lower density, lower temperature)
            highAltitudeKinematicViscosity = 5e-4; % [m2/s]
            highAltitudeSpeedOfSound = 295; % [m/s]
            
            dragCoefficient = drag(rocket, angleOfAttack, freestreamVelocity, ...
                highAltitudeKinematicViscosity, highAltitudeSpeedOfSound);
            
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsGreaterThan(0));
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsFinite);
        end
        
        function testRocketWithShortLength(testCase)
            % Test with unusually short rocket (triggering warning)
            rocket = testCase.TestRocket;
            % rocket.stagePositions = [0, 0.15, 0.3]; % Very short rocket [m]
            % rocket.stageDiameters = [0.05, 0.1, 0.08]; % [m]
            rocket.stagePositions = [0, 0.1, 0.2, 0.3];
            rocket.stageDiameters = [0.05, 0.1, 0.1, 0.08];
            
            angleOfAttack = 0;
            freestreamVelocity = 100;
            kinematicViscosity = testCase.StandardKinematicViscosity;
            speedOfSound = testCase.StandardSpeedOfSound;
            
            % This should work but might trigger warnings
            dragCoefficient = drag(rocket, angleOfAttack, freestreamVelocity, kinematicViscosity, speedOfSound);
            
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsGreaterThan(0));
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsFinite);
        end
         function testMinimalValidRocket(testCase)
            % Test with minimal valid rocket configuration
            minimalRocket.maxDiameter = 0.1;
            minimalRocket.maxCrossSectionArea = pi * (0.1/2)^2;
            minimalRocket.meanFinChord = 0.1;
            minimalRocket.exposedFinArea = 0.01;
            minimalRocket.finBodyDiameter = 0.1;
            minimalRocket.virtualFinArea = 0.015;
            minimalRocket.stagePositions = [0, 0.2, 0.5, 0.7];
            minimalRocket.stageDiameters = [0.05, 0.1, 0.1, 0.09];
            minimalRocket.numFins = 3;
            minimalRocket.finThickness = 0.002;
            minimalRocket.finSpan = 0.04;
            minimalRocket.finRootChord = 0.08;
            minimalRocket.finTipChord = 0.06;
            minimalRocket.finLeadingEdgeLength = 0.1;
            minimalRocket.finTrailingEdgeLength = 0.08;
            minimalRocket.finRootPosition = 0.6; % ADDED: required field
            minimalRocket.numLaunchLugs = 1;
            minimalRocket.lugSurfaceArea = 0.0005;
            minimalRocket.coneMode = 'on';
            minimalRocket.motor_state = 'off';
            minimalRocket.motor_dia = 0.03;
            
            angleOfAttack = 0;
            freestreamVelocity = 100;
            kinematicViscosity = testCase.StandardKinematicViscosity;
            speedOfSound = testCase.StandardSpeedOfSound;
            
            dragCoefficient = drag(minimalRocket, angleOfAttack, freestreamVelocity, kinematicViscosity, speedOfSound);
            
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsGreaterThan(0));
            testCase.verifyThat(dragCoefficient, ...
                matlab.unittest.constraints.IsFinite);
        end
    end
end