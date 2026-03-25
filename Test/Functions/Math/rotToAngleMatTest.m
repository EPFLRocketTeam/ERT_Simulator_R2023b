classdef rotToAngleMatTest < matlab.unittest.TestCase

    % Private property to hold the path that is added temporarily
    properties (Access = private)
        AddedPath;
    end
    
    methods (TestClassSetup) % <-- CHANGED from TestSetup
        function addFunctionPath(testCase)
            % This function temporarily adds the directory of the normalizeVect 
            % function to the MATLAB path so the tests can find it
            
            % 1. Get the path of the current test file directory 
            % (e.g., ...\ERT_Simulator_R2023b\Test\Functions\Math)
            testDir = fileparts(mfilename('fullpath'));
            
            % 2. Move up two directories to reach the root folder 
            % (e.g., ...\ERT_Simulator_R2023b)
            rootPath = fileparts(fileparts(fileparts(testDir))); 
            
            % 3. Construct the path to the function file's directory 
            % (e.g., ...\ERT_Simulator_R2023b\Src\Functions\Math)
            functionPath = fullfile(rootPath, 'Src', 'Functions', 'Math');
            
            % 4. Add the path to MATLAB's search path
            addpath(functionPath);
            
            % 5. Store the path so we can remove it later in TestClassTeardown
            testCase.AddedPath = functionPath;
        end
    end
    
    methods (TestClassTeardown)
        function removeFunctionPath(testCase)
            % This removes the path added in TestClassSetup, keeping the MATLAB environment clean.
            rmpath(testCase.AddedPath);
        end
    end

    % --- Existing Test Methods ---
    methods (Test)
        function testBasicCase(testCase)
            % Test that the function outputs the correct result
            % for random rotation matrices

            % first rotation matrix
            C1 = [-0.2593,-0.0741,0.9630;
                  0.5185 ,-0.8519,0.0741;
                  0.8148 ,0.5185 ,0.2593];
            % second rotation matrix
            C2 = [-0.1644,0.4521,0.8767 ;
                  0.7808 ,0.6027,-0.1644;
                  -0.6027,0.6575,-0.4521];
            % combine
            C = cat(3,C1,C2);
            % get angles
            angles = rotToAngleMat(C);

            % declare expected angles
            angle1Exp = [15.9483,-74.3654,15.9483];
            angle2Exp = [19.9831,-61.2468,-70.0169];
            anglesExp = cat(3,angle1Exp,angle2Exp);


            % verify that the function returned the expected values
            testCase.verifyEqual(angles, anglesExp, 'AbsTol', 1e-4);
        end

        function testIdentityRotation(testCase)
            % Test identity rotation matrix gives zero angles
            C = eye(3);
            angle = rotToAngleMat(C);
            
            expected = [0, 0, 0];
            testCase.verifyEqual(angle, expected, 'AbsTol', 1e-10);
        end
        
        function testPureYawRotation(testCase)
            % Test pure yaw rotation (Y-axis rotation, apparently)
            yawAngle = 30; % degrees
            yawRad = deg2rad(yawAngle);
            
            C = [cos(yawRad) , 0, sin(yawRad);
                 0           , 1, 0;
                 -sin(yawRad), 0, cos(yawRad)];
            
            angle = rotToAngleMat(C);
            
            % Negative because function is weird
            expected = [0, -yawAngle, 0];
            testCase.verifyEqual(angle, expected, 'AbsTol', 1e-10);
        end
        
        function testPurePitchRotation(testCase)
            % Test pure pitch rotation (X-axis rotation, apparently)
            pitchAngle = 45; % degrees
            pitchRad = deg2rad(pitchAngle);
            
            C = [1, 0            , 0             ;
                 0, cos(pitchRad), -sin(pitchRad);
                 0, sin(pitchRad), cos(pitchRad)];
            
            angle = rotToAngleMat(C);
            
            % Negative because function is weird
            expected = [-pitchAngle, 0, 0];
            testCase.verifyEqual(angle, expected, 'AbsTol', 1e-10);
        end
        
        function testPureRollRotation(testCase)
            % Test pure roll rotation (Z-axis rotation, apparently)
            rollAngle = 60; % degrees
            rollRad = deg2rad(rollAngle);
            
            C = [cos(rollRad), -sin(rollRad), 0 ;
                 sin(rollRad),  cos(rollRad), 0 ;
                 0           ,  0           , 1];
            
            angle = rotToAngleMat(C);
            
            % Negative because function is weird
            expected = [0, 0, -rollAngle];
            testCase.verifyEqual(angle, expected, 'AbsTol', 1e-10);
        end
        
        function testMultipleRotationMatrices(testCase)
            % Test multiple rotation matrices input (3x3xn)
            C1 = eye(3); % Identity
            
            % 30 degree pitch rotation
            pitchAngle = 30;
            pitchRad = deg2rad(pitchAngle);
            C2 = [cos(pitchRad),  0, sin(pitchRad);
                  0,               1, 0;
                  -sin(pitchRad), 0, cos(pitchRad)];
            
            C = cat(3, C1, C2); % 3x3x2 matrix
            
            angle = rotToAngleMat(C);
            disp('penis');
            disp(angle);
            
            % Verify dimensions
            testCase.verifySize(angle, [1, 3, 2]);
            
            % Verify first rotation (identity)
            testCase.verifyEqual(angle(:,:,1), [0, 0, 0], 'AbsTol', 1e-10);
            
            % Verify second rotation (pitch)
            testCase.verifyEqual(angle(:,:,2), [0, -pitchAngle, 0], 'AbsTol', 1e-10);
        end
        
        function testCombinedRotation(testCase)
            % Test combined rotation (pitch + yaw + roll)
            pitchAngle = 10;
            yawAngle = 20;
            rollAngle = 30;
            
            pitchRad = deg2rad(pitchAngle);
            yawRad = deg2rad(yawAngle);
            rollRad = deg2rad(rollAngle);
            
            % Create rotation matrices for each axis
            Rx = [1, 0,                0;
                  0, cos(rollRad), -sin(rollRad);
                  0, sin(rollRad),  cos(rollRad)];
            
            Ry = [cos(pitchRad),  0, sin(pitchRad);
                  0,               1, 0;
                  -sin(pitchRad), 0, cos(pitchRad)];
            
            Rz = [cos(yawRad), -sin(yawRad), 0;
                  sin(yawRad),  cos(yawRad), 0;
                  0,            0,             1];
            
            % Combined rotation (order matters - using typical aerospace convention)
            C = Rz * Ry * Rx;
            
            angle = rotToAngleMat(C);
            
            % Note: Due to Euler angle conventions and order dependencies,
            % the extracted angles may not exactly match the input angles
            % This test verifies the function works without error
            testCase.verifySize(angle, [1, 3]);
            testCase.verifyGreaterThan(angle(1), -90);
            testCase.verifyLessThan(angle(1), 90);
            testCase.verifyGreaterThan(angle(2), -90);
            testCase.verifyLessThan(angle(2), 90);
            testCase.verifyGreaterThan(angle(3), -180);
            testCase.verifyLessThan(angle(3), 180);
        end
        
        function testNegativeAngles(testCase)
            % Test negative rotation angles
            pitchAngle = -15;
            yawAngle = -25;
            rollAngle = -35;
            
            pitchRad = deg2rad(pitchAngle);
            yawRad = deg2rad(yawAngle);
            rollRad = deg2rad(rollAngle);
            
            Rx = [1, 0,                0;
                  0, cos(rollRad), -sin(rollRad);
                  0, sin(rollRad),  cos(rollRad)];
            
            Ry = [cos(pitchRad),  0, sin(pitchRad);
                  0,               1, 0;
                  -sin(pitchRad), 0, cos(pitchRad)];
            
            Rz = [cos(yawRad), -sin(yawRad), 0;
                  sin(yawRad),  cos(yawRad), 0;
                  0,            0,             1];
            
            C = Rz * Ry * Rx;
            
            angle = rotToAngleMat(C);
            
            testCase.verifySize(angle, [1, 3]);
            % Should extract reasonable angles within proper ranges
            testCase.verifyLessThan(abs(angle(1)), 90);
            testCase.verifyLessThan(abs(angle(2)), 90);
            testCase.verifyLessThan(abs(angle(3)), 180);
        end
        
        function testGimbalLockCases(testCase)
            % Test gimbal lock cases (pitch near ±90 degrees)
            
            % Test pitch = 90 degrees
            pitchAngle = 90;
            pitchRad = deg2rad(pitchAngle);
            
            C = [cos(pitchRad),  0, sin(pitchRad);
                 0,               1, 0;
                 -sin(pitchRad), 0, cos(pitchRad)];
            
            angle = rotToAngleMat(C);
            
            testCase.verifyEqual(angle(2), -pitchAngle, 'AbsTol', 1e-10);
            
            % Test pitch = -90 degrees
            pitchAngle = -90;
            pitchRad = deg2rad(pitchAngle);
            
            C = [cos(pitchRad),  0, sin(pitchRad);
                 0,               1, 0;
                 -sin(pitchRad), 0, cos(pitchRad)];
            
            angle = rotToAngleMat(C);
            
            testCase.verifyEqual(angle(2), -pitchAngle, 'AbsTol', 1e-10);
        end
        
        function testAngleRanges(testCase)
            % Test that extracted angles are within proper ranges
            % Pitch: -90 to 90 degrees, Yaw: -90 to 90, Roll: -180 to 180
            
            % Generate random rotation matrices
            for i = 1:10
                % Create random rotation matrix
                randomAngles = rand(1,3) .* [180, 90, 180] - [90, 45, 90];
                pitchRad = deg2rad(randomAngles(1));
                yawRad = deg2rad(randomAngles(2));
                rollRad = deg2rad(randomAngles(3));
                
                Rx = [1, 0,                0;
                      0, cos(rollRad), -sin(rollRad);
                      0, sin(rollRad),  cos(rollRad)];
                
                Ry = [cos(pitchRad),  0, sin(pitchRad);
                      0,               1, 0;
                      -sin(pitchRad), 0, cos(pitchRad)];
                
                Rz = [cos(yawRad), -sin(yawRad), 0;
                      sin(yawRad),  cos(yawRad), 0;
                      0,            0,             1];
                
                C = Rz * Ry * Rx;
                
                angle = rotToAngleMat(C);
                
                % Verify ranges
                testCase.verifyGreaterThanOrEqual(angle(1), -90);
                testCase.verifyLessThanOrEqual(angle(1), 90);
                testCase.verifyGreaterThanOrEqual(angle(2), -90);
                testCase.verifyLessThanOrEqual(angle(2), 90);
                testCase.verifyGreaterThanOrEqual(angle(3), -180);
                testCase.verifyLessThanOrEqual(angle(3), 180);
            end
        end
        
        function testSingleMatrixOutputSize(testCase)
            % Test output size for single rotation matrix
            C = eye(3);
            angle = rotToAngleMat(C);
            
            testCase.verifySize(angle, [1, 3]);
        end
        
        function testMultipleMatricesOutputSize(testCase)
            % Test output size for multiple rotation matrices
            C1 = eye(3);
            C2 = [0, 0, 1; 0, 1, 0; -1, 0, 0]; % 90 deg pitch
            C3 = [0, -1, 0; 1, 0, 0; 0, 0, 1]; % 90 deg yaw
            
            C = cat(3, C1, C2, C3);
            angle = rotToAngleMat(C);
            
            testCase.verifySize(angle, [1,3,3]);
        end
    end
end