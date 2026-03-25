classdef rotToQuatTest < matlab.unittest.TestCase

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

            % rotation matrix
            C = [-0.2593,-0.0741,0.9630;
                  0.5185 ,-0.8519,0.0741;
                  0.8148 ,0.5185 ,0.2593];
            % get quaternion
            q = rotToQuat(C);
            disp(q);

            % declare expected quaternion
            qExp = [ 0.5773 ;
                     0.1924 ;
                     0.7698 ;
                    -0.1924];

            % verify that the function returned the expected values
            testCase.verifyEqual(q, qExp, 'AbsTol', 1e-4);
        end

        function testIdentityRotation(testCase)
            % Test identity rotation matrix gives identity quaternion
            C = eye(3);
            q = rotToQuat(C);
            
            expected = [0; 0; 0; 1];
            testCase.verifyEqual(q, expected, 'AbsTol', 1e-10);
        end
        
        function testPureXRotation(testCase)
            % Test 90 degree rotation about X-axis
            angle = pi/2;
            C = [1,  0,  0;
                 0,  0,  1;
                 0, -1,  0];
            
            q = rotToQuat(C);
            
            expected = [sin(angle/2); 0; 0; cos(angle/2)];
            testCase.verifyEqual(q, expected, 'AbsTol', 1e-10);
        end
        
        function testPureYRotation(testCase)
            % Test 90 degree rotation about Y-axis
            angle = pi/2;
            C = [ 0, 0, -1;
                  0, 1,  0;
                  1, 0,  0];
            
            q = rotToQuat(C);
            
            expected = [0; sin(angle/2); 0; cos(angle/2)];
            testCase.verifyEqual(q, expected, 'AbsTol', 1e-10);
        end
        
        function testPureZRotation(testCase)
            % Test 90 degree rotation about Z-axis
            angle = pi/2;
            C = [0, 1, 0;
                -1, 0, 0;
                 0, 0, 1];
            
            q = rotToQuat(C);
            
            expected = [0; 0; sin(angle/2); cos(angle/2)];
            testCase.verifyEqual(q, expected, 'AbsTol', 1e-10);
        end
        
        function test180DegreeRotation(testCase)
            % Test 180 degree rotation about X-axis
            % angle = pi;
            C = [1,  0,  0;
                 0, -1,  0;
                 0,  0, -1];
            
            q = rotToQuat(C);
            
            % expected = [sin(angle/2); 0; 0; cos(angle/2)];
            % For 180°, both q and -q are valid, so check magnitude
            testCase.verifyEqual(norm(q), 1, 'AbsTol', 1e-10);
            testCase.verifyEqual(abs(q(1)), 1, 'AbsTol', 1e-10);
            testCase.verifyEqual(q(4), 0, 'AbsTol', 1e-10);
        end
        
        function testSmallAngleRotation(testCase)
            % Test small angle rotation
            angle = deg2rad(5); % 5 degrees
            axis = [1; 2; 3]; axis = axis / norm(axis);
            
            % Rodrigues' rotation formula
            K = [0, -axis(3), axis(2);
                 axis(3), 0, -axis(1);
                 -axis(2), axis(1), 0];
            C = eye(3) + sin(angle) * K + (1 - cos(angle)) * K * K;
            
            q = rotToQuat(C);
            
            % Should be a valid quaternion
            testCase.verifyEqual(norm(q), 1, 'AbsTol', 1e-10);
        end
        
        function testQuaternionProperties(testCase)
            % Test that output quaternion has unit norm
            angles = [0, pi/6, pi/4, pi/3, pi/2];
            for angle = angles
                % Random axis rotation
                axis = rand(3,1); axis = axis / norm(axis);
                K = [0, -axis(3), axis(2);
                     axis(3), 0, -axis(1);
                     -axis(2), axis(1), 0];
                C = eye(3) + sin(angle) * K + (1 - cos(angle)) * K * K;
                
                q = rotToQuat(C);
                
                % Test unit norm
                testCase.verifyEqual(norm(q), 1, 'AbsTol', 1e-10);
            end
        end
        
        function testNumericalStability(testCase)
            % Test numerical stability with various rotations
            testAngles = [1e-6, 1e-3, 1, 45, 89, 90, 135, 179, 180];
            
            for angleDeg = testAngles
                angleRad = deg2rad(angleDeg);
                axis = [1; 1; 1]; axis = axis / norm(axis); % Arbitrary axis
                
                K = [0, -axis(3), axis(2);
                     axis(3), 0, -axis(1);
                     -axis(2), axis(1), 0];
                C = eye(3) + sin(angleRad) * K + (1 - cos(angleRad)) * K * K;
                
                q = rotToQuat(C);
                
                % Should always produce unit quaternion
                testCase.verifyEqual(norm(q), 1, 'AbsTol', 1e-10, ...
                    sprintf('Failed for angle %.1f degrees', angleDeg));
            end
        end
        
        function testTraceCalculation(testCase)
            % Test specific trace-based calculations
            C = [0, 0, 1;
                 0, 1, 0;
                 -1, 0, 0];
            
            q = rotToQuat(C);
            
            % Verify the internal trace calculation
            T = trace(C);
            testCase.verifyEqual(T, 1, 'AbsTol', 1e-10);
            
            % q4^2 should be (1 + T)/4 = (1 + 1)/4 = 0.5
            testCase.verifyEqual(q(4)^2, 0.5, 'AbsTol', 1e-10);
        end
    end
end