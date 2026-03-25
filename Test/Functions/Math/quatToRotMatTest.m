classdef quatToRotMatTest < matlab.unittest.TestCase

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
            % for random quaternions

            % first quaternion
            q1 = [3;1;4;1];
            q1 = q1/norm(q1); % normalize to 1
            % second quaternion
            q2 = [5;9;2;6];
            q2 = q2/norm(q2); % normalize to 1
            % combine
            q = [q1,q2];
            % get rotation matrices
            C = quatToRotMat(q);

            % declare expected rotation matrices
            C1Exp = [-0.2593,-0.0741,0.9630;
                     0.5185 ,-0.8519,0.0741;
                     0.8148 ,0.5185 ,0.2593];
            C2Exp = [-0.1644,0.4521,0.8767 ;
                     0.7808 ,0.6027,-0.1644;
                     -0.6027,0.6575,-0.4521];
            CExp = cat(3,C1Exp,C2Exp);

            % verify that the function returned the expected values
            testCase.verifyEqual(C, CExp, 'AbsTol', 1e-4);
        end

        function testIdentityQuaternion(testCase)
            % Test identity quaternion gives identity matrix
            q = [0; 0; 0; 1]; % Identity quaternion
            C = quatToRotMat(q);
            
            expected = eye(3);
            testCase.verifyEqual(C, expected, 'AbsTol', 1e-10);
        end
        
        function testPureXRotation(testCase)
            % Test 90 degree rotation about X-axis
            angle = pi/2;
            q = [sin(angle/2); 0; 0; cos(angle/2)];
            C = quatToRotMat(q);
            
            expected = [1,  0,  0;
                        0,  0, -1;
                        0,  1,  0];
            testCase.verifyEqual(C, expected, 'AbsTol', 1e-10);
        end
        
        function testPureYRotation(testCase)
            % Test 90 degree rotation about Y-axis
            angle = pi/2;
            q = [0; sin(angle/2); 0; cos(angle/2)];
            C = quatToRotMat(q);
            
            expected = [ 0, 0,  1;
                         0, 1,  0;
                        -1, 0,  0];
            testCase.verifyEqual(C, expected, 'AbsTol', 1e-10);
        end
        
        function testPureZRotation(testCase)
            % Test 90 degree rotation about Z-axis
            angle = pi/2;
            q = [0; 0; sin(angle/2); cos(angle/2)];
            C = quatToRotMat(q);
            
            expected = [0,-1, 0;
                        1, 0, 0;
                        0, 0, 1];
            testCase.verifyEqual(C, expected, 'AbsTol', 1e-10);
        end
        
        function testMultipleQuaternions(testCase)
            % Test multiple quaternions input (4xn)
            q1 = [0; 0; 0; 1]; % Identity
            q2 = [sin(pi/4); 0; 0; cos(pi/4)]; % 90° X rotation
            q = [q1, q2]; % 4x2 matrix
            
            C = quatToRotMat(q);
            
            % Verify dimensions
            testCase.verifySize(C, [3, 3, 2]);
            
            % Verify first rotation (identity)
            testCase.verifyEqual(C(:,:,1), eye(3), 'AbsTol', 1e-10);
            
            % Verify second rotation (90° X)
            expected = [1,  0,  0;
                        0,  0, -1;
                        0,  1,  0];
            testCase.verifyEqual(C(:,:,2), expected, 'AbsTol', 1e-10);
        end
        
        function testRotationMatrixProperties(testCase)
            % Test that rotation matrix is orthogonal and determinant = 1
            angles = [0, pi/6, pi/4, pi/3, pi/2];
            for angle = angles
                % Random axis rotation
                axis = rand(3,1); axis = axis/norm(axis);
                q = [axis * sin(angle/2); cos(angle/2)];
                
                C = quatToRotMat(q);
                
                % Test orthogonality: C * C' = I
                identityTest = C * C';
                testCase.verifyEqual(identityTest, eye(3), 'AbsTol', 1e-10);
                
                % Test determinant = 1
                detC = det(C);
                testCase.verifyEqual(detC, 1, 'AbsTol', 1e-10);
            end
        end
        
        function testInverseRotation(testCase)
            % Test that quaternion conjugate gives inverse rotation
            q = [0.5; 0.5; 0.5; 0.5]; % Random quaternion
            q = q / norm(q); % Normalize
            
            C = quatToRotMat(q);
            qInv = [-q(1:3); q(4)]; % Inverse quaternion
            CInv = quatToRotMat(qInv);
            
            % CInv should equal C'
            testCase.verifyEqual(CInv, C', 'AbsTol', 1e-10);
            
            % C * CInv should equal identity
            product = C * CInv;
            testCase.verifyEqual(product, eye(3), 'AbsTol', 1e-10);
        end
        
        function testSmallAngleApproximation(testCase)
            % Test small angles approximate identity + skew-symmetric matrix
            smallAngle = 1e-3;
            wx = smallAngle; wy = 0; wz = 0;
            
            % Small angle quaternion approximation
            q = [wx/2; wy/2; wz/2; 1];
            q = q / norm(q);
            
            C = quatToRotMat(q);
            
            % For small angles, C ≈ I + skew([wx, wy, wz])
            expectedApprox = eye(3) + [0, -wz, wy; wz, 0, -wx; -wy, wx, 0];
            
            testCase.verifyEqual(C, expectedApprox, 'AbsTol', 1e-6);
        end
        
        function testCompositionOfRotations(testCase)
            % Test that quaternion multiplication corresponds to matrix multiplication
            % q1 * q2 should correspond to C1 * C2
            
            % First rotation: 30° about X
            angle1 = pi/6;
            q1 = [sin(angle1/2); 0; 0; cos(angle1/2)];
            C1 = quatToRotMat(q1);
            
            % Second rotation: 45° about Y
            angle2 = pi/4;
            q2 = [0; sin(angle2/2); 0; cos(angle2/2)];
            C2 = quatToRotMat(q2);
            
            % Combined rotation: q = q1 * q2
            q1Vec = q1; q2Vec = q2;
            qCombined = [q1Vec(4)*q2Vec(1:3) + q2Vec(4)*q1Vec(1:3) + cross(q1Vec(1:3), q2Vec(1:3));
                         q1Vec(4)*q2Vec(4) - dot(q1Vec(1:3), q2Vec(1:3))];
            qCombined = qCombined / norm(qCombined);
            
            CCombined = quatToRotMat(qCombined);
            CExpected = C1 * C2;
            
            testCase.verifyEqual(CCombined, CExpected, 'AbsTol', 1e-10);
        end
        
        function testInputDimensions(testCase)
            % Test various input dimensions
            % Single quaternion
            qSingle = [0.5; 0.5; 0.5; 0.5];
            CSingle = quatToRotMat(qSingle);
            testCase.verifySize(CSingle, [3, 3, 1]);
            
            % Multiple quaternions (4x2)
            qMulti = [qSingle, [0; 0; 0; 1]];
            CMulti = quatToRotMat(qMulti);
            testCase.verifySize(CMulti, [3, 3, 2]);
            
            % Multiple quaternions (4x5)
            qMulti5 = repmat([0; 0; 0; 1], 1, 5);
            CMulti5 = quatToRotMat(qMulti5);
            testCase.verifySize(CMulti5, [3, 3, 5]);
        end
    end
end