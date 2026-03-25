classdef rotMatTest < matlab.unittest.TestCase

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
            % Test that the function generates the correct
            % matrix for a random angle and axis

            % angle
            a = pi/5;
            % axis
            ax = 2; % y-axis
            % get rotation matrix
            C = rotMat(a,ax);

            % declare expected rotation matrix
            CExp = [cos(pi/5), 0, sin(pi/5) ;
                    0        , 1, 0         ;
                   -sin(pi/5), 0, cos(pi/5)];
            
            % verify that the function returned the expected result
            testCase.verifyEqual(C, CExp, 'AbsTol', 1e-10);
        end

        function testZeroAngleAllAxes(testCase)
            % Test zero angle rotation for all axes
            angles = [0, 0, 0];
            for ax = 1:3
                C = rotMat(angles(ax), ax);
                expected = eye(3);
                testCase.verifyEqual(C, expected, 'AbsTol', 1e-10);
            end
        end
        
        function testXAxisRotation(testCase)
            % Test rotation about X-axis (axis 1)
            angle = pi/4; % 45 degrees
            C = rotMat(angle, 1);
            
            expected = [1, 0, 0;
                        0, cos(angle), -sin(angle);
                        0, sin(angle), cos(angle)];
            
            testCase.verifyEqual(C, expected, 'AbsTol', 1e-10);
        end
        
        function testYAxisRotation(testCase)
            % Test rotation about Y-axis (axis 2)
            angle = pi/3; % 60 degrees
            C = rotMat(angle, 2);
            
            expected = [cos(angle), 0, sin(angle);
                        0, 1, 0;
                        -sin(angle), 0, cos(angle)];
            
            testCase.verifyEqual(C, expected, 'AbsTol', 1e-10);
        end
        
        function testZAxisRotation(testCase)
            % Test rotation about Z-axis (axis 3)
            angle = pi/6; % 30 degrees
            C = rotMat(angle, 3);
            
            expected = [cos(angle), -sin(angle), 0;
                        sin(angle), cos(angle), 0;
                        0, 0, 1];
            
            testCase.verifyEqual(C, expected, 'AbsTol', 1e-10);
        end
        
        function testNegativeAngles(testCase)
            % Test negative angle rotations
            angle = -pi/4; % -45 degrees
            for ax = 1:3
                C = rotMat(angle, ax);
                % Should be transpose of positive angle rotation
                CPositive = rotMat(-angle, ax);
                testCase.verifyEqual(C, CPositive', 'AbsTol', 1e-10);
            end
        end
        
        function test90DegreeRotations(testCase)
            % Test 90 degree rotations for all axes
            angle = pi/2;
            expectedResults = {
                [1, 0, 0; 0, 0, -1; 0, 1, 0], ... X-axis
                [0, 0, 1; 0, 1, 0; -1, 0, 0], ... Y-axis  
                [0, -1, 0; 1, 0, 0; 0, 0, 1]  ... Z-axis
            };
            
            for ax = 1:3
                C = rotMat(angle, ax);
                testCase.verifyEqual(C, expectedResults{ax}, 'AbsTol', 1e-10);
            end
        end
        
        function test180DegreeRotations(testCase)
            % Test 180 degree rotations for all axes
            angle = pi;
            expectedResults = {
                [1, 0, 0; 0, -1, 0; 0, 0, -1], ... X-axis
                [-1, 0, 0; 0, 1, 0; 0, 0, -1], ... Y-axis
                [-1, 0, 0; 0, -1, 0; 0, 0, 1]  ... Z-axis
            };
            
            for ax = 1:3
                C = rotMat(angle, ax);
                testCase.verifyEqual(C, expectedResults{ax}, 'AbsTol', 1e-10);
            end
        end
        
        function testRotationMatrixProperties(testCase)
            % Test that rotation matrices are orthogonal with determinant 1
            angles = [0.1, 0.5, 1.0, 2.0];
            for ax = 1:3
                for angle = angles
                    C = rotMat(angle, ax);
                    
                    % Test orthogonality: C * C' = I
                    identityTest = C * C';
                    testCase.verifyEqual(identityTest, eye(3), 'AbsTol', 1e-10);
                    
                    % Test determinant = 1
                    detC = det(C);
                    testCase.verifyEqual(detC, 1, 'AbsTol', 1e-10);
                    
                    % Test inverse equals transpose
                    testCase.verifyEqual(inv(C), C', 'AbsTol', 1e-10);
                end
            end
        end
        
        function testSmallAngles(testCase)
            % Test small angle approximations
            smallAngle = 1e-6;
            for ax = 1:3
                C = rotMat(smallAngle, ax);
                
                % For small angles, C ≈ I + skew-symmetric matrix
                expectedApprox = eye(3);
                switch ax
                    case 1
                        expectedApprox(2,3) = -smallAngle;
                        expectedApprox(3,2) = smallAngle;
                    case 2
                        expectedApprox(1,3) = smallAngle;
                        expectedApprox(3,1) = -smallAngle;
                    case 3
                        expectedApprox(1,2) = -smallAngle;
                        expectedApprox(2,1) = smallAngle;
                end
                
                testCase.verifyEqual(C, expectedApprox, 'AbsTol', 1e-10);
            end
        end
        
        function testMultipleAngleValues(testCase)
            % Test multiple angle values for consistency
            testAngles = deg2rad([0, 15, 30, 45, 60, 75, 90, 120, 135, 150, 180]);
            
            for ax = 1:3
                for angle = testAngles
                    C = rotMat(angle, ax);
                    
                    % Verify basic properties
                    testCase.verifyEqual(det(C), 1, 'AbsTol', 1e-10);
                    testCase.verifyEqual(C * C', eye(3), 'AbsTol', 1e-10);
                    
                    % Verify specific elements based on axis
                    switch ax
                        case 1
                            testCase.verifyEqual(C(1,1), 1, 'AbsTol', 1e-10);
                            testCase.verifyEqual(C(2:3,2:3), [cos(angle), -sin(angle); sin(angle), cos(angle)], ...
                                'AbsTol', 1e-10);
                        case 2
                            testCase.verifyEqual(C(2,2), 1, 'AbsTol', 1e-10);
                            testCase.verifyEqual(C([1,3],[1,3]), [cos(angle), sin(angle); -sin(angle), cos(angle)], ...
                                'AbsTol', 1e-10);
                        case 3
                            testCase.verifyEqual(C(3,3), 1, 'AbsTol', 1e-10);
                            testCase.verifyEqual(C(1:2,1:2), [cos(angle), -sin(angle); sin(angle), cos(angle)], ...
                                'AbsTol', 1e-10);
                    end
                end
            end
        end
        
        function testInvalidAxisError(testCase)
            % Test that invalid axis numbers throw errors
            invalidAxes = [0, 4, -1, 10];
            for ax = invalidAxes
                testCase.verifyError(@() rotMat(0, ax), 'rotMat:axisNumber');
            end
        end
    end
end