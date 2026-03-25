classdef quatToEulerAnglesTest < matlab.unittest.TestCase

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
            % get Euler angles
            [phi,theta,psi] = quatToEulerAngles(q(1,:),q(2,:),q(3,:),q(4,:));

            % declare expected Euler angles
            phiExp = [1.107148717794090,2.173083672929860];
            thetaExp = [-0.952409684908894,0.646930182498288];
            psiExp = [2.034443935795703,1.778292553230099];

            % verify that the function returned the expected values
            testCase.verifyEqual([phi,theta,psi], [phiExp,thetaExp,psiExp], 'AbsTol', 1e-10);
        end

        function testIdentityQuaternion(testCase)
            % Test that the identity quaternion gives zero angles
            [phi, theta, psi] = quatToEulerAngles(0, 0, 0, 1);
            testCase.verifyEqual(phi, 0, 'AbsTol', 1e-10);
            testCase.verifyEqual(theta, 0, 'AbsTol', 1e-10);
            testCase.verifyEqual(psi, 0, 'AbsTol', 1e-10);
        end
        
        function testPureRoll(testCase)
            % Test pure roll rotation
            angle = pi/4; % 45 degrees
            qw = cos(angle/2); qx = sin(angle/2); qy = 0; qz = 0;
            
            [phi, theta, psi] = quatToEulerAngles(qx, qy, qz, qw);
            
            testCase.verifyEqual(phi, angle, 'AbsTol', 1e-10);
            testCase.verifyEqual(theta, 0, 'AbsTol', 1e-10);
            testCase.verifyEqual(psi, 0, 'AbsTol', 1e-10);
        end
        
        function testPurePitch(testCase)
            % Test pure pitch rotation
            angle = pi/6; % 30 degrees
            qw = cos(angle/2); qx = 0; qy = sin(angle/2); qz = 0;
            
            [phi, theta, psi] = quatToEulerAngles(qx, qy, qz, qw);
            
            testCase.verifyEqual(phi, 0, 'AbsTol', 1e-10);
            testCase.verifyEqual(theta, angle, 'AbsTol', 1e-10);
            testCase.verifyEqual(psi, 0, 'AbsTol', 1e-10);
        end
        
        function testPureYaw(testCase)
            % Test pure yaw rotation
            angle = pi/3; % 60 degrees
            qw = cos(angle/2); qx = 0; qy = 0; qz = sin(angle/2);
            
            [phi, theta, psi] = quatToEulerAngles(qx, qy, qz, qw);
            
            testCase.verifyEqual(phi, 0, 'AbsTol', 1e-10);
            testCase.verifyEqual(theta, 0, 'AbsTol', 1e-10);
            testCase.verifyEqual(psi, angle, 'AbsTol', 1e-10);
        end
        
        function testAngleRanges(testCase)
            % Test that angles are in proper ranges
            % Roll and Yaw: -pi to pi, Pitch: -pi/2 to pi/2
            
            testAngles = [-pi, -pi/2, 0, pi/2, pi];
            for angle = testAngles
                qw = cos(angle/2); qx = sin(angle/2); qy = 0; qz = 0;
                [phi, theta, psi] = quatToEulerAngles(qx, qy, qz, qw);
                
                % Check ranges
                testCase.verifyGreaterThanOrEqual(phi, -pi);
                testCase.verifyLessThanOrEqual(phi, pi);
                testCase.verifyGreaterThanOrEqual(theta, -pi/2);
                testCase.verifyLessThanOrEqual(theta, pi/2);
                testCase.verifyGreaterThanOrEqual(psi, -pi);
                testCase.verifyLessThanOrEqual(psi, pi);
            end
        end
    end
end