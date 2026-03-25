classdef quatEvolveTest < matlab.unittest.TestCase

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
            % for a random quaternion and angular velocity
            q = [1; 2; 0.3; 1.6];
            w = [1; 3; 5];
            qDot = quatEvolve(q, w);
            qDotExpected = [3.584136662812134;-3.481726674375732;3.970240998843640;-7.075381339500586];
            testCase.verifyEqual(qDot, qDotExpected, 'AbsTol', 1e-10);
        end

        function testZeroAngularVelocity(testCase)
            % Test that zero angular velocity produces near-zero derivative
            q = [1; 0; 0; 0];
            w = [0; 0; 0];
            qDot = quatEvolve(q, w);
            testCase.verifyEqual(qDot, zeros(4,1), 'AbsTol', 1e-10);
        end
        
        function testXAxisRotation(testCase)
            % Test rotation about X-axis from identity quaternion
            q = [1; 0; 0; 0];
            w = [1; 0; 0];
            qDot = quatEvolve(q, w);
            expected = [0; 0; 0; -0.5];
            testCase.verifyEqual(qDot, expected, 'AbsTol', 1e-10);
        end

        function testYAxisRotation(testCase)
            % Test rotation about X-axis from identity quaternion
            q = [1; 0; 0; 0];
            w = [0; 1; 0];
            qDot = quatEvolve(q, w);
            expected = [0; 0; 0.5; 0];
            testCase.verifyEqual(qDot, expected, 'AbsTol', 1e-10);
        end

        function testZAxisRotation(testCase)
            % Test rotation about X-axis from identity quaternion
            q = [1; 0; 0; 0];
            w = [0; 0; 1];
            qDot = quatEvolve(q, w);
            expected = [0; -0.5; 0; 0];
            testCase.verifyEqual(qDot, expected, 'AbsTol', 1e-10);
        end
        
        function testQuaternionNormPreservation(testCase)
            % Test that the derivative maintains quaternion structure
            q = [0.5; 0.5; 0.5; 0.5];
            q = q / norm(q);
            w = [0.1; -0.2; 0.3];
            qDot = quatEvolve(q, w);
            
            % For small rotations, the derivative should be approximately
            % orthogonal to the quaternion (q·qDot ≈ 0)
            dotProduct = dot(q, qDot);
            testCase.verifyLessThan(abs(dotProduct), 0.1);
        end
    end
end