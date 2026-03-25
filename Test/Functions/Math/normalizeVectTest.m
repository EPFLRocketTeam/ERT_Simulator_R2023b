classdef normalizeVectTest < matlab.unittest.TestCase

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
        
        function testBasicNormalization(testCase)
            % Test a simple 2D vector
            v = [3, 4];
            expectedUnitV = [0.6, 0.8];
            unitV = normalizeVect(v);
            
            % Verify the result using a tolerance for floating-point comparison
            testCase.verifyThat(unitV, ...
                matlab.unittest.constraints.IsEqualTo(expectedUnitV, ...
                'Within', matlab.unittest.constraints.AbsoluteTolerance(1e-10)));
        end
        
        function testZeroVector(testCase)
            % Test the zero vector case
            v = [0, 0, 0];
            expectedUnitV = [0, 0, 0];
            unitV = normalizeVect(v);
            
            % Verify the result (this should be exact)
            testCase.verifyEqual(unitV, expectedUnitV);
        end
        
        function testAlreadyUnitVector(testCase)
            % Test a vector that is already a unit vector
            v = [0, 1, 0];
            expectedUnitV = [0, 1, 0];
            unitV = normalizeVect(v);
            
            % Verify the result is unchanged
            testCase.verifyThat(unitV, ...
                matlab.unittest.constraints.IsEqualTo(expectedUnitV, ...
                'Within', matlab.unittest.constraints.AbsoluteTolerance(1e-10)));
        end
        
        function testNegativeComponents(testCase)
            % Test a vector with negative components
            v = [-5, -12]; % norm is 13
            expectedUnitV = [-5/13, -12/13];
            unitV = normalizeVect(v);
            
            % Verify the result
            testCase.verifyThat(unitV, ...
                matlab.unittest.constraints.IsEqualTo(expectedUnitV, ...
                'Within', matlab.unittest.constraints.AbsoluteTolerance(1e-10)));
        end
        
        function test3DVector(testCase)
            % Test a 3D vector
            v = [1, 2, 2]; % norm is 3
            expectedUnitV = [1/3, 2/3, 2/3];
            unitV = normalizeVect(v);
            
            % Verify the result
            testCase.verifyThat(unitV, ...
                matlab.unittest.constraints.IsEqualTo(expectedUnitV, ...
                'Within', matlab.unittest.constraints.AbsoluteTolerance(1e-10)));
        end
        
        function testColumnVector(testCase)
            % Test a column vector to ensure orientation is preserved
            v = [3; 4];
            expectedUnitV = [0.6; 0.8];
            unitV = normalizeVect(v);
            
            % Verify the result
            testCase.verifyThat(unitV, ...
                matlab.unittest.constraints.IsEqualTo(expectedUnitV, ...
                'Within', matlab.unittest.constraints.AbsoluteTolerance(1e-10)));
            
            % Verify the size/orientation is the same as the input
            testCase.verifySize(unitV, size(v));
        end
        
    end
end