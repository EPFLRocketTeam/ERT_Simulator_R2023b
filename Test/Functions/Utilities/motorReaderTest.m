classdef motorReaderTest < matlab.unittest.TestCase

    properties
        AddedPath;
    end
    
    properties (TestParameter)
        validMotorFile = {'motorTestFile.txt','motorTestFile2.txt'};
        invalidMotorFile = {'nonexistentFile.txt', '', ' '};
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
    
    methods (Test)
        function testBasicFunctionality(testCase)
            % Test that the function returns expected output structure
            motorFile = 'motorTestFile.txt';
            
            [t, T, Info] = motorReader(motorFile);
            
            % Verify outputs exist
            testCase.verifyNotEmpty(t, 'Time vector should not be empty');
            testCase.verifyNotEmpty(T, 'Thrust vector should not be empty');
            testCase.verifyNotEmpty(Info, 'Info structure should not be empty');
            
            % Verify dimensions match
            testCase.verifyEqual(length(t), length(T), ...
                'Time and thrust vectors should have same length');
        end
        
        function testOutputTypes(testCase)
            % Test that outputs have correct data types
            motorFile = 'motorTestFile.txt';
            
            [t, T, Info] = motorReader(motorFile);
            
            % Check data types
            testCase.verifyClass(t, 'double', 'Time should be double');
            testCase.verifyClass(T, 'double', 'Thrust should be double');
            testCase.verifyClass(Info, 'cell', 'Info should be a cell array');
        end
        
        function testFirstLineInfo(testCase)
            % Test that the first line info is correctly parsed
            motorFile = 'motorTestFile.txt';
            
            [~, ~, Info] = motorReader(motorFile);
            
            % Verify Info contains expected number of elements
            testCase.verifyNumElements(Info, 7, 'Info should contain 7 elements');
            
            % Verify specific values from the first line
            expectedFirstElement = 'Demo2Motor';
            testCase.verifyEqual(Info{1}{1}, expectedFirstElement, ...
                'First element should be the motor name');
        end
        
        function testTimeVectorProperties(testCase)
            % Test properties of the time vector
            motorFile = 'motorTestFile.txt';
            
            [t, ~, ~] = motorReader(motorFile);
            
            % Check time monotonicity
            testCase.verifyTrue(all(diff(t) >= 0), ...
                'Time should be monotonically increasing');
            
            % Check time starts at 0
            testCase.verifyEqual(t(1), 0, 'Time should start at 0');
            
            % Check time spacing (should be uniform at 0.001)
            expectedDelta = 0.001;
            actualDelta = mean(diff(t));
            testCase.verifyEqual(actualDelta, expectedDelta, 'RelTol', 0.01, ...
                'Time steps should be approximately 0.001 seconds');
        end
        
        function testThrustVectorProperties(testCase)
            % Test properties of the thrust vector
            motorFile = 'motorTestFile.txt';
            
            [~, T, ~] = motorReader(motorFile);
            
            % Check thrust is non-negative (physical motor can't have negative thrust)
            testCase.verifyTrue(all(T >= 0), 'Thrust values should be non-negative');
            
            % Check thrust is finite
            testCase.verifyTrue(all(isfinite(T)), 'Thrust values should be finite');
        end
        
        function testFirstAndLastDataPoints(testCase)
            % Test specific data points from the file
            motorFile = 'motorTestFile.txt';
            
            [t, T, ~] = motorReader(motorFile);
            
            % Test first data point
            testCase.verifyEqual(t(1), 0, 'First time should be 0');
            testCase.verifyEqual(T(1), 4.3974, 'RelTol', 1e-4, ...
                'First thrust value mismatch');
            
            % Test a middle data point (e.g., at t=5.0)
            idx_5s = find(abs(t - 5.0) < 1e-6);
            if ~isempty(idx_5s)
                testCase.verifyEqual(T(idx_5s), 1529.6289, 'RelTol', 1e-4, ...
                    'Thrust at t=5.0 mismatch');
            end
            
            % Test last data point
            testCase.verifyEqual(t(end), 10.223, 'RelTol', 1e-4, ...
                'Last time mismatch');
            testCase.verifyEqual(T(end), 6.7615, 'RelTol', 1e-4, ...
                'Last thrust value mismatch');
        end
        
        function testFileNotFound(testCase)
            % Test behavior with non-existent file
            nonExistentFile = 'nonExistentFile.txt';
            
            % This should throw an error
            testCase.verifyError(@() motorReader(nonExistentFile), ...
                'MATLAB:FileIO:InvalidFid', ...
                'Should throw error for non-existent file');
        end
        
        function testDataConsistency(testCase)
            % Test that all data points are read correctly
            motorFile = 'motorTestFile.txt';
            
            % Read the file manually to count lines
            fid = fopen(motorFile, 'r');
            fgetl(fid); % Skip header
            lineCount = 0;
            while ~feof(fid)
                fgetl(fid);
                lineCount = lineCount + 1;
            end
            fclose(fid);
            
            % Read with motorReader
            [t, ~, ~] = motorReader(motorFile);
            
            % Verify number of data points
            testCase.verifyEqual(length(t), lineCount, ...
                'Number of data points mismatch');
        end
        
        function testInvalidInputs(testCase)
            % Test with invalid inputs
            
            % Test with empty string
            testCase.verifyError(@() motorReader(''), ...
                'MATLAB:FileIO:InvalidFid', ...
                'Should error with empty filename');
            
            % Test with non-string input
            testCase.verifyError(@() motorReader(123), ...
                'MATLAB:FileIO:InvalidFid', ...
                'Should error with numeric input');
        end
        
        function testNumericPrecision(testCase)
            % Test that numeric precision is maintained
            motorFile = 'motorTestFile.txt';
            
            [t, T, ~] = motorReader(motorFile);
            
            % Check that we have double precision
            testCase.verifyClass(t, 'double');
            testCase.verifyClass(T, 'double');
            
            % Check some specific values with high precision
            % Find index near a specific time
            idx = find(abs(t - 0.05) < 1e-6);
            if ~isempty(idx)
                testCase.verifyEqual(T(idx), 21.8012, 'AbsTol', 1e-4, ...
                    'High precision thrust value mismatch');
            end
        end
        
        function testDataRange(testCase)
            % Test that data is within expected ranges
            motorFile = 'motorTestFile.txt';
            
            [t, T, ~] = motorReader(motorFile);
            
            % Time should be within file bounds
            testCase.verifyTrue(t(1) >= 0, 'Time should start >= 0');
            testCase.verifyTrue(t(end) <= 11, 'Time should be within reasonable range');
            
            % Thrust should be within reasonable range for this motor
            testCase.verifyTrue(max(T) < 2000, 'Max thrust too high');
            testCase.verifyTrue(min(T) >= 0, 'Min thrust should be >= 0');
        end
    end
    
    methods (Test, ParameterCombination = 'sequential')
        function testMultipleFiles(testCase, validMotorFile)
            % Test with different motor files (parameterized)
            [t, T, Info] = motorReader(validMotorFile);
            
            testCase.verifyNotEmpty(t);
            testCase.verifyNotEmpty(T);
            testCase.verifyNotEmpty(Info);
        end
    end
end