classdef cpSinTest < matlab.unittest.TestCase
    % Test class for the drag function.

    % Private property to hold the path that is added temporarily
    properties (Access = private)
        AddedPath;
        cpSinArray = readmatrix("data_Cp_conical_nose_sinTest.csv");
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
    
    methods (TestClassTeardown)
        function removeFunctionPath(testCase)
            % This removes the path added in TestClassSetup
            rmpath(testCase.AddedPath);
        end
    end

    methods (Test)
        
        function testBasic(testCase)
            % declare plausible function parameters
            semiVertexAngleIndex = 125;  % must be between 0 and 225, inclusive
            semiVertexAngle = semiVertexAngleIndex*0.2 *pi/180;
            machNumberIndex = 250;  % must be between 1 and 450, inclusive
            machNumber = machNumberIndex*0.02 + 1;
            % get ouput
            value = cpSin(semiVertexAngle,machNumber,testCase.cpSinArray);

            % declare expected output
            valueExpected = testCase.cpSinArray(machNumberIndex,semiVertexAngleIndex+1);

            %verify
            testCase.verifyEqual(value,valueExpected,'AbsTol',1e-15);
        end

        function testIndexOutOfRange(testCase)
            % declare parameters with out of range index
            semiVertexAngleIndex = 300;  % should be between -225 and 225, inclusive
            semiVertexAngle = semiVertexAngleIndex*0.2 *pi/180;
            machNumberIndex = 250;  % should be between 1 and 450, inclusive
            machNumber = machNumberIndex*0.02 + 1;

            %verify that we get an error
            testCase.verifyError(@() cpSin(semiVertexAngle,machNumber,testCase.cpSinArray),"MATLAB:badsubscript");
            
            % declare parameters with out of range index
            semiVertexAngleIndex = 125;  % should be between -225 and 225, inclusive
            semiVertexAngle = semiVertexAngleIndex*0.2 *pi/180;
            machNumberIndex = 500;  % should be between 1 and 450, inclusive
            machNumber = machNumberIndex*0.02 + 1;

            %verify that we get an error
            testCase.verifyError(@() cpSin(semiVertexAngle,machNumber,testCase.cpSinArray),"MATLAB:badsubscript");
        end
    end
end