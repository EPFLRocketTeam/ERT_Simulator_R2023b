classdef simOutputReaderTest < matlab.unittest.TestCase
    properties
        AddedPath
    end
    methods(Test)
        function addFunctionPath(testCase)
            % This function temporarily adds the directory of the
            % inertialMatrix function to the MATLAB path so the tests can find it

            % 1. Get the path of the current test file directory
            % (e.g., ...\ERT_Simulator_R2023b\Test\Functions\Math)
            testDir = fileparts(mfilename('fullpath'));

            % 2. Move up two directories to reach the root folder
            % (e.g., ...\ERT_Simulator_R2023b)
            rootPath = fileparts(fileparts(fileparts(testDir)));

            % 3. Construct the path to the function file's directory
            % (e.g., ...\ERT_Simulator_R2023b\Src\Simulator_3D)
            functionPath = fullfile(rootPath, 'Src', 'Functions', 'Utilities');

            % 4. Add the paths to MATLAB's search path
            addpath(functionPath);

            % 5. Store the paths so we can remove it later in TestClassTeardown
            testCase.AddedPath = {functionPath};
        end

        
        function testFileParsing(testCase)
            % Create a temporary test file
            fileName = 'testSimOutput.txt';
            fid = fopen(fileName, 'w');
            fprintf(fid, 'Margin 2.5\nAlpha 1.2\nCd 0.45\n');
            fclose(fid);
            
            % Run reader
            simOutput = simOutputReader(fileName);
            
            % Verify results
            testCase.verifyEqual(simOutput.margin, 2.5);
            testCase.verifyEqual(simOutput.alpha, 1.2);
            testCase.verifyEqual(simOutput.cd, 0.45);
            
            % Cleanup
            delete(fileName);
        end
    end
end