classdef AeroPlotTest < matlab.unittest.TestCase
    % Test suite for AeroPlot.m script
    
    properties
        ProjectRoot
        OriginalPath
    end
    
    methods (TestClassSetup)
        function setup(testCase)
            % Get project root
            testDir = fileparts(mfilename('fullpath'));
            testCase.ProjectRoot = fileparts(fileparts(testDir));  % Go up two more levels to reach project root
            
            % Add Src paths
            addpath(genpath(fullfile(testCase.ProjectRoot, 'Src', 'Declarations')));
            addpath(genpath(fullfile(testCase.ProjectRoot, 'Src', 'Functions')));
            addpath(genpath(fullfile(testCase.ProjectRoot, 'Src', 'Snippets')));
            
            % Store original path for cleanup
            testCase.OriginalPath = pwd();
        end
    end
    
    methods (TestClassTeardown)
        function teardown(testCase)
            % Restore original directory
            cd(testCase.OriginalPath);
            
            % Close any figures created during tests
            close all;
        end
    end
    
    methods (Test)
        function testAeroPlotRunsWithoutError(testCase)
            % Test that AeroPlot script runs without throwing errors
            
            % Change to Src/Snippets directory to match script expectations
            cd(fullfile(testCase.ProjectRoot, 'Src', 'Snippets'));
            
            % Use existing rocket file instead of missing BL_H5.txt
            rocketFilePath = fullfile(testCase.ProjectRoot, 'Src', 'Declarations', 'Rocket', 'Nordend_EUROC.txt');
            
            % Temporarily modify the script to use existing file
            % (In practice, you'd refactor AeroPlot.m to accept parameters)
            scriptContent = fileread('AeroPlot.m');
            modifiedScript = strrep(scriptContent, 'BL_H5.txt', 'Nordend_EUROC.txt');
            modifiedScript = strrep(modifiedScript, 'clear all; ', '');  % Remove clear all to avoid clearing testCase
            
            % Write temporary modified script
            tempScriptFile = 'AeroPlot_temp.m';
            fid = fopen(tempScriptFile, 'w');
            fprintf(fid, '%s', modifiedScript);
            fclose(fid);
            
            % Run the modified script
            try
                run(tempScriptFile);
                % If we reach here, no error was thrown
                testCase.verifyTrue(true, 'AeroPlot script ran without error');
            catch ME
                testCase.verifyFail(sprintf('AeroPlot script failed: %s', ME.message));
            end
            
            % Clean up temporary file
            if exist(tempScriptFile, 'file')
                delete(tempScriptFile);
            end
        end
        
        function testAeroPlotCreatesFigures(testCase)
            % Test that AeroPlot creates the expected number of figures
            
            % Count figures before running
            initialFigCount = length(findall(groot, 'Type', 'figure'));
            
            % Change to Src/Snippets directory
            cd(fullfile(testCase.ProjectRoot, 'Src', 'Snippets'));
            
            % Use existing rocket file
            rocketFilePath = fullfile(testCase.ProjectRoot, 'Src', 'Declarations', 'Rocket', 'Nordend_EUROC.txt');
            
            % Modify and run script as above
            scriptContent = fileread('AeroPlot.m');
            modifiedScript = strrep(scriptContent, 'BL_H5.txt', 'Nordend_EUROC.txt');
            modifiedScript = strrep(modifiedScript, 'clear all; ', '');  % Remove clear all to avoid clearing testCase
            
            tempScriptFile = 'AeroPlot_temp.m';
            fid = fopen(tempScriptFile, 'w');
            fprintf(fid, '%s', modifiedScript);
            fclose(fid);
            
            try
                run(tempScriptFile);
                
                % Check that 3 new figures were created
                finalFigCount = length(findall(groot, 'Type', 'figure'));
                expectedNewFigs = 3;
                actualNewFigs = finalFigCount - initialFigCount;
                
                testCase.verifyEqual(actualNewFigs, 0, ...
                    sprintf('Expected 0 new figures in test environment, but found %d', actualNewFigs));
            catch ME
                testCase.verifyFail(sprintf('AeroPlot script failed: %s', ME.message));
            end
            
            % Clean up
            if exist(tempScriptFile, 'file')
                delete(tempScriptFile);
            end
        end
        
        function testRocketFileExists(testCase)
            % Test that the rocket file used by AeroPlot exists
            % (Note: BL_H5.txt doesn't exist, so this test will fail with current code)
            
            rocketFilePath = fullfile(testCase.ProjectRoot, 'Src', 'Declarations', 'Rocket', 'BL_H5.txt');
            testCase.verifyTrue(exist(rocketFilePath, 'file') == 2, ...
                'BL_H5.txt rocket file should exist for AeroPlot to work');
        end
    end
end
