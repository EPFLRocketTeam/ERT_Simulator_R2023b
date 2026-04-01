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
            testCase.runModifiedAeroPlot('Nordend_EUROC.txt');
            testCase.verifyTrue(true, 'AeroPlot script ran without error');
        end

        function testAeroPlotCreatesFigures(testCase)
            % Test that AeroPlot creates the expected number of figures
            initialFigCount = length(findall(groot, 'Type', 'figure'));

            testCase.runModifiedAeroPlot('Nordend_EUROC.txt');

            finalFigCount = length(findall(groot, 'Type', 'figure'));
            actualNewFigs = finalFigCount - initialFigCount;
            expectedNewFigs = 3;

            testCase.verifyEqual(actualNewFigs, expectedNewFigs, ...
                sprintf('Expected %d new figures, but found %d', expectedNewFigs, actualNewFigs));
        end
    end

    methods (Access = private)
        function runModifiedAeroPlot(testCase, rocketFileName)
            % Helper to run a modified version of AeroPlot with a given rocket file.
            % This function temporarily changes directory to Src/Snippets,
            % modifies the script, runs it, and restores the original directory
            % and cleans up the temporary file.

            % Save original directory and set up cleanup
            originalDir = pwd();
            cleanupObj = onCleanup(@() cd(originalDir));

            % Change to Src/Snippets where AeroPlot.m lives
            snippetsDir = fullfile(testCase.ProjectRoot, 'Src', 'Snippets');
            cd(snippetsDir);

            % Read original script
            scriptContent = fileread('AeroPlot.m');

            % Replace rocket file name and remove 'clear all;' to preserve test state
            modifiedScript = strrep(scriptContent, 'BL_H5.txt', rocketFileName);
            modifiedScript = strrep(modifiedScript, 'clear all; ', '');

            % Create a temporary file
            tempScriptFile = 'AeroPlot_temp.m';
            fid = fopen(tempScriptFile, 'w');
            fprintf(fid, '%s', modifiedScript);
            fclose(fid);

            % Ensure temp file is deleted when the function exits
            cleanupFile = onCleanup(@() deleteIfExists(tempScriptFile));

            % Run the modified script – any error will propagate and cause the test to fail
            run(tempScriptFile);
        end
    end
end

% Helper function to safely delete a file if it exists
function deleteIfExists(fileName)
    if exist(fileName, 'file')
        delete(fileName);
    end
end