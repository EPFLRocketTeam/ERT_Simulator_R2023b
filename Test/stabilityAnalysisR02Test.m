classdef stabilityAnalysisR02Test < matlab.unittest.TestCase
    % Test class for the stability analysis calculation.
    % To run, type 'runtests('stabilityAnalysisR02Test')' in the Command Window.
    
    properties (Access = private)
        analysisResults;
    end
    
    methods (TestClassSetup)
        function runAnalysis(testCase)
            % Execute the main analysis script and store results
            % Note: stabilityAnalysisR02 must be in the MATLAB path or current folder
            addpath('../Src');
            % The stabilityAnalysisR02 function returns a struct with field names
            % like vNom, stabilityNom, epsilonNom, etc.
            testCase.analysisResults = stabilityAnalysisR02(); 
        end
    end
    
    methods (Test)
        
        % =================================================================
        % Nominal Case Tests
        % =================================================================
        
        function testNominalSpeed(testCase)
            % Check V_nom >= 20
            vNomActual = testCase.analysisResults.vNom;
            diagnostic = sprintf(...
                'Nominal Speed Error: Speed off rail must be >= 20 m/s. Actual: %f', ...
                vNomActual);
            testCase.verifyGreaterThanOrEqual(vNomActual, 20, diagnostic);
        end
        
        function testNominalStabilityMargin(testCase)
            % Check (P-W)/d >= 1.5
            stabilityNomActual = testCase.analysisResults.stabilityNom;
            diagnostic = sprintf(...
                'Nominal Stability Error: Static Margin (P-W)/d must be >= 1.5. Actual: %f', ...
                stabilityNomActual);
            testCase.verifyGreaterThanOrEqual(stabilityNomActual, 1.5, diagnostic);
        end
        
        function testNominalDampingRatio(testCase)
            % Check epsilon >= 0.05 and epsilon < 0.3
            epsilonNomActual = testCase.analysisResults.epsilonNom;
            diagnostic = sprintf(...
                'Nominal Damping Ratio Error: Epsilon must be >= 0.05. Actual: %f', ...
                epsilonNomActual);
            testCase.verifyGreaterThanOrEqual(epsilonNomActual, 0.05, diagnostic);
            diagnostic = sprintf(...
                'Nominal Damping Ratio Error: Epsilon must be < 0.3. Actual: %f', ...
                epsilonNomActual);
            testCase.verifyLessThan(epsilonNomActual, 0.3, diagnostic);
        end
        
        % =================================================================
        % Max Speed Case Tests
        % =================================================================
        
        function testMaxSpeedSpeed(testCase)
            % Check V_max >= 20
            vMaxActual = testCase.analysisResults.vMax;
            diagnostic = sprintf(...
                'Max Speed Error: Max speed must be >= 20 m/s. Actual: %f', ...
                vMaxActual);
            testCase.verifyGreaterThanOrEqual(vMaxActual, 20, diagnostic);
        end
        
        function testMaxSpeedStabilityMargin(testCase)
            % Check (P-W)/d >= 1.5
            stabilityMaxActual = testCase.analysisResults.stabilityMax;
            diagnostic = sprintf(...
                'Max Speed Stability Error: Static Margin (P-W)/d must be >= 1.5. Actual: %f', ...
                stabilityMaxActual);
            testCase.verifyGreaterThanOrEqual(stabilityMaxActual, 1.5, diagnostic);
        end
        
        function testMaxSpeedDampingRatio(testCase)
            % Check epsilon >= 0.05 and epsilon < 0.3
            epsilonMaxActual = testCase.analysisResults.epsilonMax;
            diagnostic = sprintf(...
                'Max Speed Damping Ratio Error: Epsilon must be >= 0.05. Actual: %f', ...
                epsilonMaxActual);
            testCase.verifyGreaterThanOrEqual(epsilonMaxActual, 0.05, diagnostic);
            diagnostic = sprintf(...
                'Max Speed Damping Ratio Error: Epsilon must be < 0.3. Actual: %f', ...
                epsilonMaxActual);
            testCase.verifyLessThan(epsilonMaxActual, 0.3, diagnostic);
        end
        
        % =================================================================
        % Worst Case (Rail Exit) Tests
        % =================================================================
        
        function testWorstCaseRailSpeed(testCase)
            % Check V_wc_rail >= 20
            vWcRailActual = testCase.analysisResults.vWcRail;
            diagnostic = sprintf(...
                'Worst Case Rail Speed Error: Speed off rail must be >= 20 m/s. Actual: %f', ...
                vWcRailActual);
            testCase.verifyGreaterThanOrEqual(vWcRailActual, 20, diagnostic);
        end
        
        function testWorstCaseRailStabilityMargin(testCase)
            % Check (P-W)/d >= 1.5
            stabilityWcRailActual = testCase.analysisResults.stabilityWcRail;
            diagnostic = sprintf(...
                'Worst Case Rail Stability Error: Static Margin (P-W)/d must be >= 1.5. Actual: %f', ...
                stabilityWcRailActual);
            testCase.verifyGreaterThanOrEqual(stabilityWcRailActual, 1.5, diagnostic);
        end
        
        function testWorstCaseRailDampingRatio(testCase)
            % Check epsilon >= 0.05 and epsilon < 0.3
            epsilonWcRailActual = testCase.analysisResults.epsilonWcRail;
            diagnostic = sprintf(...
                'Worst Case Rail Damping Ratio Error: Epsilon must be >= 0.05. Actual: %f', ...
                epsilonWcRailActual);
            testCase.verifyGreaterThanOrEqual(epsilonWcRailActual, 0.05, diagnostic);
            diagnostic = sprintf(...
                'Worst Case Rail Damping Ratio Error: Epsilon must be < 0.3. Actual: %f', ...
                epsilonWcRailActual);
            testCase.verifyLessThan(epsilonWcRailActual, 0.3, diagnostic);
        end
        
        % =================================================================
        % Worst Case Max Speed Tests
        % =================================================================
        
        function testWorstCaseMaxSpeedSpeed(testCase)
            % Check V_wc_max >= 20
            vWcMaxActual = testCase.analysisResults.vWcMax;
            diagnostic = sprintf(...
                'Worst Case Max Speed Error: Max speed must be >= 20 m/s. Actual: %f', ...
                vWcMaxActual);
            testCase.verifyGreaterThanOrEqual(vWcMaxActual, 20, diagnostic);
        end
        
        function testWorstCaseMaxSpeedStabilityMargin(testCase)
            % Check (P-W)/d >= 1.5
            stabilityWcMaxActual = testCase.analysisResults.stabilityWcMax;
            diagnostic = sprintf(...
                'Worst Case Max Speed Stability Error: Static Margin (P-W)/d must be >= 1.5. Actual: %f',...
                stabilityWcMaxActual);
            testCase.verifyGreaterThanOrEqual(stabilityWcMaxActual, 1.5, diagnostic);
        end
        
        function testWorstCaseMaxSpeedDampingRatio(testCase)
            % Check epsilon >= 0.05 and epsilon < 0.3
            epsilonWcMaxActual = testCase.analysisResults.epsilonWcMax;
            diagnostic = sprintf(...
                'Worst Case Max Speed Damping Ratio Error: Epsilon must be >= 0.05. Actual: %f',...
                epsilonWcMaxActual);
            testCase.verifyGreaterThanOrEqual(epsilonWcMaxActual, 0.05, diagnostic);
            diagnostic = sprintf(...
                'Worst Case Max Speed Damping Ratio Error: Epsilon must be < 0.3. Actual: %f', ...
                epsilonWcMaxActual);
            testCase.verifyLessThan(epsilonWcMaxActual, 0.3, diagnostic);
        end
        
        % =================================================================
        % Extra Value Tests
        % =================================================================
        
        function testMinStaticMarginNominal(testCase)
            % Check Min Static Margin (cut) is positive and reasonable
            minStabilityNomActual = testCase.analysisResults.minStabilityNom;
            diagnostic = sprintf(...
                'Nominal Min Static Margin Error: Should not be negative. Actual: %f', ...
                minStabilityNomActual);
            testCase.verifyGreaterThanOrEqual(minStabilityNomActual, 0, diagnostic);
        end
        
    end
end