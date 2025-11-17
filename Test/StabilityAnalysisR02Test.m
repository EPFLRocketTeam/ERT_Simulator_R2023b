classdef StabilityAnalysisR02Test < matlab.unittest.TestCase
    % Test class for the stability analysis calculation.
    % To run, type 'runtests('stabilityAnalysisTest')' in the Command Window.
    
    % properties (TestParameter)
    %     % Define tolerance for floating point comparisons
    %     Tolerance = {1e-3}; 
    % end
    
    properties (Access = private)
        AnalysisResults;
    end
    
    methods (TestClassSetup)
        function runAnalysis(testCase)
            % Execute the main analysis script and store results
            % Note: stabilityAnalysis must be in the MATLAB path or current folder
            testCase.AnalysisResults = StabilityAnalysis();
        end
    end
    
    methods (Test)
        
        % =================================================================
        % Nominal Case Tests
        % =================================================================
        
        function testNominalSpeed(testCase)
            % Check V_nom >= 20
            V_nom_actual = testCase.AnalysisResults.V_nom;
            diagnostic = sprintf(...
                'Nominal Speed Error: Speed off rail must be >= 20 m/s. Actual: %f', ...
                V_nom_actual);
            testCase.verifyGreaterThanOrEqual(V_nom_actual, 20, diagnostic);
        end
        
        function testNominalStabilityMargin(testCase)
            % Check (P-W)/d >= 1.5
            Stability_nom_actual = testCase.AnalysisResults.Stability_nom;
            diagnostic = sprintf(...
                'Nominal Stability Error: Static Margin (P-W)/d must be >= 1.5. Actual: %f', ...
                Stability_nom_actual);
            testCase.verifyGreaterThanOrEqual(Stability_nom_actual, 1.5, diagnostic);
        end
        
        function testNominalDampingRatio(testCase)
            % Check epsilon >= 0.05 and epsilon < 0.3
            epsilon_nom_actual = testCase.AnalysisResults.epsilon_nom;
            diagnostic = sprintf(...
                'Nominal Damping Ratio Error: Epsilon must be >= 0.05. Actual: %f', ...
                epsilon_nom_actual);
            testCase.verifyGreaterThanOrEqual(epsilon_nom_actual, 0.05, diagnostic);
            diagnostic = sprintf(...
                'Nominal Damping Ratio Error: Epsilon must be < 0.3. Actual: %f', ...
                epsilon_nom_actual);
            testCase.verifyLessThan(epsilon_nom_actual, 0.3, diagnostic);
        end
        
        % =================================================================
        % Max Speed Case Tests
        % =================================================================
        
        function testMaxSpeedSpeed(testCase)
            % Check V_max >= 20
            V_max_actual = testCase.AnalysisResults.V_max;
            diagnostic = sprintf(...
                'Max Speed Error: Max speed must be >= 20 m/s. Actual: %f', ...
                V_max_actual);
            testCase.verifyGreaterThanOrEqual(V_max_actual, 20, diagnostic);
        end
        
        function testMaxSpeedStabilityMargin(testCase)
            % Check (P-W)/d >= 1.5
            Stability_max_actual = testCase.AnalysisResults.Stability_max;
            diagnostic = sprintf(...
                'Max Speed Stability Error: Static Margin (P-W)/d must be >= 1.5. Actual: %f', ...
                Stability_max_actual);
            testCase.verifyGreaterThanOrEqual(Stability_max_actual, 1.5, diagnostic);
        end
        
        function testMaxSpeedDampingRatio(testCase)
            % Check epsilon >= 0.05 and epsilon < 0.3
            epsilon_max_actual = testCase.AnalysisResults.epsilon_max;
            diagnostic = sprintf(...
                'Max Speed Damping Ratio Error: Epsilon must be >= 0.05. Actual: %f', ...
                epsilon_max_actual);
            testCase.verifyGreaterThanOrEqual(epsilon_max_actual, 0.05, diagnostic);
            diagnostic = sprintf(...
                'Max Speed Damping Ratio Error: Epsilon must be < 0.3. Actual: %f', ...
                epsilon_max_actual);
            testCase.verifyLessThan(epsilon_max_actual, 0.3, diagnostic);
        end
        
        % =================================================================
        % Worst Case (Rail Exit) Tests
        % =================================================================
        
        function testWorstCaseRailSpeed(testCase)
            % Check V_wc_rail >= 20
            V_wc_rail_actual = testCase.AnalysisResults.V_wc_rail;
            diagnostic = sprintf(...
                'Worst Case Rail Speed Error: Speed off rail must be >= 20 m/s. Actual: %f', ...
                V_wc_rail_actual);
            testCase.verifyGreaterThanOrEqual(V_wc_rail_actual, 20, diagnostic);
        end
        
        function testWorstCaseRailStabilityMargin(testCase)
            % Check (P-W)/d >= 1.5
            Stability_wc_rail_actual = testCase.AnalysisResults.Stability_wc_rail;
            diagnostic = sprintf(...
                'Worst Case Rail Stability Error: Static Margin (P-W)/d must be >= 1.5. Actual: %f', ...
                Stability_wc_rail_actual);
            testCase.verifyGreaterThanOrEqual(Stability_wc_rail_actual, 1.5, diagnostic);
        end
        
        function testWorstCaseRailDampingRatio(testCase)
            % Check epsilon >= 0.05 and epsilon < 0.3
            epsilon_wc_rail_actual = testCase.AnalysisResults.epsilon_wc_rail;
            diagnostic = sprintf(...
                'Worst Case Rail Damping Ratio Error: Epsilon must be >= 0.05. Actual: %f', ...
                epsilon_wc_rail_actual);
            testCase.verifyGreaterThanOrEqual(epsilon_wc_rail_actual, 0.05, diagnostic);
            diagnostic = sprintf(...
                'Worst Case Rail Damping Ratio Error: Epsilon must be < 0.3. Actual: %f', ...
                epsilon_wc_rail_actual);
            testCase.verifyLessThan(epsilon_wc_rail_actual, 0.3, diagnostic);
        end
        
        % =================================================================
        % Worst Case Max Speed Tests
        % =================================================================
        
        function testWorstCaseMaxSpeedSpeed(testCase)
            % Check V_wc_max >= 20
            V_wc_max_actual = testCase.AnalysisResults.V_wc_max;
            diagnostic = sprintf(...
                'Worst Case Max Speed Error: Max speed must be >= 20 m/s. Actual: %f', ...
                V_wc_max_actual);
            testCase.verifyGreaterThanOrEqual(V_wc_max_actual, 20, diagnostic);
        end
        
        function testWorstCaseMaxSpeedStabilityMargin(testCase)
            % Check (P-W)/d >= 1.5
            Stability_wc_max_actual = testCase.AnalysisResults.Stability_wc_max;
            diagnostic = sprintf(...
                'Worst Case Max Speed Stability Error: Static Margin (P-W)/d must be >= 1.5. Actual: %f',...
                Stability_wc_max_actual);
            testCase.verifyGreaterThanOrEqual(Stability_wc_max_actual, 1.5, diagnostic);
        end
        
        function testWorstCaseMaxSpeedDampingRatio(testCase)
            % Check epsilon >= 0.05 and epsilon < 0.3
            epsilon_wc_max_actual = testCase.AnalysisResults.epsilon_wc_max;
            diagnostic = sprintf(...
                'Worst Case Max Speed Damping Ratio Error: Epsilon must be >= 0.05. Actual: %f',...
                epsilon_wc_max_actual);
            testCase.verifyGreaterThanOrEqual(epsilon_wc_max_actual, 0.05, diagnostic);
            diagnostic = sprintf(...
                'Worst Case Max Speed Damping Ratio Error: Epsilon must be < 0.3. Actual: %f', ...
                epsilon_wc_max_actual);
            testCase.verifyLessThan(epsilon_wc_max_actual, 0.3, diagnostic);
        end
        
        % =================================================================
        % Extra Value Tests
        % =================================================================
        
        function testMinStaticMarginNominal(testCase)
            % Check Min Static Margin (cut) is positive and reasonable
            Min_Stability_nom_actual = testCase.AnalysisResults.Min_Stability_nom;
            diagnostic = sprintf(...
                'Nominal Min Static Margin Error: Should not be negative. Actual: %f', ...
                Min_Stability_nom_actual);
            testCase.verifyGreaterThanOrEqual(Min_Stability_nom_actual, 0, diagnostic);
        end
        
    end
end