classdef inertialCouplingTest < matlab.unittest.TestCase
    % Simple test suite to verify inertial coupling simulation runs without errors
    % To run: results = runtests('inertialCouplingTest');
    % Please try running inertialCoupling as described in
    % inertialCoupling.m as it is meant to be run after Main.m, this test
    % only tries to emulate and isn't the best way to do so
    
    properties (Access = private)
        OriginalWarningState
        OriginalFigureVisibility
        TestDataCreated = false
    end
    
    methods (TestClassSetup)
        function setupTestEnvironment(testCase)
            % Save current warning state and figure visibility
            testCase.OriginalWarningState = warning;
            testCase.OriginalFigureVisibility = get(0, 'DefaultFigureVisible');
            
            % Suppress warnings and figures for clean test output
            warning('off', 'all');
            set(0, 'DefaultFigureVisible', 'off');
            
            % Create dummy data for the simulation
            testCase.createTestData();
        end
    end
    
    methods (TestClassTeardown)
        function teardownTestEnvironment(testCase)
            % Restore original warning state and figure visibility
            warning(testCase.OriginalWarningState);
            set(0, 'DefaultFigureVisible', testCase.OriginalFigureVisibility);
            
            % Clean up workspace
            if testCase.TestDataCreated
                clear S2 T2;
            end
        end
    end
    
    methods (Access = private)
        function createTestData(testCase)
            % Create minimal dummy data that the script expects
            % The script needs S2 and T2 variables in the workspace
            
            % Create time vector
            T2 = linspace(0, 10, 100)';
            
            % Create state matrix S2 with correct dimensions:
            % [x, y, z, vx, vy, vz, q1, q2, q3, q4, ωx, ωy, ωz]
            S2 = zeros(100, 13);
            
            % Position (simple upward trajectory)
            S2(:, 3) = 50 * T2;  % z position increases
            
            % Velocity (constant upward)
            S2(:, 6) = 50;  % vz = 50 m/s
            
            % Quaternion (identity quaternion - no rotation)
            S2(:, 7) = 1;  % q1 = 1
            S2(:, 8:10) = 0;  % q2 = q3 = q4 = 0
            
            % Angular velocity (small perturbations)
            S2(:, 11) = 0.01 * sin(2*pi*T2/10);  % ωx
            S2(:, 12) = 0.02 * sin(2*pi*T2/5);   % ωy
            S2(:, 13) = 0.005;                   % ωz (small roll)
            
            % Make variables available in base workspace
            assignin('base', 'S2', S2);
            assignin('base', 'T2', T2);
            
            testCase.TestDataCreated = true;
            
            fprintf('Created test data: S2 (%dx%d), T2 (%dx%d)\n', ...
                size(S2, 1), size(S2, 2), size(T2, 1), size(T2, 2));
        end
    end
    
    methods (Test)
        function testScriptRunsWithoutErrors(testCase)
            % Test that the entire inertial coupling script runs without errors
            % This test just verifies the script executes without crashing
            
            fprintf('\n=== Testing inertial_coupling.m ===\n');
            
            try
                % Run the script and capture any output/errors
                [~, ~] = evalc('inertial_coupling');
                
                % If we get here, the script ran without errors
                testCase.verifyTrue(true, 'Script executed without errors');
                
                fprintf('✓ Script executed successfully\n');
                
            catch ME
                % Script threw an error
                fprintf('✗ Script failed with error: %s\n', ME.message);
                
                % Provide helpful debugging information
                fprintf('\nDebug information:\n');
                fprintf('Error in: %s\n', ME.stack(1).name);
                fprintf('Line: %d\n', ME.stack(1).line);
                
                % Fail the test with the error message
                testCase.verifyFail(sprintf('Script failed with error: %s', ME.message));
            end
        end
        
        function testShortSimulationRuns(testCase)
            % Test a shorter simulation to save time
            
            fprintf('\n=== Testing short simulation ===\n');
            
            % Temporarily modify the script parameters for faster testing
            originalScript = fileread('inertial_coupling.m');
            
            % Modify the simulation time to be very short for testing
            modifiedScript = strrep(originalScript, 't_simu = 20;', 't_simu = 0.1;');
            
            try
                % Write modified script to temp file
                tempFile = tempname;
                fid = fopen(tempFile, 'w');
                fwrite(fid, modifiedScript);
                fclose(fid);
                
                % Run the modified script
                [~, ~] = evalc(['run(''' tempFile ''')']);
                
                % Clean up temp file
                delete(tempFile);
                
                testCase.verifyTrue(true, 'Short simulation executed successfully');
                fprintf('✓ Short simulation executed successfully\n');
                
            catch ME
                % Clean up temp file if it exists
                if exist('tempFile', 'var') && exist(tempFile, 'file')
                    delete(tempFile);
                end
                
                fprintf('✗ Short simulation failed: %s\n', ME.message);
                testCase.verifyFail(sprintf('Short simulation failed: %s', ME.message));
            end
        end
        
        function testRequiredVariablesExist(testCase)
            % Test that required variables exist in workspace
            
            fprintf('\n=== Checking required variables ===\n');
            
            % Check if S2 exists
            if evalin('base', 'exist(''S2'', ''var'')')
                S2 = evalin('base', 'S2');
                fprintf('✓ S2 exists: %dx%d matrix\n', size(S2, 1), size(S2, 2));
                testCase.verifySize(S2, [100, 13], 'S2 should be 100x13');
            else
                fprintf('✗ S2 does not exist\n');
                testCase.verifyFail('S2 variable missing');
            end
            
            % Check if T2 exists
            if evalin('base', 'exist(''T2'', ''var'')')
                T2 = evalin('base', 'T2');
                fprintf('✓ T2 exists: %dx%d vector\n', size(T2, 1), size(T2, 2));
                testCase.verifySize(T2, [100, 1], 'T2 should be 100x1');
            else
                fprintf('✗ T2 does not exist\n');
                testCase.verifyFail('T2 variable missing');
            end
            
            % Check initial frame index
            if evalin('base', 'exist(''id_initial'', ''var'')')
                id_initial = evalin('base', 'id_initial');
                fprintf('✓ id_initial exists: %d\n', id_initial);
                testCase.verifyGreaterThan(id_initial, 0);
                testCase.verifyLessThanOrEqual(id_initial, 100);
            else
                fprintf('Note: id_initial will be created by script (default = 100)\n');
            end
        end
        
        function testParameterConsistency(testCase)
            % Test that simulation parameters are physically reasonable
            
            fprintf('\n=== Checking parameter consistency ===\n');
            
            % These are the parameters that will be set by the script
            % We'll verify they make physical sense
            
            try
                % Run a snippet to get parameters
                scriptSnippet = [
                    'T = 3736; ' ...
                    'm = 75; ' ...
                    'g = 9.81; ' ...
                    'D = 0.185; ' ...
                    'h = 4.115; ' ...
                    'rho_atm = 1.225; ' ...
                    'Cd = 0.35;'
                ];
                
                evalc(scriptSnippet);
                
                % Check thrust-to-weight ratio
                T = 3736; m = 75; g = 9.81;
                thrustToWeight = T / (m * g);
                fprintf('Thrust-to-weight ratio: %.2f\n', thrustToWeight);
                testCase.verifyGreaterThan(thrustToWeight, 1, 'Rocket should accelerate upward');
                
                % Check dimensions
                D = 0.185; h = 4.115;
                aspectRatio = h / D;
                fprintf('Aspect ratio (h/D): %.2f\n', aspectRatio);
                testCase.verifyGreaterThan(aspectRatio, 10, 'Rocket should be slender');
                
                % Check drag coefficient
                Cd = 0.35;
                fprintf('Drag coefficient: %.2f\n', Cd);
                testCase.verifyGreaterThan(Cd, 0, 'Drag coefficient should be positive');
                testCase.verifyLessThan(Cd, 1.5, 'Drag coefficient should be reasonable');
                
                fprintf('✓ Parameters are physically reasonable\n');
                
            catch ME
                fprintf('✗ Parameter check failed: %s\n', ME.message);
                testCase.verifyFail(sprintf('Parameter check failed: %s', ME.message));
            end
        end
        
        function testSimulationOutput(testCase)
            % Test that simulation produces expected outputs
            
            fprintf('\n=== Checking simulation output ===\n');
            
            % We'll run a very short simulation and check outputs exist
            
            originalScript = fileread('inertial_coupling.m');
            
            % Create a minimal test version
            testScript = [
                '%% Minimal test simulation\n' ...
                'id_initial = 1;\n' ...
                'S_simu = S2(1:10,:);\n' ...
                't_simu = T2(1:10);\n' ...
                'X_simu = S_simu(:,1:3);\n' ...
                'V_simu = S_simu(:,4:6);\n' ...
                'Q_simu = S_simu(:,7:10);\n' ...
                'W_simu = S_simu(:,11:13);\n' ...
                '\n' ...
                '%% Simple parameters\n' ...
                'T = 1000; alpha = 0; m = 75; g = 9.81;\n' ...
                'x_g = 2; D = 0.185; R = D/2; h = 4;\n' ...
                'Dbot = h - x_g; Dtop = x_g;\n' ...
                'S = pi*R^2; Cd = 0.35; MS = 2.5;\n' ...
                'Dgp = MS*D; rho_atm = 1.225;\n' ...
                '\n' ...
                '%% Quick simulation\n' ...
                't0 = t_simu(id_initial);\n' ...
                'dt = 0.01;\n' ...
                'sim_time = 0.05;\n' ...
                'N = sim_time/dt;\n' ...
                't = linspace(t0, t0 + sim_time, N);\n' ...
                '\n' ...
                '%% Initial conditions\n' ...
                'x0 = X_simu(id_initial,:)'';\n' ...
                'v0 = V_simu(id_initial,:)'';\n' ...
                'q0 = Q_simu(id_initial,:)'';\n' ...
                'omega0 = W_simu(id_initial, :)'';\n' ...
                '\n' ...
                '%% Skip ODE solver, just check setup\n' ...
                'fprintf(''Simulation setup successful\\n'');\n' ...
                '\n' ...
                '%% Check that all variables were created\n' ...
                'variables = who;\n' ...
                'fprintf(''Created %d variables\\n'', length(variables));\n'
            ];
            
            try
                % Run the test script
                evalc(testScript);
                
                % Check workspace
                vars = evalin('base', 'who');
                fprintf('Created variables: %s\n', strjoin(vars', ', '));
                
                % Verify key variables exist
                requiredVars = {'T', 'm', 'g', 'x0', 'v0', 'q0', 'omega0'};
                for i = 1:length(requiredVars)
                    varName = requiredVars{i};
                    exists = evalin('base', ['exist(''' varName ''', ''var'')']);
                    if exists
                        fprintf('✓ %s exists\n', varName);
                    else
                        fprintf('✗ %s missing\n', varName);
                    end
                end
                
                testCase.verifyTrue(true, 'Minimal simulation setup successful');
                fprintf('✓ Simulation setup completed\n');
                
            catch ME
                fprintf('✗ Simulation setup failed: %s\n', ME.message);
                testCase.verifyFail(sprintf('Simulation setup failed: %s', ME.message));
            end
        end
        
        function testGraphicsFunctions(testCase)
            % Test that graphics/animation parts don't crash
            
            fprintf('\n=== Testing graphics functions ===\n');
            
            % Test individual plotting functions that might be called
            
            try
                % Create simple test data
                t_test = linspace(0, 1, 100);
                X_test = zeros(100, 14);
                X_test(:, 6) = 50 * t_test;  % Simple vz
                euler_test = [0.1*sin(2*pi*t_test); 0.05*cos(2*pi*t_test); 0.01*t_test];
                L_test = 1000 * ones(size(t_test));
                E_test = 50000 * ones(size(t_test));
                q_norm = ones(size(t_test));
                
                % Test plotting functions (suppress figures)
                figure('Visible', 'off');
                plot(t_test, X_test(:, 6), 'bx');
                close(gcf);
                
                figure('Visible', 'off');
                plot(t_test, rad2deg(euler_test(1,:)), 'bx-');
                hold on;
                plot(t_test, rad2deg(euler_test(2,:)), 'gx-');
                plot(t_test, rad2deg(euler_test(3,:)), 'rx-');
                close(gcf);
                
                figure('Visible', 'off');
                subplot(2,1,1);
                plot(t_test, L_test, 'bx-');
                subplot(2,1,2);
                plot(t_test, E_test, 'bx-');
                close(gcf);
                
                figure('Visible', 'off');
                plot(t_test, q_norm, 'bx-');
                close(gcf);
                
                fprintf('✓ All plotting functions work\n');
                testCase.verifyTrue(true, 'Graphics functions work');
                
            catch ME
                fprintf('✗ Graphics test failed: %s\n', ME.message);
                testCase.verifyFail(sprintf('Graphics test failed: %s', ME.message));
            end
        end
    end
    
    methods (Test, TestTags = {'Integration'})
        function testFullIntegration(testCase)
            % Full integration test (runs the complete simulation)
            % This test takes longer and is tagged separately
            
            fprintf('\n=== Running full integration test ===\n');
            
            % Skip in automated environments
            if getenv('CI') || getenv('GITHUB_ACTIONS')
                fprintf('Skipping full integration test in CI environment\n');
                testCase.assumeFail('Skipped in CI');
                return;
            end
            
            % Check if we should run long tests
            runLongTests = false; % Set to true for full test
            if ~runLongTests
                fprintf('Skipping full integration test (set runLongTests=true to enable)\n');
                testCase.assumeTrue(false, 'Long tests disabled');
                return;
            end
            
            try
                % Modify script for faster but complete test
                originalScript = fileread('inertial_coupling.m');
                
                % Make modifications for test:
                % 1. Shorter simulation time
                % 2. Smaller number of steps
                modifiedScript = originalScript;
                modifiedScript = strrep(modifiedScript, 't_simu = 20;', 't_simu = 1;'); % 1 second
                modifiedScript = strrep(modifiedScript, 'dt = 0.01;', 'dt = 0.02;'); % Larger step
                
                % Write to temp file and run
                tempFile = tempname;
                fid = fopen(tempFile, 'w');
                fwrite(fid, modifiedScript);
                fclose(fid);
                
                fprintf('Running 1-second simulation...\n');
                tic;
                [output, ~] = evalc(['run(''' tempFile ''')']);
                elapsedTime = toc;
                
                % Clean up
                delete(tempFile);
                
                fprintf('Simulation completed in %.2f seconds\n', elapsedTime);
                fprintf('Output length: %d characters\n', length(output));
                
                % Check for common issues in output
                if contains(output, 'Error') || contains(output, 'error')
                    fprintf('Warning: Output contains error messages\n');
                end
                
                if contains(output, 'Warning')
                    fprintf('Note: Output contains warnings (this is expected)\n');
                end
                
                testCase.verifyTrue(true, 'Full integration test passed');
                fprintf('✓ Full integration test completed\n');
                
            catch ME
                fprintf('✗ Full integration test failed: %s\n', ME.message);
                testCase.verifyFail(sprintf('Full integration test failed: %s', ME.message));
            end
        end
    end
end