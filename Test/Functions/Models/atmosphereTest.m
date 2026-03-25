classdef atmosphereTest < matlab.unittest.TestCase
    % Test suite for the atmosphere function and its helper functions
    
    properties
        % Path to tested function in src
        AddedPath;
        % Sample environment structure for testing
        env;

        % constants
        p0 = 101325;                    % [Pa] Pressure at sea level
        g = 9.80665;                    % [m/sec^2] Gravity at sea level
        R_star = 287.04;                % [J / (kg K)] Real gas constant of air (R0 / M_air)
        gamma = 1.4;                    % [-] Specific heat coefficient of air
        defaultTemperature = 186.95;    % Default temperature if altitude is out of bounds
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

        function createTestEnvironment(testCase)
            % Create a standard environment for testing
            testCase.env.Temperature_Ground = 288; % ~15°C in Kelvin
            testCase.env.Humidity_Ground = 0.5; % 50% humidity
            testCase.env.Saturation_Vapor_Ratio = 0.622; % Typical value
        end
    end
    
    methods (TestClassTeardown)
        function removeFunctionPath(testCase)
            % This removes the path added in TestClassSetup
            rmpath(testCase.AddedPath);
        end
    end
    
    methods(Test)
        %% Test complete atmosphere function
        function testAtmosphereAtSeaLevel(testCase)
            % Test all outputs at sea level
            alt = 0;
            
            [T, a, p, rho, nu] = atmosphere(alt, testCase.env);
            
            % Verify each output
            testCase.verifyEqual(T, testCase.env.Temperature_Ground, ...
                'RelTol', 1e-10, 'Sea level temperature incorrect');
            
            testCase.verifyEqual(p, 101325, 'RelTol', 0.01, ...
                'Sea level pressure incorrect'); % 1% tolerance
            
            % Speed of sound at sea level should be ~340 m/s
            testCase.verifyEqual(a, sqrt(1.4 * 287.04 * T), ...
                'RelTol', 1e-10, 'Speed of sound calculation incorrect');
            
            % Density at sea level should be ~1.225 kg/m³
            testCase.verifyGreaterThan(rho, 1, ...
                'Sea level density too low');
            testCase.verifyLessThan(rho, 1.3, ...
                'Sea level density too high');
            
            % Viscosity should be positive
            testCase.verifyGreaterThan(nu, 0, ...
                'Viscosity should be positive');
        end
        
        function testAtmospherePressureAtHighAltitude(testCase)
            % Test pressure above 86000m (should be zero)
            alt = 90000;
            
            [~, ~, p, ~, ~] = atmosphere(alt, testCase.env);
            
            testCase.verifyEqual(p, 0, 'RelTol', 1e-10, ...
                'Pressure above 86000m should be zero');
        end
        
        function testAtmosphereNegativeAltitude(testCase)
            % Test with negative altitude (should still work)
            alt = -100;
            
            [T, ~, p, ~, ~] = atmosphere(alt, testCase.env);

            % Negative altitude should return the default temperature
            testCase.verifyEqual(T, testCase.defaultTemperature, ...
                'Expected a different default temperature for negative altitudes');
            
            % Negative altitude should return the sea-level pressure
            testCase.verifyEqual(p, testCase.p0, ...
                'Expected a different default pressure for negative altitudes');
        end
        
        %% Test physical consistency
        function testIdealGasLawConsistency(testCase)
            % Verify ideal gas law consistency (with humidity correction)
            altitudes = [0, 5000, 11000, 20000];
            
            R = 287.04;
            x = testCase.env.Saturation_Vapor_Ratio * testCase.env.Humidity_Ground;
            
            for i = 1:length(altitudes)
                alt = altitudes(i);
                [T, ~, p, rho, ~] = atmosphere(alt, testCase.env);
                
                if p > 0
                    % Calculate density from ideal gas law
                    rho_calc = p / (T * R) * (1 + x) / (1 + 1.609 * x);
                    
                    testCase.verifyEqual(rho, rho_calc, 'RelTol', 1e-10, ...
                        sprintf('Ideal gas law mismatch at %dm', alt));
                end
            end
        end
        
        function testSpeedOfSoundConsistency(testCase)
            % Verify speed of sound calculation
            altitudes = [0, 5000, 11000, 20000, 32000];
            
            for i = 1:length(altitudes)
                alt = altitudes(i);
                [T, a, ~, ~, ~] = atmosphere(alt, testCase.env);
                
                a_calc = sqrt(testCase.gamma * testCase.R_star * T);
                
                testCase.verifyEqual(a, a_calc, 'RelTol', 1e-10, ...
                    sprintf('Speed of sound mismatch at %dm', alt));
            end
        end
        
        function testMonotonicPressure(testCase)
            % Verify pressure always decreases with altitude
            altitudes = linspace(0, 85000, 100);

            p = zeros(1,100);

            for i = 1:100
                [~, ~, p(i), ~, ~] = atmosphere(altitudes(i), testCase.env);
            end

            % Check that pressure is strictly decreasing
            pressure_diff = diff(p);
            
            % Allow small numerical errors, but overall should be negative
            testCase.verifyTrue(all(pressure_diff < 1e-6), ...
                'Pressure should be non-increasing with altitude');
            
            % Check that most differences are significantly negative
            negative_count = sum(pressure_diff < -1e-10);
            testCase.verifyGreaterThan(negative_count, length(altitudes)/2, ...
                'Pressure should generally decrease with altitude');
        end
        
        %% Test consistency of calculations
        function testConsistentOutputs(testCase)
            % Test that the function produces consistent outputs over time

            % Test for different altitudes
            altitudes = [0, 5000, 11000, 20000, 32000];
            
            % Run function twice and compare
            for i = 1:length(altitudes)
                alt = altitudes(i);
                
                % First run
                [T1, a1, p1, rho1, nu1] = atmosphere(alt, testCase.env);
                
                % Second run (should be identical)
                [T2, a2, p2, rho2, nu2] = atmosphere(alt, testCase.env);
                
                % Verify consistency
                testCase.verifyEqual(T1, T2, 'AbsTol', 1e-10, ...
                    sprintf('Temperature not consistent between runs at %dm', alt));
                testCase.verifyEqual(a1, a2, 'AbsTol', 1e-10, ...
                    sprintf('Speed of sound not consistent at %dm', alt));
                testCase.verifyEqual(p1, p2, 'AbsTol', 1e-10, ...
                    sprintf('Pressure not consistent at %dm', alt));
                testCase.verifyEqual(rho1, rho2, 'AbsTol', 1e-10, ...
                    sprintf('Density not consistent at %dm', alt));
                testCase.verifyEqual(nu1, nu2, 'AbsTol', 1e-10, ...
                    sprintf('Viscosity not consistent at %dm', alt));
            end
        end
        
        function testViscosityCalculation(testCase)
            % Test viscosity calculation consistency
            altitudes = [0, 5000, 11000, 20000];
            
            for i = 1:length(altitudes)
                alt = altitudes(i);
                [T, ~, p, rho, nu] = atmosphere(alt, testCase.env);
                
                if p > 0
                    % Calculate dynamic viscosity using Sutherland's formula
                    T0 = testCase.env.Temperature_Ground;
                    mu = 1.715e-5 * (T/T0)^1.5 * (T0 + 110.4) / (T + 110.4);
                    
                    % Calculate kinematic viscosity
                    nu_calc = mu / rho;
                    
                    testCase.verifyEqual(nu, nu_calc, 'RelTol', 1e-10, ...
                        sprintf('Viscosity mismatch at %dm', alt));
                    
                    % Verify viscosity is positive
                    testCase.verifyGreaterThan(nu, 0, ...
                        sprintf('Viscosity should be positive at %dm', alt));
                end
            end
        end
    end
    
    methods(Test, TestTags = {'Visualization'})
        %% Visualization tests (run these separately if needed)
        function plotAtmosphericProfiles(testCase)
            % Create altitude vector
            altitudes = linspace(0, 85000, 1000);
            
            % Calculate atmospheric properties
            T = zeros(1,1000);
            a = zeros(1,1000);
            p = zeros(1,1000);
            rho = zeros(1,1000);
            nu = zeros(1,1000);
            for i = 1:1000
                [T(i), a(i), p(i), rho(i), nu(i)] = atmosphere(altitudes(i), testCase.env);
            end
            
            % Create figure
            fig = figure('Visible', 'off');
            
            % Temperature profile
            subplot(2, 3, 1);
            semilogx(T, altitudes/1000, 'b-', 'LineWidth', 2);
            xlabel('Temperature (K)');
            ylabel('Altitude (km)');
            title('Temperature Profile');
            grid on;
            
            % Pressure profile
            subplot(2, 3, 2);
            loglog(p, altitudes/1000, 'r-', 'LineWidth', 2);
            xlabel('Pressure (Pa)');
            ylabel('Altitude (km)');
            title('Pressure Profile');
            grid on;
            
            % Density profile
            subplot(2, 3, 3);
            loglog(rho, altitudes/1000, 'g-', 'LineWidth', 2);
            xlabel('Density (kg/m³)');
            ylabel('Altitude (km)');
            title('Density Profile');
            grid on;
            
            % Speed of sound profile
            subplot(2, 3, 4);
            semilogx(a, altitudes/1000, 'm-', 'LineWidth', 2);
            xlabel('Speed of Sound (m/s)');
            ylabel('Altitude (km)');
            title('Speed of Sound Profile');
            grid on;
            
            % Viscosity profile
            subplot(2, 3, 5);
            loglog(nu, altitudes/1000, 'c-', 'LineWidth', 2);
            xlabel('Kinematic Viscosity (m²/s)');
            ylabel('Altitude (km)');
            title('Viscosity Profile');
            grid on;
            
            % Combined normalized plot
            subplot(2, 3, 6);
            hold on;
            plot(T/max(T), altitudes/1000, 'b-', 'LineWidth', 2);
            plot(p/max(p), altitudes/1000, 'r-', 'LineWidth', 2);
            plot(rho/max(rho), altitudes/1000, 'g-', 'LineWidth', 2);
            xlabel('Normalized Values');
            ylabel('Altitude (km)');
            title('Normalized Profiles');
            legend('Temperature', 'Pressure', 'Density', 'Location', 'best');
            grid on;
            hold off;
            
            sgtitle('International Standard Atmosphere Profiles');
            
            % Verify the figure was created
            testCase.verifyClass(fig, 'matlab.ui.Figure', ...
                'Figure should be created');
            
            close(fig);
        end
    end
end