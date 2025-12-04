% ------------------------------------------------------------------------
% AEROPLOT - Aerodynamic Analysis and Visualization Tool
% 
% Purpose: Plot aerodynamic coefficients (drag, lift, stability) for rocket
%          analysis across various angles of attack, Mach numbers, and 
%          operational conditions.
% 
% Features:
%   - Drag coefficient (dragCoefficientMatrix) vs Mach number for different angles of attack
%   - Normal force coefficient (normalForceCoefficient) vs angle of attack
%   - Stability margin vs angle of attack
%
% Dependencies:
%   - rocketReader.m: Reads rocket configuration from file
%   - drag.m: Computes drag coefficient
%   - normalLift.m: Computes normal force coefficient and center of pressure
%
% Date : 1.03.2018
% Author : Eric Brunner
% Updated : 30.12.2025 - Added comprehensive comments and error handling
% -------------------------------------------------------------------------

clear all; 
close all; 
clc; % Clear command window for clean output

% -------------------------------------------------------------------------
% 1.0 PATH SETUP AND DEPENDENCY MANAGEMENT
% -------------------------------------------------------------------------
try
    addpath(genpath('../Declarations'),...
            genpath('../Functions'),...
            genpath('../Snippets'));
    fprintf('Successfully added required paths.\n');
catch ME
    error('AeroPlot:PathError', 'Failed to add required paths: %s', ME.message);
end

% -------------------------------------------------------------------------
% 2.0 CONFIGURATION PARAMETERS
% -------------------------------------------------------------------------

% Rocket configuration
readRocketFile = 1;           % Flag to read rocket from file (1=yes, 0=no)
rocketFile = 'BL_H3.txt';    % Rocket configuration file name

% Analysis ranges
velocityRange = linspace(0, 346, 20);    % Velocity range [m/s] (0 to speed of sound)
angleOfAttackRange = linspace(0, pi/4, 5);      % Angle of attack range [rad] (0 to 45°)
reynoldsNumberRange = linspace(0, 1, 10);            % Reynolds number parameter range (unused)

% Atmospheric properties (constant for this analysis)
kineamaticVelocity = 1.5e-5;           % Kinematic viscosity [m²/s]
speedOfSound = 346;                   % Speed of sound at sea level [m/s]

% -------------------------------------------------------------------------
% 3.0 ROCKET CONFIGURATION LOADING
% -------------------------------------------------------------------------
if readRocketFile
   try
       Rocket = rocketReader(rocketFile);
       fprintf('Successfully loaded rocket configuration from: %s\n', rocketFile);
       
       % Display basic rocket information
       fprintf('  Rocket Name: %s\n', Rocket.Name);
       fprintf('  Reference Diameter: %.3f m\n', Rocket.dm);
       fprintf('  Center of Mass: %.3f m\n', Rocket.rocket_cm);
       fprintf('  Reference Area: %.4f m²\n', Rocket.Sm);
   catch ME
       error('aeroPlot:RocketLoadError', ...
             'Failed to load rocket configuration: %s', ME.message);
   end
else
    % Alternative: Use hardcoded rocket structure if not reading from file
    warning('aeroPlot:ManualConfig', 'Using manual rocket configuration.');
    Rocket = struct(); % Create your rocket structure here
end

% -------------------------------------------------------------------------
% 4.0 AERODYNAMIC COEFFICIENT CALCULATION
% -------------------------------------------------------------------------
fprintf('\nComputing aerodynamic coefficients...\n');

% Initialize coefficient matrices
dragCoefficientMatrix = zeros(length(velocityRange), length(angleOfAttackRange));  % Drag coefficient matrix
CNa = zeros(1, length(angleOfAttackRange));                      % Normal force slope
XCP = zeros(1, length(angleOfAttackRange));                      % Center of pressure

% 4.1 Drag Coefficient Calculation (dragCoefficientMatrix vs Mach, for different alpha)
fprintf('  Calculating drag coefficients...\n');
for i = 1:length(velocityRange)
    for j = 1:length(angleOfAttackRange)
        % Calculate drag coefficient for each velocity and angle of attack
        dragCoefficientMatrix(i,j) = drag(Rocket, angleOfAttackRange(j), velocityRange(i), ...
                       kineamaticVelocity, speedOfSound);
    end
    
    % Progress indicator for large calculations
    if mod(i, 5) == 0
        fprintf('    Completed %.0f%% of drag calculations\n', ...
                (i/length(velocityRange))*100);
    end
end

% 4.2 Normal Force and Center of Pressure Calculation
fprintf('  Calculating normal force coefficients...\n');
for j = 1:length(angleOfAttackRange)
    % Calculate normal force coefficient slope and center of pressure
    % Parameters: Rocket, alpha, airbrake deployment, Mach, roll rate, 
    %             useNonlinear flag
    [CNa(j), XCP(j)] = normalLift(Rocket, angleOfAttackRange(j), 1.1, 0, 0, 1);
end

fprintf('Aerodynamic calculations complete.\n\n');

% -------------------------------------------------------------------------
% 5.0 DATA VISUALIZATION
% -------------------------------------------------------------------------
fprintf('Generating plots...\n');

% Set default plotting styles for consistent appearance
set(0, 'defaultaxescolororder', [0 0 0]);  % Black color for all lines
set(0, 'DefaultAxesLineStyleOrder', {'-', '--', ':', '-.', '-*'}); % Line styles

% 5.1 Drag Coefficient vs Mach Number
figure('Name', 'Drag Coefficient Analysis', ...
       'Position', [100, 100, 800, 600], ...
       'NumberTitle', 'off');
hold on; grid on; box on;

% Plot each angle of attack as separate line
for i = 1:length(angleOfAttackRange)
    plot(velocityRange/speedOfSound, dragCoefficientMatrix(:,i), ...
         'DisplayName', sprintf('\\alpha = %.1f°', rad2deg(angleOfAttackRange(i))), ...
         'LineWidth', 2);
end

% Format the drag coefficient plot
title('Body-Fin Drag Coefficient vs Mach Number', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Mach Number [-]', 'FontSize', 12);
ylabel('Drag Coefficient C_d [-]', 'FontSize', 12);
legend('Location', 'best', 'FontSize', 10);
set(gca, 'FontSize', 12, 'GridLineStyle', '--', 'GridAlpha', 0.3);
xlim([0, max(velocityRange/speedOfSound)]);
ylim([0, max(dragCoefficientMatrix(:)) * 1.1]); % Add 10% margin

% Add reference information
text(0.05, 0.95, sprintf('Rocket: %s', rocketFile(1:end-4)), ...
     'Units', 'normalized', 'FontSize', 10, 'BackgroundColor', 'white');

fprintf('  Plot 1: Drag coefficient generated.\n');

% 5.2 Normal Force Coefficient vs Angle of Attack
figure('Name', 'Normal Force Analysis', ...
       'Position', [950, 100, 800, 600], ...
       'NumberTitle', 'off');
hold on; grid on; box on;

% Calculate and plot normal force coefficient (normalForceCoefficient = CNa * alpha)
normalForceCoefficient = CNa .* angleOfAttackRange; % Normal force coefficient at each alpha
plot(rad2deg(angleOfAttackRange), normalForceCoefficient, ...
     'LineWidth', 2, 'Color', 'b', 'Marker', 'o', 'MarkerSize', 6);

% Format the normal force plot
title('Normal Force Coefficient vs Angle of Attack', ...
      'FontSize', 14, 'FontWeight', 'bold');
xlabel('Angle of Attack \alpha [°]', 'FontSize', 12);
ylabel('Normal Force Coefficient C_N [-]', 'FontSize', 12);
set(gca, 'FontSize', 12, 'GridLineStyle', '--', 'GridAlpha', 0.3);
xlim([0, rad2deg(max(angleOfAttackRange))]);
ylim([0, max(normalForceCoefficient) * 1.1]);

% Add reference information
text(0.05, 0.95, sprintf('Maximum C_N = %.3f', max(normalForceCoefficient)), ...
     'Units', 'normalized', 'FontSize', 10, 'BackgroundColor', 'white');

fprintf('  Plot 2: Normal force coefficient generated.\n');

% 5.3 Stability Margin vs Angle of Attack
figure('Name', 'Stability Analysis', ...
       'Position', [1800, 100, 800, 600], ...
       'NumberTitle', 'off');
hold on; grid on; box on;

% Calculate stability margin (calibers)
stabilityMargin = (XCP - Rocket.rocket_cm) / Rocket.dm;

plot(rad2deg(angleOfAttackRange), stabilityMargin, ...
     'LineWidth', 2, 'Color', 'r', 'Marker', 's', 'MarkerSize', 6);

% Format the stability margin plot
title('Stability Margin vs Angle of Attack', ...
      'FontSize', 14, 'FontWeight', 'bold');
xlabel('Angle of Attack \alpha [°]', 'FontSize', 12);
ylabel('Stability Margin (X_{cp} - X_{cm})/d_m [calibers]', 'FontSize', 12);
set(gca, 'FontSize', 12, 'GridLineStyle', '--', 'GridAlpha', 0.3);
xlim([0, rad2deg(max(angleOfAttackRange))]);

% Add stability guidelines
% 1 caliber = minimum stability (marginal stability)
% 2 calibers = typical minimum for stable flight
yline(1, '--k', '1 caliber (marginal)', 'LineWidth', 1, 'FontSize', 9);
yline(2, '--k', '2 calibers (minimum)', 'LineWidth', 1, 'FontSize', 9);

% Indicate stable region
yl = ylim;
fill([0 rad2deg(max(angleOfAttackRange)) rad2deg(max(angleOfAttackRange)) 0], ...
     [2 2 yl(2) yl(2)], 'g', 'FaceAlpha', 0.1, 'EdgeColor', 'none');

% Add reference information
text(0.05, 0.95, sprintf('Mean stability: %.2f calibers', mean(stabilityMargin)), ...
     'Units', 'normalized', 'FontSize', 10, 'BackgroundColor', 'white');

fprintf('  Plot 3: Stability margin generated.\n');

% -------------------------------------------------------------------------
% 6.0 DATA EXPORT OPTION (Optional Enhancement)
% -------------------------------------------------------------------------
% Uncomment to enable data export
% exportData = false;
% if exportData
%     % Save workspace variables
%     save('AeroPlot_Results.mat', 'Rocket', 'velocityRange', 'angleOfAttackRange', ...
%          'dragCoefficientMatrix', 'CNa', 'XCP', 'stabilityMargin');
%     
%     % Export to CSV for external analysis
%     resultsTable = table(angleOfAttackRange', CNa', XCP', stabilityMargin', ...
%                          'VariableNames', {'Alpha_rad', 'CNa', 'XCP', 'Stability_Calibers'});
%     writetable(resultsTable, 'AeroPlot_Results.csv');
%     fprintf('\nResults exported to AeroPlot_Results.mat and AeroPlot_Results.csv\n');
% end

% -------------------------------------------------------------------------
% 7.0 SUMMARY OUTPUT
% -------------------------------------------------------------------------
fprintf('\n=== AEROPLOT ANALYSIS COMPLETE ===\n');
fprintf('Rocket Configuration: %s\n', rocketFile);
fprintf('Analysis Range:\n');
fprintf('  Velocity: %.0f to %.0f m/s (%d points)\n', ...
        min(velocityRange), max(velocityRange), length(velocityRange));
fprintf('  Angle of Attack: 0 to %.1f° (%d points)\n', ...
        rad2deg(max(angleOfAttackRange)), length(angleOfAttackRange));
fprintf('\nKey Results:\n');
fprintf('  Minimum Drag Coefficient: %.4f\n', min(dragCoefficientMatrix(:)));
fprintf('  Maximum Drag Coefficient: %.4f\n', max(dragCoefficientMatrix(:)));
fprintf('  Normal Force Slope at α=0: %.3f /rad\n', CNa(1));
fprintf('  Stability Margin Range: %.2f to %.2f calibers\n', ...
        min(stabilityMargin), max(stabilityMargin));

% Check stability criteria
minStability = min(stabilityMargin);
if minStability < 1
    fprintf('\n⚠️  WARNING: Rocket is UNSTABLE at some angles of attack!\n');
    fprintf('   Minimum stability: %.2f calibers (should be > 1)\n', minStability);
elseif minStability < 2
    fprintf('\n⚠️  CAUTION: Marginal stability detected.\n');
    fprintf('   Minimum stability: %.2f calibers (recommended > 2)\n', minStability);
else
    fprintf('\n✓ Rocket appears STABLE across all analyzed angles.\n');
end

fprintf('\nPlots have been generated in separate windows.\n');