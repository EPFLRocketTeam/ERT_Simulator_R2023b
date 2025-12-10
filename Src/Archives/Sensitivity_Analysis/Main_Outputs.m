 %% Main_Outputs.m - Frédéric Berdoz - October 2020 
%
% Main runabale script for visualizing the outpus
% 

% Initialize
clc; clear all; close all;
addpath(genpath('../Declarations'),...
        genpath('../Functions'),...
        genpath('../Snippets'),...
        genpath('../Simulator_3D'),...
        genpath('./Helpers'));
    
% ODE integrator warnings
warning('off');

%% Loading the paramters
% Rocket Definition
Rocket = rocketReader('BL2_H3.txt');
Environment = environnementReader('Environment/Environnement_Definition_SA.txt');
simulationOutputs = SimOutputReader('Simulation/Simulation_outputs.txt');

simulatior3D = multilayerwindSimulator3D(Rocket, Environment, simulationOutputs);

%% Simulation

[railTime, railState] = simulatior3D.RailSim();

[burnTime, burnState, burnTimeEvents, burnStateEvents, burnEventIndices] = simulatior3D.FlightSim([railTime(end) simulatior3D.Rocket.Burn_Time(end)], railState(end, 2));

[coastTime, coastState, coastTimeEvents, coastStateEvents, coastEventIndices] = simulatior3D.FlightSim([burnTime(end) 40], burnState(end, 1:3)', burnState(end, 4:6)', burnState(end, 7:10)', burnState(end, 11:13)');

flightTime = [burnTime; coastTime(2:end)];
flightState = [burnState; coastState(2:end, :)];

combinedRailFlightTime = [railTime;flightTime];
combinedRailFlightState = [railState;flightState(:,3) flightState(:,6)];

[drogueTime, drogueState, drogueTimeEvents, drogueStateEvents, drogueEventIndices] = simulatior3D.DrogueParaSim(flightTime(end), flightState(end,1:3)', flightState(end, 4:6)');

[mainChuteTime, mainChuteState, mainChuteTimeEvents, S4E, mainChuteEventsIndices] = simulatior3D.MainParaSim(drogueTime(end), drogueState(end,1:3)', drogueState(end, 4:6)');

[crashTime, crashState, crashTimeEvents, crashStateEvents, crashEventIndices] = simulatior3D.CrashSim(flightTime(end), flightState(end,1:3)', flightState(end, 4:6)');

%% Margin plot
figure('Name','Static Margin'); hold on;
title 'Static Margin';
yyaxis left;
plot(flightTime, simulatior3D.simAuxResults.CM, 'DisplayName', 'X_{CM}');
plot(flightTime, simulatior3D.simAuxResults.Xcp, 'DisplayName', 'X_{CP}');
ylabel 'X_{CM}, X_{CP} [cm]'
yyaxis right;
plot(flightTime, simulatior3D.simAuxResults.Margin, 'DisplayName', 'Margin');
xlabel 'Time [s]';
ylabel 'Margin [-]';
legend show;

%% Margin * CNa plot and apogee
figure('Name','Simulator Outputs'); 

subplot(1,2,1)
hold on;
title 'Stability Margin'
xline(flightTime(1), '-', {'End of Rail'}, 'LabelVerticalAlignment', 'middle', 'LabelHorizontalAlignment', 'center', 'Color', 'green', 'LineWidth', 1.2, 'DisplayName', 'End of Rail');
xline(simulatior3D.Rocket.Burn_Time,  '-', {'End of Propulsion'}, 'LabelVerticalAlignment', 'middle', 'LabelHorizontalAlignment', 'center', 'Color', 'red', 'LineWidth', 1.2, 'DisplayName', 'End of Propulsion');
xline(flightTime(end),  '-', {'Apogee'}, 'LabelVerticalAlignment', 'middle', 'LabelHorizontalAlignment', 'center', 'Color', 'magenta', 'LineWidth', 1.2, 'DisplayName', 'Apogee');
legend show;

yyaxis left
plot(flightTime,simulatior3D.simAuxResults.Cn_alpha.*simulatior3D.simAuxResults.Margin, 'LineWidth', 1.2, 'DisplayName', 'Stability')
xlabel 'Time [s]';
ylabel 'MS{\times}C_{N{\alpha}} [-]';
grid on
ylim([0 50]);

yyaxis right
ylabel 'V [m/s]';
plot(flightTime,sqrt(flightState(:,4).^2 + flightState(:,5).^2 + flightState(:,6).^2) , 'LineWidth', 1.2, 'DisplayName', 'Velocity')

% Altitude vs. drift
subplot(1,2,2)
title 'Altitude vs Drift'; 
hold on; grid on
plot(sqrt(flightState(:,1).^2 + flightState(:,2).^2), flightState(:,3), 'DisplayName', 'Ascent', 'LineWidth', 1.2);
plot(sqrt(drogueState(:,1).^2 + drogueState(:,2).^2), drogueState(:,3), 'DisplayName', 'Drogue', 'LineWidth', 1.2);
plot(sqrt(mainChuteState(:,1).^2 + mainChuteState(:,2).^2), mainChuteState(:,3), 'DisplayName', 'Main', 'LineWidth', 1.2);
plot(sqrt(crashState(:,1).^2 + crashState(:,2).^2), crashState(:,3), 'd', 'DisplayName', 'CrashSim', 'LineWidth', 1.2);
xlabel 'Drift [m]'; ylabel 'Altitude [m]';

ymax = ceil(flightState(end,3)/1000)*1000;
ylim([0 ymax]);
xmax = ceil(sqrt(crashState(end,1).^2 + crashState(end,2).^2)/100)*100;
xlim([0,xmax]);

legend show;

%% Wind Model
alt = [10 100 200 500 1000 2000];
Vspeed = 3 * ones(1,6);
Vturb = 0.2 * ones(1,6);
Vazy = 150 * ones(1,6);

turb_std = Vturb .* Vspeed;
axis = 0:10:4000;

% Baseline plot
figure('Name', 'Wind Model')
title 'Multi-layered Wind Model';
xlabel 'Velocity [m/s]';
ylabel 'Altitude [m]';
hold on;
grid on;

xl = [-10 15];
xlim(xl);
yl = [0 4000];
ylim(yl);
plt1 = plot(simulatior3D.Environment.Vspeed, axis, 'DisplayName', 'Wind Model', 'LineWidth', 1.2);
plt2 = plot(Vspeed, alt, 'd', 'DisplayName', 'Data Points', 'LineWidth', 1.5);
yline(flightState(end,3), '--', 'LineWidth', 1.2, 'DisplayName', 'Apogee');

for i = 1:10
    
    Vspeed_i_dp = normrnd(Vspeed, turb_std);
    Vspeed_i = interp1(alt, Vspeed_i_dp, axis, 'pchip', 'extrap');
    Vazy_i = interp1(alt, Vazy, axis, 'pchip', 'extrap');
    
    plot(Vspeed_i, axis, 'LineWidth', 0.8, 'Color', [0.8 0.8 0.8]);
    plot(Vspeed_i_dp, alt, 'x', 'DisplayName', 'Data Points', 'LineWidth', 1.2, 'MarkerSize', 4, 'Color', [0.8 0.8 0.8]);
end

xBox = [xl(1), xl(1), 0, 0, xl(1)]
yBox = [yl(1), yl(2), yl(2), yl(1), yl(1)]
patch(xBox, yBox,'black', 'EdgeColor','none', 'FaceColor', 'red', 'FaceAlpha', 0.1);
legend('Deterministic Model', 'Data Points', 'Apogee', 'Random Models');
uistack(plt1,'top')
uistack(plt2,'top')