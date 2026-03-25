%% Initialize

close all; clear all;
addpath(genpath('../Declarations'),...
        genpath('../Functions'),...
        genpath('../Snippets'),...
        genpath('../Simulator_1D'));

% Rocket Definition
Rocket = rocketReader('Rocket_Definition_Final.txt');
Environment = environnementReader('Environnement_Definition.txt');
simulationOutputs = SimOutputReader('Simulation_outputs.txt');

%% Simulate and plot trajectories

h1 = figure(1); ax1 = gca; hold(ax1, 'on');
h2 = figure(2); ax2 = gca; hold(ax2, 'on');

X_Para = [];
X_NoPara = [];
V = -6:2:6;

for v = V

    Environment.V_inf = v;
    
    simulatior3D = Simulator3D(Rocket, Environment, simulationOutputs);
    
    %% ------------------------------------------------------------------------
    % 6DOF Rail Simulation
    %--------------------------------------------------------------------------

    [railTime, railState] = simulatior3D.RailSim();

    %% ------------------------------------------------------------------------
    % 6DOF Flight Simulation
    %--------------------------------------------------------------------------

    [flightTime, flightState] = simulatior3D.FlightSim(railTime(end), railState(end,2));

    %% ------------------------------------------------------------------------
    % 3DOF Recovery Drogue
    %--------------------------------------------------------------------------

    [drogueTime, drogueState] = simulatior3D.DrogueParaSim(flightTime(end), flightState(end,1:3)', flightState(end, 4:6)');

    %% ------------------------------------------------------------------------
    % 3DOF Recovery Main
    %--------------------------------------------------------------------------

    [mainChuteTime, mainChuteState] = simulatior3D.MainParaSim(drogueTime(end), drogueState(end,1:3)', drogueState(end, 4:6)');
    
    %% ------------------------------------------------------------------------
    % 3DOF Crash Simulation
    %--------------------------------------------------------------------------

    [crashTime, crashState, crashTimeEvents, crashStateEvents, crashEventIndices] = simulatior3D.CrashSim(flightTime(end), flightState(end,1:3)', flightState(end, 4:6)');

    plot3(ax1, [flightState(:,1);drogueState(:,1);mainChuteState(:,1)], [flightState(:,2);drogueState(:,2);mainChuteState(:,2)], [flightState(:,3);drogueState(:,3);mainChuteState(:,3)], 'DisplayName', ['Trajectoire, v = ' num2str(V)]);
    plot3(ax2, [flightState(:,1);crashState(:,1)], [flightState(:,2);crashState(:,2)], [flightState(:,3);crashState(:,3)], 'DisplayName', ['Trajectoire, v = ' num2str(V)]);
    
    X_Para = [X_Para, mainChuteState(end,1)];
    X_NoPara = [X_NoPara, crashState(end,1)];
end

%% Plot map
axes(ax1);
daspect([1 1 1]); pbaspect([0.5, 0.5, 1]); view(30, 50);
% [XX, YY, M, Mcolor] = get_google_map(Environment.startLatitude, Environment.startLongitude, 'Height', ceil(diff(xlim)/3.4), 'Width', ceil(diff(ylim)/3.4));
xImage = [xlim',xlim'];
yImage = [ylim;ylim];
zImage = zeros(2);
% colormap(Mcolor);
% surf(xImage, yImage, zImage, 'CData', M,'FaceColor', 'texturemap', 'EdgeColor', 'none', 'DisplayName', 'Base Map');
% title('Sensibilité au vent, parachute déployé');
% xlabel('S [m]'); ylabel 'E [m]'; zlabel 'Altitude [m]';
% legend show;
% set(ax1, 'Fontsize', 14)

axes(ax2);
daspect([1 1 1]); pbaspect([0.5, 0.5, 1]); view(30, 50);
% [XX, YY, M, Mcolor] = get_google_map(Environment.startLatitude, Environment.startLongitude, 'Height', ceil(diff(xlim)/3.4), 'Width', ceil(diff(ylim)/3.4));
xImage = [xlim',xlim'];
yImage = [ylim;ylim];
zImage = zeros(2);
% colormap(Mcolor);
% surf(xImage, yImage, zImage, 'CData', M,'FaceColor', 'texturemap', 'EdgeColor', 'none', 'DisplayName', 'Base Map');
% title('Sensibilité au vent, parachute non-déployé');
% xlabel('S [m]'); ylabel 'E [m]'; zlabel 'Altitude [m]';
% legend show;
% set(ax2, 'Fontsize', 14);

figure(3); hold on;
plot(V, X_Para, 'DisplayName', 'Drift avec parachute');
plot(V, X_NoPara, 'DisplayName', 'Drift sans parachute');
title('Drift en fonction du vent')
xlabel('V [m/s]'); ylabel('Drift [m]');
l = legend('show'); set(l, 'Location', 'NorthWest');
set(gca, 'Fontsize', 14);
grid on;