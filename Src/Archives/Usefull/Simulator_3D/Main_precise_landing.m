%% Rocket Simulator 3D

% Initialize
close all; clear all;
addpath(genpath('../Declarations'),...
        genpath('../Functions'),...
        genpath('../Snippets'),...
        genpath('../Simulator_1D'));

% Rocket Definition
Rocket = rocketReader('WASSERFALLEN_FRANKENSTEIN.txt');
Environment = environnementReader('Environment/Environnement_Definition_Wasserfallen.txt');
simulationOutputs = SimOutputReader('Simulation/Simulation_outputs.txt');

simulatior3D = multilayerwindSimulator3D(Rocket, Environment, simulationOutputs);

%% ------------------------------------------------------------------------
% 6DOF Rail Simulation
%--------------------------------------------------------------------------

[railTime, railState] = simulatior3D.RailSim();

display(['Launch rail departure velocity : ' num2str(railState(end,2))]);
display(['Launch rail departure time : ' num2str(railTime(end))]);

%% ------------------------------------------------------------------------
% 6DOF Flight Simulation
%--------------------------------------------------------------------------

[burnTime, burnState, burnTimeEvents, burnStateEvents, burnEventIndices] = simulatior3D.FlightSim([railTime(end) simulatior3D.Rocket.Burn_Time(end)], railState(end, 2));

%simulatior3D.Rocket.coneMode = 'off';

[coastTime, coastState, coastTimeEvents, coastStateEvents, coastEventIndices] = simulatior3D.FlightSim([burnTime(end) 40], burnState(end, 1:3)', burnState(end, 4:6)', burnState(end, 7:10)', burnState(end, 11:13)');

flightTime = [burnTime; coastTime(2:end)];
flightState = [burnState; coastState(2:end, :)];

combinedRailFlightTime = [railTime;flightTime];
combinedRailFlightState = [railState;flightState(:,3) flightState(:,6)];

display(['Apogee AGL : ' num2str(flightState(end,3))]);
display(['Apogee AGL @t = ' num2str(flightTime(end))]);
[maxi,index] = max(flightState(:,6));
display(['Max speed : ' num2str(maxi)]);
display(['Max speed @t = ' num2str(flightTime(index))]);
[~,a,~,density,nu] = stdAtmos(flightState(index,3),Environment);
Fd = 0.5*simulatior3D.simAuxResults.Cd(index)*density*pi*Rocket.maxDiameter^2/4*maxi^2;
display(['Max drag force = ' num2str(Fd)]);
display(['Max drag force along rocket axis = ' num2str(Fd*cos(simulatior3D.simAuxResults.Delta(index)))]);
C_Dab = drag_shuriken(Rocket, 0, simulatior3D.simAuxResults.Delta(index), maxi, nu);
F_Dab = 0.5*C_Dab*density*pi*Rocket.maxDiameter^2/4*maxi^2;
display(['AB drag force at max speed = ' num2str(F_Dab)]);
display(['Max Mach number : ' num2str(maxi/a)]);
[maxi,index] = max(diff(combinedRailFlightState(:,2))./diff(combinedRailFlightTime));
display(['Max acceleration : ' num2str(maxi)]);
display(['Max g : ' num2str(maxi/9.81)]);
display(['Max g @t = ' num2str(combinedRailFlightTime(index))]);

%% ------------------------------------------------------------------------
% 3DOF Recovery Drogue
%--------------------------------------------------------------------------

[T3, S3, drogueTimeEvents, drogueStateEvents, drogueEventIndices] = simulatior3D.DrogueParaSim(flightTime(end), flightState(end,1:3)', flightState(end, 4:6)');

%% ------------------------------------------------------------------------
% 3DOF Recovery Main
%--------------------------------------------------------------------------

[T4, mainChuteState, mainChuteTimeEvents, S4E, mainChuteEventsIndices] = simulatior3D.MainParaSim(T3(end), S3(end,1:3)', S3(end, 4:6)');

display(['Touchdown @t = ' num2str(T4(end)) ' = ' num2str(floor(T4(end)/60)) ' min ' num2str(mod(T4(end),60)) ' s']);

%% ------------------------------------------------------------------------
% 3DOF Crash Simulation
%--------------------------------------------------------------------------

[crashTime, crashState, crashTimeEvents, crashStateEvents, crashEventIndices] = simulatior3D.CrashSim(flightTime(end), flightState(end,1:3)', flightState(end, 4:6)');




%--------------------------------------------------------------------------

plotShowAnswer = input('Show plots ? [Y/N]\n','s');
if ~strcmp(plotShowAnswer,{'Y','y','Yes','yes'})
    return
end

%% ------------------------------------------------------------------------
% Plots
%--------------------------------------------------------------------------

% PLOT 1 : 3D rocket trajectory

C = quat2rotmat(flightState(:, 7:10));
angle = rot2anglemat(C);

% plot rocket orientation
figure('Name','3D Trajectory Representation'); hold on;
directionVectors = zeros(length(C),3);
for i  = 1:length(C)
    directionVectors(i,:) = C(:,:,i)*[0;0;1];
end
%quiver3(flightState(:,1), flightState(:,2), flightState(:,3), directionVectors(:,1), directionVectors(:,2), directionVectors(:,3));

% plot trajectory of CM
plot3(flightState(:,1), flightState(:,2), flightState(:,3), 'DisplayName', 'Ascent','LineWidth',2);
plot3(S3(:,1), S3(:,2), S3(:,3), 'DisplayName', 'Drogue Descent','LineWidth',2);
plot3(mainChuteState(:,1), mainChuteState(:,2), mainChuteState(:,3), 'DisplayName', 'Main Descent','LineWidth',2);
plot3(crashState(:,1), crashState(:,2), crashState(:,3), 'DisplayName', 'Ballistic Descent','LineWidth',2)
daspect([1 1 1]); pbaspect([1, 1, 1]); view(45, 45);
[XX, YY, M, Mcolor] = get_google_map(Environment.startLatitude, Environment.startLongitude, 'Height', ceil(diff(xlim)/3.4), 'Width', ceil(diff(ylim)/3.4));
xImage = [xlim',xlim'];
yImage = [ylim;ylim];
zImage = zeros(2);
colormap(Mcolor);
surf(xImage, yImage, zImage, 'CData', M,'FaceColor', 'texturemap', 'EdgeColor', 'none', 'DisplayName', 'Base Map');
title '3D trajectory representation'
xlabel 'S [m]'; ylabel 'E [m]'; zlabel 'Altitude [m]';
legend show;

