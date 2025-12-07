%% Rocket Simulator 3D

% Initialize
close all; clear all; clc;
addpath(genpath('./Declarations'),...
        genpath('./Functions'),...
        genpath('./Snippets'),...
        genpath('./Simulator_3D'));

% Rocket Definition
rocket = rocketReader('Nordend_EUROC.txt');
environment = environnementReader('Environment/Environnement_Definition_EuRoC.txt');
dragFile = 'Drag/dragOK_N1332.csv';
interpType = 'time';
simOutputs = SimOutputReader('Simulation/Simulation_outputs.txt');
simObj = Simulator3D_DragOR(rocket, environment, dragFile, interpType, simOutputs);

%% ------------------------------------------------------------------------
% 6DOF Rail Simulation
%--------------------------------------------------------------------------

[railTime, railState] = simObj.RailSim();

display(['Launch rail departure velocity : ' num2str(railState(end,2))]);
display(['Launch rail departure time : ' num2str(railTime(end))]);

%% ------------------------------------------------------------------------
% 6DOF Flight Simulation
%--------------------------------------------------------------------------

[burnTime, burnState, burnTimeE, burnStateE, burnIE] = simObj.FlightSim([railTime(end) simObj.Rocket.Burn_Time(end)], railState(end, 2));

%simObj.Rocket.coneMode = 'off';

[coastTime, coastState, coastTimeE, coastStateE, coastIE] = simObj.FlightSim([burnTime(end) 40], burnState(end, 1:3)', burnState(end, 4:6)', burnState(end, 7:10)', burnState(end, 11:13)');

flightTime = [burnTime; coastTime(2:end)];
flightState = [burnState; coastState(2:end, :)];

% flightState_dot is [X_dot;V_dot;Q_dot;W_dot]

totalTime = [railTime; flightTime];
combinedState = [railState; flightState(:,3) flightState(:,6)];

% flightState [x,y,z, vx,vy,vz, Q1, Q2, Q3, Q4, W1, W2, W3] with Q quaternions in rocket frame,
% W angle of attack in earth frame

display(['Apogee AGL : ' num2str(flightState(end,3))]);
display(['Apogee AGL @t = ' num2str(flightTime(end))]);
[maxSpeed, speedIndex] = max(flightState(:,6));
display(['Max speed : ' num2str(maxSpeed)]);
display(['Max speed @t = ' num2str(flightTime(speedIndex))]);
[~,soundSpeed,~,density,viscosity] = atmosphere(flightState(speedIndex,3),environment);
dragForce = 0.5*simObj.SimAuxResults.Cd(speedIndex)*density*pi*rocket.maxDiameter^2/4*maxSpeed^2;
display(['Max drag force = ' num2str(dragForce)]);
display(['Max drag force along rocket axis = ' num2str(dragForce*cos(simObj.SimAuxResults.Delta(speedIndex)))]);
cdAB = drag_shuriken(rocket, 0, simObj.SimAuxResults.Delta(speedIndex), maxSpeed, viscosity);
dragForceAB = 0.5*cdAB*density*pi*rocket.maxDiameter^2/4*maxSpeed^2;
display(['AB drag force at max speed = ' num2str(dragForceAB)]);
display(['Max Mach number : ' num2str(maxSpeed/soundSpeed)]);
[maxAccel, accelIndex] = max(diff(combinedState(:,2))./diff(totalTime));
display(['Max acceleration : ' num2str(maxAccel)]);
display(['Max g : ' num2str(maxAccel/9.81)]);
display(['Max g @t = ' num2str(totalTime(accelIndex))]);

figure(Name="Euler angles")
quaternion = flightState(:,7:10)';
[phi, theta, psi] = quat_to_euler_angles(quaternion(1,:), quaternion(2,:), quaternion(3,:), quaternion(4,:));
hold on
plot(flightTime, phi .* 180 ./ pi, LineWidth=2)
plot(flightTime, theta .* 180 ./ pi, LineWidth=2)
plot(flightTime, psi .* 180 ./ pi, LineWidth=2)
grid on
box on
xlabel("t [s]")
ylabel("Angles")
legend("\phi", "\theta", "\psi", fontsize=15)

%% ------------------------------------------------------------------------
% 3DOF Recovery Drogue
%--------------------------------------------------------------------------

[drogueTime, drogueState, drogueTimeE, drogueStateE, drogueIE] = simObj.DrogueParaSim(flightTime(end), flightState(end,1:3)', flightState(end, 4:6)');

%% ------------------------------------------------------------------------
% 3DOF Recovery Main
%--------------------------------------------------------------------------

[mainTime, mainState, mainTimeE, mainStateE, mainIE] = simObj.MainParaSim(drogueTime(end), drogueState(end,1:3)', drogueState(end, 4:6)');

display(['Touchdown @t = ' num2str(mainTime(end)) ' = ' num2str(floor(mainTime(end)/60)) ' min ' num2str(mod(mainTime(end),60)) ' s']);

%% ------------------------------------------------------------------------
% 3DOF Crash Simulation
%--------------------------------------------------------------------------

[crashTime, crashState, crashTimeE, crashStateE, crashIE] = simObj.CrashSim(flightTime(end), flightState(end,1:3)', flightState(end, 4:6)');

%% ------------------------------------------------------------------------
% 6DOF Crash Simulation for the nosecone
%--------------------------------------------------------------------------

% % There is currently an error with the integration
% 
% nosecone = rocketReader('Rocket_Definition_Eiger_I_Final_Nosecone.txt');
% 
% % SimObj2 = Simulator3D(nosecone, environment, simOutputs);
% simObj.Rocket = nosecone;
% 
% [noseconeTime, noseconeState, noseconeTimeE, noseconeStateE, noseconeIE] = simObj.Nose_CrashSim_6DOF([flightTime(end) 40], flightState(end, 1:3)', flightState(end, 4:6)', flightState(end, 7:10)', flightState(end, 11:13)');

%% ------------------------------------------------------------------------
% Payload Crash Simulation
%--------------------------------------------------------------------------

%[payloadTime, payloadState, payloadTimeE, payloadStateE, payloadIE] = simObj.CrashSim(flightTime(end), flightState(end,1:3)', flightState(end, 4:6)');

%% ------------------------------------------------------------------------
% Analyse results ?
%--------------------------------------------------------------------------

plotShowAnswer = input('Show plots ? [Y/N]\n','s');
if ~strcmp(plotShowAnswer,{'Y','y','Yes','yes'})
    return
end

%% ------------------------------------------------------------------------
% Plots
%--------------------------------------------------------------------------

% PLOT 1 : 3D rocket trajectory

rotationMatrices = quat2rotmat(flightState(:, 7:10));
angles = rot2anglemat(rotationMatrices);

% plot rocket orientation
figure('Name','3D Trajectory Representation'); hold on;
directionVectors = zeros(length(rotationMatrices),3);
for i  = 1:length(rotationMatrices)
    directionVectors(i,:) = rotationMatrices(:,:,i)*[0;0;1];
end

% plot trajectory of CM
plot3(flightState(:,1), flightState(:,2), flightState(:,3), 'DisplayName', 'Ascent','LineWidth',2);
plot3(drogueState(:,1), drogueState(:,2), drogueState(:,3), 'DisplayName', 'Drogue Descent','LineWidth',2);
plot3(mainState(:,1), mainState(:,2), mainState(:,3), 'DisplayName', 'Main Descent','LineWidth',2);
plot3(crashState(:,1), crashState(:,2), crashState(:,3), 'DisplayName', 'Ballistic Descent','LineWidth',2)
daspect([1 1 1]); pbaspect([1, 1, 1]); view(45, 45);

xImage = [xlim',xlim'];
yImage = [ylim;ylim];
zImage = zeros(2);
colormap('jet');
surf(environment.map_x, environment.map_y, environment.map_z, 'EdgeColor', 'none', 'DisplayName', 'Base Map');
title '3D trajectory representation'
xlabel 'S [m]'; ylabel 'E [m]'; zlabel 'Altitude [m]';
legend show;

% PLOT 2 : time dependent altitude
figure('Name','Time dependent altitude'); hold on;
plot(flightTime, flightState(:,3), 'DisplayName', 'Ascent');
plot(drogueTime, drogueState(:,3), 'DisplayName', 'Drogue Descent');
plot(mainTime, mainState(:,3), 'DisplayName', 'Main Descent');
plot(crashTime, crashState(:,3), 'DisplayName', 'Ballistic Descent');
title 'Altitude vs. time'
xlabel 't [s]'; ylabel 'Altitude [m]';
legend show;

% PLOT 3 : Altitude vs. drift
figure('Name','Altitude vs Drift')'; hold on;
plot(sqrt(drogueState(:,1).^2 + drogueState(:,2).^2), drogueState(:,3), 'DisplayName', 'Drogue');
plot(sqrt(mainState(:,1).^2 + mainState(:,2).^2), mainState(:,3), 'DisplayName', 'Main');
plot(sqrt(crashState(:,1).^2 + crashState(:,2).^2), crashState(:,3), 'd', 'DisplayName', 'CrashSim');
title 'Altitude vs. drift'
xlabel 'Drift [m]'; ylabel 'Altitude [m]';
legend show;

% PLOT 4 : Aerodynamic properties
figure('Name','Aerodynamic properties'); hold on;
% Plot Margin
subplot(3,2,1);
plot(flightTime, simObj.SimAuxResults.Margin)
hold on;
plot(ones(1,2)*rocket.Burn_Time, ylim, 'g');
title 'Margin';
% Plot Xcp
subplot(3,2,2);
plot(flightTime, simObj.SimAuxResults.Xcp)
hold on;
plot(ones(1,2)*rocket.Burn_Time, ylim, 'g');
title 'X_{cp}';
% Plot AoA vs. time
subplot(3,2,3);
plot(flightTime, simObj.SimAuxResults.Alpha)
hold on;
plot(ones(1,2)*rocket.Burn_Time, ylim, 'g');
title '\alpha';
% Plot CNa vs. speed
subplot(3,2,4);
plot(flightTime, simObj.SimAuxResults.Cn_alpha)
hold on;
plot(ones(1,2)*rocket.Burn_Time, ylim, 'g');
title 'Cn_{\alpha}';

subplot(3,2,5);
plot(flightTime, simObj.SimAuxResults.Cd*1.3) % 1.3 is scale corrective CD factor!
hold on;
title 'SCALED CD';

% Plot angle with vertical
subplot(3,2,6);
plot(flightTime, simObj.SimAuxResults.Delta)
ylim([0, 1]);
currentYlim = ylim;
set(gca, 'YTick', currentYlim(1):0.1:currentYlim(2));
hold on;
plot(ones(1,2)*rocket.Burn_Time, ylim, 'g');
title 'Delta, angle with Oz'
screenSize = get(groot, 'Screensize');
set(gcf,'Position',[screenSize(1:2), screenSize(3)*0.5, screenSize(4)]);

% PLOT 5 : Mass properties
figure('Name','Mass properties'); hold on;
% Plot mass vs. time
subplot(2,2,1);
plot(flightTime, simObj.SimAuxResults.Mass)
hold on;
plot(ones(1,2)*rocket.Burn_Time, ylim, 'g');
currentYlim = ylim;
title 'Mass';
set(gca, 'YTick', currentYlim(1):0.5:currentYlim(2));
hold on;
plot(ones(1,2)*rocket.Burn_Time, ylim, 'g');
% Plot CM vs. time
subplot(2,2,2);
plot(flightTime, simObj.SimAuxResults.CM)
currentYlim = ylim;
title 'CM';
set(gca, 'YTick', currentYlim(1):0.01:currentYlim(2));
hold on;
plot(ones(1,2)*rocket.Burn_Time, ylim, 'g');
% Plot Il vs. time
subplot(2,2,3);
plot(flightTime, simObj.SimAuxResults.Il)
currentYlim = ylim;
title 'Il';
set(gca, 'YTick', currentYlim(1):0.1:currentYlim(2));
hold on;
plot(ones(1,2)*rocket.Burn_Time, ylim, 'g');
%Plot Ir vs. time
subplot(2,2,4);
plot(flightTime, simObj.SimAuxResults.Ir)
title 'Ir';
hold on;
plot(ones(1,2)*rocket.Burn_Time, ylim, 'g');
screenSize = get(groot, 'Screensize');
set(gcf,'Position',[screenSize(3)*0.5, screenSize(2),...
    screenSize(3)*0.5, screenSize(3)*0.5]);

% PLOT 6 : Margin plot
figure('Name','Dynamic stability margin'); hold on;
title 'Stability margin'
yyaxis left;
plot(flightTime, simObj.SimAuxResults.CM, 'DisplayName', 'X_{CM}');
plot(flightTime, simObj.SimAuxResults.Xcp, 'DisplayName', 'X_{CP}');
ylabel 'X_{CM}, X_{CP} [cm]'
yyaxis right;
plot(flightTime, simObj.SimAuxResults.Margin, 'DisplayName', 'Margin');
ylabel 'Margin [calibers]';
title 'Dynamic Stability Margin'
legend show;

% plot 7 : norm of quaternion
figure('Name','Norm of quaternion'); hold on;
plot(flightTime, sqrt(sum(flightState(:, 7:10).^2, 2)));