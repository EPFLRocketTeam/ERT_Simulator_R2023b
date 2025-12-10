%% rocket Simulator 3D

% Initialize
close all; clear all; clc;
addpath(genpath('./Declarations'),...
        genpath('./Functions'),...
        genpath('./Snippets'),...
        genpath('./Simulator_3D'));

% Rocket Definition
rocket = rocketReader('Wildhorn.txt');
environment = environnementReader('Environment/Environnement_Definition_EuRoC.txt');
simulationOutputs = SimOutputReader('Simulation/Simulation_outputs.txt');
simulator3D = Simulator3D(rocket, environment, simulationOutputs);

%% ------------------------------------------------------------------------
% 6DOF Rail Simulation
%--------------------------------------------------------------------------

% Motor ignition
simulator3D.rocket.motorState = 'on';

[railTime, railState] = simulator3D.RailSim();

display(['Launch rail departure velocity : ' num2str(railState(end,2))]);
display(['Launch rail departure time : ' num2str(railTime(end))]);

%% ------------------------------------------------------------------------
% 6DOF Flight Simulation
%--------------------------------------------------------------------------

[burnTime, burnState, burnTimeEvents, burnStateEvents, burnEventIndices] = simulator3D.FlightSim([railTime(end) simulator3D.rocket.Burn_Time(end)], railState(end, 2));

% simulator3D.rocket.coneMode = 'off';

[coastTime, coastState, coastTimeEvents, coastStateEvents, coastEventIndices] = simulator3D.FlightSim([burnTime(end) 40], burnState(end, 1:3)', burnState(end, 4:6)', burnState(end, 7:10)', burnState(end, 11:13)');

flightTime = [burnTime; coastTime(2:end)];
flightState = [burnState; coastState(2:end, :)];

% flightState_dot is [X_dot;V_dot;Q_dot;W_dot]

combinedRailFlightTime = [railTime; flightTime];
combinedRailFlightState = [railState; flightState(:,3) flightState(:,6)];

% flightState: [x,y,z, vx,vy,vz, Q1, Q2, Q3, Q4, W1, W2, W3] 
% where Q are quaternions in rocket frame, W are angular velocities in earth frame

display(['Apogee AGL : ' num2str(flightState(end,3))]);
display(['Apogee AGL @t = ' num2str(flightTime(end))]);

[maxSpeed, maxSpeedIndex] = max(flightState(:,6));
display(['Max speed : ' num2str(maxSpeed)]);
display(['Max speed @t = ' num2str(flightTime(maxSpeedIndex))]);

[~, speedOfSound, ~, density, kinematicViscosity] = atmosphere(flightState(maxSpeedIndex,3), environment);
dragForce = 0.5 * simulator3D.simAuxResults.dragCoefficient(maxSpeedIndex) * density * pi * rocket.maxDiameter^2 / 4 * maxSpeed^2;
display(['Max drag force = ' num2str(dragForce)]);
display(['Max drag force along rocket axis = ' num2str(dragForce * cos(simulator3D.simAuxResults.flightPathAngle(maxSpeedIndex)))]);

dragCoefficientAb = drag_shuriken(rocket, 0, simulator3D.simAuxResults.flightPathAngle(maxSpeedIndex), maxSpeed, kinematicViscosity);
dragForceAb = 0.5 * dragCoefficientAb * density * pi * rocket.maxDiameter^2 / 4 * maxSpeed^2;
display(['AB drag force at max speed = ' num2str(dragForceAb)]);
display(['Max Mach number : ' num2str(maxSpeed / speedOfSound)]);

[maxAcceleration, maxAccelerationIndex] = max(diff(combinedRailFlightState(:,2)) ./ diff(combinedRailFlightTime));
display(['Max acceleration : ' num2str(maxAcceleration)]);
display(['Max g : ' num2str(maxAcceleration/9.81)]);
display(['Max g @t = ' num2str(combinedRailFlightTime(maxAccelerationIndex))]);

% Motor shutdown
simulator3D.rocket.motorState = 'off';

%% ------------------------------------------------------------------------
% 3DOF Recovery Drogue
%--------------------------------------------------------------------------

[drogueTime, drogueState, drogueTimeEvents, drogueStateEvents, drogueEventIndices] = simulator3D.DrogueParaSim(flightTime(end), flightState(end,1:3)', flightState(end, 4:6)');

%% ------------------------------------------------------------------------
% 3DOF Recovery Main
%--------------------------------------------------------------------------

[mainChuteTime, mainChuteState, mainChuteTimeEvents, mainChuteStateEvents, mainChuteEventIndices] = simulator3D.MainParaSim(drogueTime(end), drogueState(end,1:3)', drogueState(end, 4:6)');

touchdownSeconds = mainChuteTime(end);
touchdownMinutes = floor(touchdownSeconds / 60);
touchdownRemainingSeconds = mod(touchdownSeconds, 60);
disp(['Touchdown @t = ' num2str(touchdownSeconds) ' = ' num2str(touchdownMinutes) ' min ' num2str(touchdownRemainingSeconds) ' s']);

%% ------------------------------------------------------------------------
% 3DOF Crash Simulation
%--------------------------------------------------------------------------

[crashTime, crashState, crashTimeEvents, crashStateEvents, crashEventIndices] = simulator3D.CrashSim(flightTime(end), flightState(end,1:3)', flightState(end, 4:6)');

%% ------------------------------------------------------------------------
% 6DOF Crash Simulation for the nosecone
%--------------------------------------------------------------------------

% % There is currently an error with the integration
% 
% noseconeRocket = rocketReader('Rocket_Definition_Eiger_I_Final_Nosecone.txt');
% 
% % simulator3D2 = Simulator3D(noseconeRocket, environment, simulationOutputs);
% simulator3D.rocket = noseconeRocket;
% 
% [noseconeCrashTime, noseconeCrashState, noseconeCrashTimeEvents, noseconeCrashStateEvents, noseconeCrashEventIndices] = simulator3D.Nose_CrashSim_6DOF([flightTime(end) 40], flightState(end, 1:3)', flightState(end, 4:6)', flightState(end, 7:10)', flightState(end, 11:13)');

%% ------------------------------------------------------------------------
% Payload Crash Simulation
%--------------------------------------------------------------------------

%[payloadCrashTime, payloadCrashState, payloadCrashTimeEvents, payloadCrashStateEvents, payloadCrashEventIndices] = simulator3D.CrashSim(flightTime(end), flightState(end,1:3)', flightState(end, 4:6)');

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

% Convert quaternions to rotation matrices
rotationMatrices = quat2rotmat(flightState(:, 7:10));
eulerAngles = rot2anglemat(rotationMatrices);

% Plot rocket orientation
directionVectors = zeros(length(rotationMatrices),3);
for i = 1:length(rotationMatrices)
    directionVectors(i,:) = rotationMatrices(:,:,i) * [0;0;1];
end

% PLOT 1: 3D rocket trajectory
figure('Name','3D Trajectory Representation'); hold on;
% quiver3(flightState(:,1), flightState(:,2), flightState(:,3), directionVectors(:,1), directionVectors(:,2), directionVectors(:,3));

% Plot trajectory of CM
plot3(flightState(:,1), flightState(:,2), flightState(:,3), 'DisplayName', 'Ascent','LineWidth',2);
plot3(drogueState(:,1), drogueState(:,2), drogueState(:,3), 'DisplayName', 'Drogue Descent','LineWidth',2);
plot3(mainChuteState(:,1), mainChuteState(:,2), mainChuteState(:,3), 'DisplayName', 'Main Descent','LineWidth',2);
plot3(crashState(:,1), crashState(:,2), crashState(:,3), 'DisplayName', 'Ballistic Descent','LineWidth',2)

daspect([1 1 1]); pbaspect([1, 1, 1]); view(45, 45);

xLimits = [min([flightState(:,1); drogueState(:,1); mainChuteState(:,1); crashState(:,1)]) ...
           max([flightState(:,1); drogueState(:,1); mainChuteState(:,1); crashState(:,1)])];
yLimits = [min([flightState(:,2); drogueState(:,2); mainChuteState(:,2); crashState(:,2)]) ...
           max([flightState(:,2); drogueState(:,2); mainChuteState(:,2); crashState(:,2)])];
zLimits = [0 max([flightState(:,3); drogueState(:,3); mainChuteState(:,3); crashState(:,3)])];

colormap('jet');
surf(environment.map_x, environment.map_y, environment.map_z, 'EdgeColor', 'none', 'DisplayName', 'Base Map');
title '3D trajectory representation'
xlabel 'S [m]'; ylabel 'E [m]'; zlabel 'Altitude [m]';
grid on
box on
legend show;

% PLOT 2: time dependent altitude
figure('Name','Time dependent altitude'); hold on;
plot(flightTime, flightState(:,3), 'DisplayName', 'Ascent');
plot(drogueTime, drogueState(:,3), 'DisplayName', 'Drogue Descent');
plot(mainChuteTime, mainChuteState(:,3), 'DisplayName', 'Main Descent');
plot(crashTime, crashState(:,3), 'DisplayName', 'Ballistic Descent');
title 'Altitude vs. time'
xlabel 't [s]'; ylabel 'Altitude [m]';
grid on
box on
legend show;

% PLOT 3: Altitude vs. drift
figure('Name','Altitude vs Drift'); hold on;
plot(sqrt(drogueState(:,1).^2 + drogueState(:,2).^2), drogueState(:,3), 'DisplayName', 'Drogue');
plot(sqrt(mainChuteState(:,1).^2 + mainChuteState(:,2).^2), mainChuteState(:,3), 'DisplayName', 'Main');
plot(sqrt(crashState(:,1).^2 + crashState(:,2).^2), crashState(:,3), 'd', 'DisplayName', 'CrashSim');
title 'Altitude vs. drift'
xlabel 'Drift [m]'; ylabel 'Altitude [m]';
grid on
box on
legend show;

% PLOT 4: Aerodynamic properties
figure('Name','Aerodynamic properties'); hold on;

% Plot Margin
subplot(3,2,1);
plot(flightTime, simulator3D.simAuxResults.Margin)
grid on; box on; hold on;
plot(ones(1,2) * rocket.Burn_Time, ylim, 'g');
title 'Margin';

% Plot centerOfPressure
subplot(3,2,2);
plot(flightTime, simulator3D.simAuxResults.centerOfPressure)
hold on; grid on; box on;
plot(ones(1,2) * rocket.Burn_Time, ylim, 'g');
title 'X_{cp}';

% Plot AoA vs. time
subplot(3,2,3);
plot(flightTime, simulator3D.simAuxResults.Alpha)
hold on; grid on; box on;
plot(ones(1,2) * rocket.Burn_Time, ylim, 'g');
title '\alpha';

% Plot CNa vs. speed
subplot(3,2,4);
plot(flightTime, simulator3D.simAuxResults.Cn_alpha)
hold on; grid on; box on;
plot(ones(1,2) * rocket.Burn_Time, ylim, 'g');
title 'Cn_{\alpha}';

% Plot Scaled CD
subplot(3,2,5);
plot(flightTime, simulator3D.simAuxResults.dragCoefficient * 1.3) % 1.3 is scale corrective CD factor
grid on; box on; hold on;
title 'SCALED CD';

% Plot angle with vertical
subplot(3,2,6);
plot(flightTime, simulator3D.simAuxResults.delta)
ylim([0, 1]);
currentYLim = ylim;
set(gca, 'YTick', currentYLim(1):0.1:currentYLim(2));
grid on; box on; hold on;
plot(ones(1,2) * rocket.Burn_Time, ylim, 'g');
title 'delta, angle with Oz'

screenSize = get(groot, 'Screensize');
set(gcf,'Position',[screenSize(1:2), screenSize(3)*0.5, screenSize(4)]);

% PLOT 5: Mass properties
figure('Name','Mass properties'); hold on;

% Plot mass vs. time
subplot(2,2,1);
plot(flightTime, simulator3D.simAuxResults.Mass)
hold on;
plot(ones(1,2) * rocket.Burn_Time, ylim, 'g');
currentYLim = ylim;
title 'Mass';
set(gca, 'YTick', currentYLim(1):0.5:currentYLim(2));
grid on; box on;

% Plot CM vs. time
subplot(2,2,2);
plot(flightTime, simulator3D.simAuxResults.CM)
currentYLim = ylim;
title 'CM';
set(gca, 'YTick', currentYLim(1):0.03:currentYLim(2));
grid on; box on;
hold on;
plot(ones(1,2) * rocket.Burn_Time, ylim, 'g');

% Plot Il vs. time
subplot(2,2,3);
plot(flightTime, simulator3D.simAuxResults.Il)
currentYLim = ylim;
title 'Il';
set(gca, 'YTick', currentYLim(1):0.5:currentYLim(2));
grid on; box on;
hold on;
plot(ones(1,2) * rocket.Burn_Time, ylim, 'g');

% Plot Ir vs. time
subplot(2,2,4);
plot(flightTime, simulator3D.simAuxResults.Ir)
title 'Ir';
hold on;
plot(ones(1,2) * rocket.Burn_Time, ylim, 'g');
grid on; box on;

set(gcf,'Position',[screenSize(3)*0.5, screenSize(2), screenSize(3)*0.5, screenSize(3)*0.5]);

% PLOT 6: Margin plot
figure('Name','Dynamic stability margin'); hold on;
title 'Stability margin'
yyaxis left;
plot(flightTime, simulator3D.simAuxResults.CM, 'DisplayName', 'X_{CM}');
plot(flightTime, simulator3D.simAuxResults.centerOfPressure, 'DisplayName', 'X_{CP}');
ylabel 'X_{CM}, X_{CP} [cm]'
yyaxis right;
plot(flightTime, simulator3D.simAuxResults.Margin, 'DisplayName', 'Margin');
ylabel 'Margin [calibers]';
title 'Dynamic Stability Margin'
grid on; box on;
legend show;

% PLOT 7: norm of quaternion
figure('Name','Norm of quaternion'); hold on;
plot(flightTime, sqrt(sum(flightState(:, 7:10).^2, 2)));
grid on; box on;

% PLOT 8: Acceleration
figure(Name="acceleration")
accelerationX = diff(flightState(:,4))./diff(flightTime);
accelerationY = diff(flightState(:,5))./diff(flightTime);
accelerationZ = diff(flightState(:,6))./diff(flightTime);
hold on
plot(flightTime(1:end-1), accelerationX, "Color","red")
plot(flightTime(1:end-1), accelerationY, "Color","blue")
plot(flightTime(1:end-1), accelerationZ, "Color","green")
grid on; box on;
xlabel("t [s]")
ylabel("Acceleration [m \cdot s^{-2}] ")
legend("ax", "ay", "az", fontsize=15)

% PLOT 9: Euler angles
figure(Name="Euler angles")
quaternionStates = flightState(:,7:10)';
[phi, theta, psi] = quat_to_euler_angles(quaternionStates(1,:), quaternionStates(2,:), quaternionStates(3,:), quaternionStates(4,:));
hold on
plot(flightTime, phi .* 180 ./ pi, LineWidth=2)
plot(flightTime, theta .* 180 ./ pi, LineWidth=2)
plot(flightTime, psi .* 180 ./ pi, LineWidth=2)
grid on; box on;
xlabel("t [s]")
ylabel("Angles")
legend("\phi", "\theta", "\psi", fontsize=15)