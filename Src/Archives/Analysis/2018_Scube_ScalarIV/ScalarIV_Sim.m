%% Rocket Simulator 3D

% Initialize
close all; clear all;
addpath(genpath('../Declarations'),...
        genpath('../Functions'),...
        genpath('../Snippets'),...
        genpath('../Simulator_1D'));

% Rocket Definition
Rocket = rocketReader('Rocket_Definition_ScalarIV.txt');
Environment = environnementReader('Environnement_Definition_Tarbes.txt');
simulationOutputs = SimOutputReader('Simulation_outputs.txt');

simulatior3D = Simulator3D(Rocket, Environment, simulationOutputs);

%% ------------------------------------------------------------------------
% 6DOF Rail Simulation
%--------------------------------------------------------------------------

[railTime, railState] = simulatior3D.RailSim();

display(['Launch rail departure velocity : ' num2str(railState(end,2))]);

%% ------------------------------------------------------------------------
% 6DOF Boost Simulation
%--------------------------------------------------------------------------
[burnTime, burnState, burnTimeEvents, burnStateEvents, burnEventIndices] = simulatior3D.FlightSim([railTime(end) simulatior3D.Rocket.Thrust_Time(end)], railState(end,2));

%% ------------------------------------------------------------------------
% 6DOF Boost Simulation
%--------------------------------------------------------------------------

simulatior3D.Rocket.coneMode = 'off';
simulatior3D.Rocket.emptyMass = simulatior3D.Rocket.emptyMass-2.1;
simulatior3D.Rocket.emptyCenterOfMass = 1.44;
simulatior3D.Rocket.emptyInertia = 5.68;

[coastTime, coastState, coastTimeEvents, coastStateEvents, coastEventIndices] = simulatior3D.FlightSim([burnTime(end) 40], burnState(end,1:3)', burnState(end,4:6)', burnState(end,7:10)', burnState(end,11:13)');

flightTime = [burnTime; coastTime(2:end)];
flightState = [burnState;coastState(2:end, :)];

display(['Apogee AGL : ' num2str(flightState(end,3))]);
display(['Max speed : ' num2str(max(flightState(:,6)))]);
display(['Max acceleration : ' num2str(max(diff(flightState(:,6))./diff(flightTime)))]);
%% ------------------------------------------------------------------------
% 3DOF Recovery Drogue
%--------------------------------------------------------------------------

[T3, S3, drogueTimeEvents, drogueStateEvents, drogueEventIndices] = simulatior3D.DrogueParaSim(flightTime(end), flightState(end,1:3)', flightState(end, 4:6)');

%% ------------------------------------------------------------------------
% 3DOF Recovery Main
%--------------------------------------------------------------------------

[mainChuteTime, mainChuteState, mainChuteTimeEvents, S4E, mainChuteEventsIndices] = simulatior3D.MainParaSim(T3(end), S3(end,1:3)', S3(end, 4:6)');

%% ------------------------------------------------------------------------
% 3DOF Crash Simulation
%--------------------------------------------------------------------------

[crashTime, crashState, crashTimeEvents, crashStateEvents, crashEventIndices] = simulatior3D.CrashSim(flightTime(end), flightState(end,1:3)', flightState(end, 4:6)');

%% ------------------------------------------------------------------------
% Analyse results
%--------------------------------------------------------------------------

% PLOT 1 : 3D rocket trajectory

C = quat2rotmat(flightState(:, 7:10));
angle = rot2anglemat(C);

% plot rocket orientation
figure; hold on;
directionVectors = zeros(length(C),3);
for i  = 1:length(C)
    directionVectors(i,:) = C(:,:,i)*[0;0;1];
end
%quiver3(flightState(:,1), flightState(:,2), flightState(:,3), directionVectors(:,1), directionVectors(:,2), directionVectors(:,3));

% plot trajectory of CM
plot3(flightState(:,1), flightState(:,2), flightState(:,3), 'DisplayName', 'Ascent');
plot3(S3(:,1), S3(:,2), S3(:,3), 'DisplayName', 'Drogue Descent');
plot3(mainChuteState(:,1), mainChuteState(:,2), mainChuteState(:,3), 'DisplayName', 'Main Descent');
plot3(crashState(:,1), crashState(:,2), crashState(:,3), 'DisplayName', 'Ballistic Descent')
daspect([1 1 1]); pbaspect([1, 1, 1]); view(45, 45);
% [XX, YY, M, Mcolor] = get_google_map(Environment.startLatitude, Environment.startLongitude, 'Height', ceil(diff(xlim)/3.4), 'Width', ceil(diff(ylim)/3.4));
% xImage = [xlim',xlim'];
% yImage = [ylim;ylim];
% zImage = zeros(2);
% colormap(Mcolor);
% surf(xImage, yImage, zImage, 'CData', M,'FaceColor', 'texturemap', 'EdgeColor', 'none', 'DisplayName', 'Base Map');
title '3D trajectory representation'
xlabel 'S [m]'; ylabel 'E [m]'; zlabel 'Altitude [m]';
legend show;

% PLOT 2 : time dependent altitude
figure; hold on;
plot(flightTime, flightState(:,3));
plot(T3, S3(:,3));
plot(mainChuteTime, mainChuteState(:,3));
plot(crashTime, crashState(:,3));
title 'Altitude vs. time'
xlabel 't [s]'; ylabel 'Altitude [m]';

% PLOT 3 : Altitude vs. drift
figure; hold on;
plot(sqrt(flightState(:,1).^2 + flightState(:,2).^2), flightState(:,3));
%quiver(sqrt(flightState(:,1).^2 + flightState(:,2).^2), flightState(:,3), sqrt(directionVectors(:,1).^2 + directionVectors(:,2).^2), directionVectors(:,3));
plot(sqrt(S3(:,1).^2 + S3(:,2).^2), S3(:,3));
plot(sqrt(mainChuteState(:,1).^2 + mainChuteState(:,2).^2), mainChuteState(:,3));
plot(sqrt(crashState(:,1).^2 + crashState(:,2).^2), crashState(:,3));
title 'Altitude vs. drift'
xlabel 'Drift [m]'; ylabel 'Altitude [m]';
daspect([1 1 1]);

% PLOT 4 : Aerodynamic properties
figure; hold on;
% Plot Margin
subplot(3,2,1);
plot(flightTime, simulatior3D.simAuxResults.Margin)
hold on;
plot(ones(1,2)*Rocket.Burn_Time, ylim, 'g');
title 'Margin';
% Plot Xcp
subplot(3,2,2);
plot(flightTime, simulatior3D.simAuxResults.Xcp)
hold on;
plot(ones(1,2)*Rocket.Burn_Time, ylim, 'g');
title 'X_{cp}';
% Plot AoA vs. time
subplot(3,2,3);
plot(flightTime, simulatior3D.simAuxResults.Alpha)
hold on;
plot(ones(1,2)*Rocket.Burn_Time, ylim, 'g');
title '\alpha';
% Plot CNa vs. time
subplot(3,2,4);
plot(flightTime, simulatior3D.simAuxResults.Cn_alpha)
hold on;
plot(ones(1,2)*Rocket.Burn_Time, ylim, 'g');
title 'Cn_{\alpha}';
% Plot Cd vs. time
subplot(3,2,5);
plot(flightTime, simulatior3D.simAuxResults.Cd)
ylim([0, 1.5]);
tmpYlim = ylim;
set(gca, 'YTick', tmpYlim(1):0.1:tmpYlim(2));
hold on;
plot(ones(1,2)*Rocket.Burn_Time, ylim, 'g');
title 'Cd'
screensize = get( groot, 'Screensize' );
set(gcf,'Position',[screensize(1:2), screensize(3)*0.5,screensize(4)]);

% PLOT 5 : Mass properties
figure; hold on;
% Plot mass vs. time
subplot(2,2,1);
plot(flightTime, simulatior3D.simAuxResults.Mass)
hold on;
plot(ones(1,2)*Rocket.Burn_Time, ylim, 'g');
tmpYlim = ylim;
title 'Mass';
set(gca, 'YTick', tmpYlim(1):0.5:tmpYlim(2));
hold on;
plot(ones(1,2)*Rocket.Burn_Time, ylim, 'g');
% Plot CM vs. time
subplot(2,2,2);
plot(flightTime, simulatior3D.simAuxResults.CM)
tmpYlim = ylim;
title 'CM';
set(gca, 'YTick', tmpYlim(1):0.01:tmpYlim(2));
hold on;
plot(ones(1,2)*Rocket.Burn_Time, ylim, 'g');
% Plot Il vs. time
subplot(2,2,3);
plot(flightTime, simulatior3D.simAuxResults.Il)
tmpYlim = ylim;
title 'Il';
set(gca, 'YTick', tmpYlim(1):0.1:tmpYlim(2));
hold on;
plot(ones(1,2)*Rocket.Burn_Time, ylim, 'g');
%Plot Ir vs. time
subplot(2,2,4);
plot(flightTime, simulatior3D.simAuxResults.Ir)
title 'Ir';
hold on;
plot(ones(1,2)*Rocket.Burn_Time, ylim, 'g');
screensize = get( groot, 'Screensize' );
set(gcf,'Position',[screensize(3)*0.5, screensize(2),...
    screensize(3)*0.5,screensize(3)*0.5]);            

% PLOT 6 : Margin plot
figure; hold on;
title 'Stability margin'
yyaxis left;
plot(flightTime, simulatior3D.simAuxResults.CM, 'DisplayName', 'X_{CM}');
plot(flightTime, simulatior3D.simAuxResults.Xcp, 'DisplayName', 'X_{CP}');
ylabel 'X_{CM}, X_{CP} [cm]'
yyaxis right;
plot(flightTime, simulatior3D.simAuxResults.Margin, 'DisplayName', 'Margin');
ylabel 'Margin [calibers]';
title 'Dynamic Stability Margin'
legend show;