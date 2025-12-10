%% Rocket Simulator 3D

% Initialize
close all; clear all;
addpath(genpath('../Declarations'),...
        genpath('../Functions'),...
        genpath('../Snippets'),...
        genpath('../Simulator_1D'));
% Rocket Definition
Rocket = rocketReader('WH_test.txt');
Environment = environnementReader('Environment/Environnement_Definition_Wasserfallen.txt');
simulationOutputs = SimOutputReader('Simulation/Simulation_outputs.txt');

simulatior3D = Simulator3D(Rocket, Environment, simulationOutputs);

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

%figure('Name','Aerodynamic properties'); hold on;

%plot(diff(combinedRailFlightState(:,2))./diff(combinedRailFlightTime));
%legend show;

%plot(flightState(:,1), flightState(:,6));

%% ------------------------------------------------------------------------
% 3DOF Recovery Drogue
%--------------------------------------------------------------------------

[drogueTime, drogueState, drogueTimeEvents, drogueStateEvents, drogueEventIndices] = simulatior3D.DrogueParaSim(flightTime(end), flightState(end,1:3)', flightState(end, 4:6)');
%figure('Name','Aerodynamic properties'); hold on;

%plot(diff(drogueState(:,3))./diff(drogueTime));
legend show;

%% ------------------------------------------------------------------------
% 3DOF Recovery Main
%--------------------------------------------------------------------------
% 
[mainChuteTime, mainChuteState, mainChuteTimeEvents, S4E, mainChuteEventsIndices] = simulatior3D.MainParaSim(drogueTime(end), drogueState(end,1:3)', drogueState(end, 4:6)');
figure('Name','Parachute descent'); hold on;
plot(drogueTime,abs(drogueState(:,6)));
plot(mainChuteTime,abs(mainChuteState(:,6)));
legend show;

display(['Touchdown @t = ' num2str(mainChuteTime(end)) ' = ' num2str(floor(mainChuteTime(end)/60)) ' min ' num2str(mod(mainChuteTime(end),60)) ' s']);

%% ------------------------------------------------------------------------
% 3DOF Crash Simulation
%--------------------------------------------------------------------------

[crashTime, crashState, crashTimeEvents, crashStateEvents, crashEventIndices] = simulatior3D.CrashSim(flightTime(end), flightState(end,1:3)', flightState(end, 4:6)');

%% ------------------------------------------------------------------------
% 6DOF Crash Simulation for the nosecone
%--------------------------------------------------------------------------

% % There is currently an error with the integration
% 
% Nosecone = rocketReader('Rocket_Definition_Eiger_I_Final_Nosecone.txt');
% 
% % simulationObj2 = Simulator3D(Nosecone, Environment, simulationOutputs);
% simulatior3D.Rocket = Nosecone;
% 
% [T6, S6, T6E, S6E, I6E] = simulatior3D.Nose_CrashSim_6DOF([flightTime(end) 40], flightState(end, 1:3)', flightState(end, 4:6)', flightState(end, 7:10)', flightState(end, 11:13)');


%% ------------------------------------------------------------------------
% Payload Crash Simulation
%--------------------------------------------------------------------------

%[T7, S7, T7E, S7E, I7E] = simulatior3D.CrashSim(flightTime(end), flightState(end,1:3)', flightState(end, 4:6)');

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
plot3(drogueState(:,1), drogueState(:,2), drogueState(:,3), 'DisplayName', 'Drogue Descent','LineWidth',2);
plot3(mainChuteState(:,1), mainChuteState(:,2), mainChuteState(:,3), 'DisplayName', 'Main Descent','LineWidth',2);
plot3(crashState(:,1), crashState(:,2), crashState(:,3), 'DisplayName', 'Ballistic Descent','LineWidth',2)
daspect([1 1 1]); pbaspect([1, 1, 1]); view(45, 45);
%[XX, YY, M, Mcolor] = get_google_map(Environment.startLatitude, Environment.startLongitude, 'Height', ceil(diff(xlim)/3.4), 'Width', ceil(diff(ylim)/3.4));
xImage = [xlim',xlim'];
yImage = [ylim;ylim];
zImage = zeros(2);
colormap('jet');
%surf(xImage, yImage, zImage, 'CData', M,'FaceColor', 'texturemap', 'EdgeColor', 'none', 'DisplayName', 'Base Map');
surf(Environment.map_x, Environment.map_y, Environment.map_z, 'EdgeColor', 'none', 'DisplayName', 'Base Map');
title '3D trajectory representation'
xlabel 'S [m]'; ylabel 'E [m]'; zlabel 'Altitude [m]';
legend show;

% PLOT 2 : time dependent altitude
figure('Name','Time dependent altitude'); hold on;
plot(flightTime, flightState(:,3), 'DisplayName', 'Ascent');
plot(drogueTime, drogueState(:,3), 'DisplayName', 'Drogue Descent');
plot(mainChuteTime, mainChuteState(:,3), 'DisplayName', 'Main Descent');
plot(crashTime, crashState(:,3), 'DisplayName', 'Ballistic Descent');
%plot(T6, S6(:,3), 'DisplayName', 'Ballistic Nosecone Descent', 'LineWidth', 2);
title 'Altitude vs. time'
xlabel 't [s]'; ylabel 'Altitude [m]';
legend show;

% PLOT 3 : Altitude vs. drift
figure('Name','Altitude vs Drift')'; hold on;
%plot(sqrt(flightState(:,1).^2 + flightState(:,2).^2), flightState(:,3), '*', 'DisplayName', 'Flight');
%quiver(sqrt(flightState(:,1).^2 + flightState(:,2).^2), flightState(:,3), sqrt(directionVectors(:,1).^2 + directionVectors(:,2).^2), directionVectors(:,3));
plot(sqrt(drogueState(:,1).^2 + drogueState(:,2).^2), drogueState(:,3), 'DisplayName', 'Drogue');
plot(sqrt(mainChuteState(:,1).^2 + mainChuteState(:,2).^2), mainChuteState(:,3), 'DisplayName', 'Main');
plot(sqrt(crashState(:,1).^2 + crashState(:,2).^2), crashState(:,3), 'd', 'DisplayName', 'CrashSim');
title 'Altitude vs. drift'
xlabel 'Drift [m]'; ylabel 'Altitude [m]';
%daspect([1 1 1]);
legend show;

% PLOT 4 : Aerodynamic properties
figure('Name','Aerodynamic properties'); hold on;
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
% Plot CNa vs. speed
subplot(3,2,4);
plot(flightTime, simulatior3D.simAuxResults.Cn_alpha)
hold on;
plot(ones(1,2)*Rocket.Burn_Time, ylim, 'g');
title 'Cn_{\alpha}';

subplot(3,2,5);
plot(flightTime, simulatior3D.simAuxResults.Cd*1.3) % 1.3 is scale corrective CD factor!
hold on;
title 'SCALED CD';


% Plot angle with vertical
subplot(3,2,6);
plot(flightTime, simulatior3D.simAuxResults.Delta)
ylim([0, 1]);
tmpYlim = ylim;
set(gca, 'YTick', tmpYlim(1):0.1:tmpYlim(2));
hold on;
plot(ones(1,2)*Rocket.Burn_Time, ylim, 'g');
title 'Delta, angle with Oz'
screensize = get( groot, 'Screensize' );
set(gcf,'Position',[screensize(1:2), screensize(3)*0.5,screensize(4)]);


% figure('Name',' CD against speed up'); hold on;
% 
% [max,idx] = max(flightState(:,6));
%  %[~, index] = unique(RAW{:,1}); 0.0: 0.01 :  max(flightState(:,6))
%  AX = interp1(flightState(1:idx,6),simulatior3D.simAuxResults.Cd(1:idx),  flightState(1,6): 0.01 :  max , 'pchip', 'extrap');  
% 
%       %      to_log = transpose([ 20.0: 0.01 : bound+20 ; AX ; AY ; AZ ; P ]);
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot( flightState(1,6): 0.01 :  max , AX ) %simulatior3D.simAuxResults.Cd
% ylim([0, 3]);
% xlim([flightState(1,6),max]);
% tmpYlim = ylim;
% set(gca, 'YTick', tmpYlim(1):0.2:tmpYlim(2));
% hold on;
% title 'Cd'
% 
% AY = interp1(flightState(idx:end,6),simulatior3D.simAuxResults.Cd(idx:end),  flightState(end,6): 0.01 :  max , 'pchip', 'extrap');  
% plot( flightState(end,6): 0.01 :  max , AY ) %simulatior3D.simAuxResults.Cd
% ylim([0, 3]);
% tmpYlim = ylim;
% set(gca, 'YTick', tmpYlim(1):0.2:tmpYlim(2));
% hold on;
% title 'Cd'





% PLOT 5 : Mass properties
figure('Name','Mass properties'); hold on;
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
figure('Name','Dynamic stability margin'); hold on;
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

% plot 7 : norm of quaternion
figure('Name','Norm of quaternion'); hold on;
plot(flightTime, sqrt(sum(flightState(:, 7:10).^2, 2)));

% % Plot 8
% figure('Name','Nosecone crash angles'); hold on;
% % AoA
% subplot(1,2,1);
% plot(T6, simulatior3D.simAuxResults.Nose_Alpha)
% title '\alpha';
% % Delta, angle with vertical
% subplot(1,2,2);
% plot(T6, simulatior3D.simAuxResults.Nose_Delta)
% ylim([0, 1]);
% tmpYlim = ylim;
% set(gca, 'YTick', tmpYlim(1):0.1:tmpYlim(2));
% title 'Delta, angle with Oz'