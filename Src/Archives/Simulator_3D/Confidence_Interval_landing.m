% Initialize
%close all; 
%clear all;
addpath(genpath('../Declarations'),...
        genpath('../Functions'),...
        genpath('../Snippets'),...
        genpath('../Simulator_1D'));

% Rocket Definition
n_sim = 20;
Rocket_0 =  rocketReader('Euroc_final_objectif.txt');
simulationOutputs = SimOutputReader('Simulation/Simulation_outputs.txt');
name_of_environnment = 'Environnement_Definition_EuRoC.txt';
%map=xyz2grid('wasserfallen map data.xyz');
%Environment =environnementReader(name_of_environnment);


angle=0;
var_angle=10;
n_sim_angle=4;
azi=linspace(angle-var_angle,angle+var_angle-10,n_sim_angle);
%azi=(angle);
figure('Name','montecarlo'); hold on;
for i=1:n_sim_angle
wind_ch([1 0.5 1 1 2 2 2 1 2 2 4],[10 100 250 500 750 1000 1500 2000 2500 3000 3600],azi(i),0.2,name_of_environnment);
if n_sim ~= 0
[azed,r_ellipse,r_ellipse1, X0, Y0, data] = landing_tool(n_sim,-1,-1, Rocket_0, simulationOutputs, name_of_environnment );
end



Environment = environnementReader(name_of_environnment,1);
% simulatior3D = multilayerwindSimulator3D(Rocket_0, Environment, simulationOutputs);
% [railTime, railState] = simulatior3D.RailSim();
% [burnTime, burnState, burnTimeEvents, burnStateEvents, burnEventIndices] = simulatior3D.FlightSim([railTime(end) simulatior3D.Rocket.Burn_Time(end)], railState(end, 2));
% [coastTime, coastState, coastTimeEvents, coastStateEvents, coastEventIndices] = simulatior3D.FlightSim([burnTime(end) 40], burnState(end, 1:3)', burnState(end, 4:6)', burnState(end, 7:10)', burnState(end, 11:13)');
% flightTime = [burnTime; coastTime(2:end)];
% flightState = [burnState; coastState(2:end, :)];
% combinedRailFlightTime = [railTime;flightTime];
%combinedRailFlightState = [railState;flightState(:,3) flightState(:,6)];
%[drogueTime, drogueState, ~, ~, ~] = simulatior3D.CrashSim(flightTime(end), flightState(end,1:3)', flightState(end, 4:6)');
% [drogueTime, drogueState, drogueTimeEvents, drogueStateEvents, drogueEventIndices] = simulatior3D.DrogueParaSim(flightTime(end), flightState(end,1:3)', flightState(end, 4:6)');
% [mainChuteTime, mainChuteState, mainChuteTimeEvents, S4E, mainChuteEventsIndices] = simulatior3D.MainParaSim(drogueTime(end), drogueState(end,1:3)', drogueState(end, 4:6)');

% To output results in CSV format for further analysis
%c= {railTime; ; burnTime; burnState; coastTime; coastState; combinedRailFlightTime; combinedRailFlightState; drogueTime; drogueState; mainChuteTime; mainChuteState };
%d= {flightState(:,1); flightState(:,2); flightState(:,3); drogueState(:,1); drogueState(:,2); drogueState(:,3); mainChuteState(:,1); mainChuteState(:,2); mainChuteState(:,3)};
%T = cell2table(d);
%writetable(T,'myDataFile.csv');

%plot rocket orientation
%figure('Name','montecarlo'); hold on;
%plot trajectory of CM
zoom = 14;
%[XX, YY, M, Mcolor] = get_google_map(Environment.startLatitude, Environment.startLongitude, 'Height', 640, 'Width', 640, 'Zoom', zoom);
metersPerPx = 156543.03392 * cos(Environment.startLatitude*pi/180)/ 2^zoom;
lim = metersPerPx*640/2; % because [-lim + lim ] = 2 lim
xlim = [-lim lim];
ylim = [-lim lim];
xImage = [xlim',xlim'];
yImage = [ylim;ylim];
zImage = zeros(2);
%colormap(Mcolor);
if (azi(i)==azi(1))
%surf(xImage, yImage, zImage, 'CData', M,'FaceColor', 'texturemap', 'EdgeColor', 'none', 'DisplayName', 'Base Map');
colormap('parula');
surf(Environment.map_x, Environment.map_y, Environment.map_z, 'EdgeColor', 'none', 'DisplayName', 'Base Map');
end
% plot3(flightState(:,1), flightState(:,2), flightState(:,3), 'DisplayName', 'Ascent','LineWidth',2);
% plot3(drogueState(:,1), drogueState(:,2), drogueState(:,3), 'DisplayName', 'Drogue Descent','LineWidth',2);
%plot3(mainChuteState(:,1), mainChuteState(:,2), mainChuteState(:,3), 'DisplayName', 'Main Descent','LineWidth',2);
%find_altitude(r_ellipse1(:,1) + X0,r_ellipse1(:,2) + Y0,Environment)
if n_sim ~= 0
%plot3(r_ellipse(:,1) + X0,r_ellipse(:,2) + Y0,0*r_ellipse(:,2),'DisplayName', '95% confidence Interval','LineWidth',2);
plot3(r_ellipse1(:,1) + X0,r_ellipse1(:,2) + Y0,find_altitude(r_ellipse1(:,1) + X0,r_ellipse1(:,2) + Y0,Environment),'k-','DisplayName', [num2str(azi(i)),'°'],'LineWidth',2);
%plot3(data(:,1), data(:,2),find_altitude(data(:,1), data(:,2),Environment),'*k' , 'DisplayName', 'noised landing');
end
%plot3(-447, 114,0,'*r' , 'DisplayName', 'real landing');
daspect([1 1 1]); pbaspect([1, 1, 2]); view(45, 45);
title '99.99% confidence interval landing zone' %'3D trajectory representation'
xlabel 'S [m]'; ylabel 'E [m]'; zlabel 'Altitude [m]';
legend show;
end