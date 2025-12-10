%% Analysis of USA flight
close all;
clear all;

% Initialize
close all; clear all;
addpath(genpath('../Declarations'),...
        genpath('../Functions'),...
        genpath('../Snippets'),...
        genpath('../Simulator_3D'),...
        genpath('../FlightData/USA'));
    
%% Load Nosecone data

nose_data = csvread('../FlightData/USA/ERT18-MATTERHORN II - flight_data telemetry.csv', 0, 0);
nose_time = 3;
nose_alt = 4;
nose_vspd = 5;
nose_pres = 6;
nose_temp = 7;
nose_accX = 8;
nose_accY = 9;
nose_accZ = 10;
nose_fs = 11;
nose_ab = 12;
nose_dp = 13;
nose_t_lo_i = find(abs(nose_data(:,nose_accX))>5, 1, 'first');
nose_t_lo = nose_data(nose_t_lo_i,nose_time); 
nose_alt_init = nose_data(nose_t_lo_i,nose_alt); 
%nose_data(:, nose_time) = (nose_data(:, nose_time)- nose_t_lo)/1000;
nose_data(:, nose_alt) = nose_data(:, nose_alt) - nose_alt_init;    
    
%% load GPS data
gps_data = csvread('../FlightData/USA/ERT18-MATTERHORN II - flight_data_telemetry_gps.csv', 1, 0);
gps_time = 1;
gps_sats = 2;
gps_lat = 3;
gps_lon = 4;
gps_alt = 5;
gps_t_lo_i = find(gps_data(:,gps_time)>nose_t_lo, 1, 'first')+1;
gps_t_cl_i = find(gps_data(:,gps_sats)==0, 1, 'first');
gps_t_lo = gps_data(gps_t_lo_i, gps_time);
gps_alt_lo = gps_data(gps_t_lo_i, gps_alt);
gps_x = gps_data(gps_t_lo_i:gps_t_cl_i, gps_lat)/180*pi*6783e3; gps_x = gps_x - gps_x(1);
gps_y = gps_data(gps_t_lo_i:gps_t_cl_i, gps_lon)/180*pi*6783e3; gps_y = -(gps_y - gps_y(1));
gps_h = gps_data(gps_t_lo_i:gps_t_cl_i, gps_alt) - gps_alt_lo;
gps_v = [diff(gps_x), diff(gps_y), diff(gps_h)]./diff(gps_data(gps_t_lo_i:gps_t_cl_i, gps_time))*1000;

%% Interpolate 3D points
nose_alt_int = interp1(nose_data(:, nose_time), nose_data(:, nose_alt), gps_data(gps_t_lo_i:gps_t_cl_i, gps_time));

%% Find general flight azimuth
p = polyfit(gps_x, gps_y, 1);
angle = atand(p(1));
%% Run simulation

% Rocket Definition
Rocket = rocketReader('Rocket_Definition_Final.txt');
Environment = environnementReader('Environnement_Definition_USA.txt');
simulationOutputs = SimOutputReader('Simulation_outputs.txt');

R = rotmat(-pi/2, 3);

simulatior3D = Simulator3D(Rocket, Environment, simulationOutputs);

% ------------------------------------------------------------------------
% 6DOF Rail Simulation
%--------------------------------------------------------------------------

[railTime, railState] = simulatior3D.RailSim();

display(['Launch rail departure velocity : ' num2str(railState(end,2))]);

% ------------------------------------------------------------------------
% 6DOF Boost Simulation
%--------------------------------------------------------------------------
[burnTime, burnState, burnTimeEvents, burnStateEvents, burnEventIndices] = simulatior3D.FlightSim([railTime(end) simulatior3D.Rocket.Thrust_Time(end)], railState(end,2));

SimX = (R*burnState(:,1:3)')';

%% Compute flight path from end of GPS data to apogee

simulatior3D.Rocket.coneMode = 'off';
simulatior3D.Rocket.emptyMass = simulatior3D.Rocket.emptyMass-2.1;
simulatior3D.Rocket.emptyCenterOfMass = 1.44;
simulatior3D.Rocket.emptyInertia = 5.68;
simulatior3D.Environment.V_inf = 2;

V0 = [gps_v(end-1,1),gps_v(end-1,2), gps_v(end-1,3)]';
phi = atan2(norm(cross(V0, [0;0;1])), dot(V0, [0;0;1]));
n = cross(V0, [0;0;1]); n = n/norm(n); 
Q0 = [n*sin(phi/2); cos(phi/2)];
W = [0, 0, 0]';

[coastTime, coastState, coastTimeEvents, coastStateEvents, coastEventIndices] = simulatior3D.FlightSim([simulatior3D.Rocket.Thrust_Time 40],...
    [gps_x(end), gps_y(end), gps_h(end)]',...
    V0,...
    Q0,...
    W);

%% Compute payload flight path

T0 = coastTime(end);
X0 = coastState(end, 1:3)';
V0 = coastState(end, 4:6)';

[T6, S6] = simulatior3D.PayloadCrashSim(T0, X0, V0);

%% Compute flight path from burnout to apogee

% [T2_3, S2_3, T2_3E, S2_3E, I2_3E] = simulatior3D.FlightSim([coastTime(end) 40],...
%     coastState(end, 1:3)',...
%     coastState(end, 4:6)',...
%     coastState(end, 7:10)',...
%     coastState(end, 11:13)');

%% Plot 3D trajectory
figure; hold on;
plot3(gps_x, gps_y, nose_alt_int, 'DisplayName', 'GPS - Baro');
plot3(gps_x, gps_y, gps_h, 'DisplayName', 'GPS - GPS');
plot3(SimX(:,1), SimX(:,2), SimX(:,3), 'DisplayName', 'Simulation Start');
plot3(coastState(:,1), coastState(:,2), coastState(:,3), 'DisplayName', 'Simulation To Apogee');
plot3(S6(:,1), S6(:,2), S6(:,3), 'DisplayName', 'Payload Crash Simulation');
quiver3(gps_x(1:end-1), gps_y(1:end-1), gps_h(1:end-1), gps_v(:,1),gps_v(:,2), gps_v(:,3), 'DisplayName', 'GPS - Velocity');
daspect([1,1,1]); pbaspect([0.5, 0.5, 1]);
legend show;
xlabel('N'); ylabel('W');


display('Apogee location:');
display(['N: ', num2str(coastState(end,1)), ', W: ', num2str(coastState(end,2))]);
display(['Distance : ', num2str(sqrt(coastState(end,1)^2 + coastState(end,2)^2))]);
display(['Angle : ', num2str(atand(coastState(end,2)/coastState(end,1)))]);

display('Payload landing location:');
display(['N: ', num2str(S6(end,1)), ', W: ', num2str(S6(end,2))]);
display(['Distance : ', num2str(sqrt(S6(end,1)^2 + S6(end,2)^2))]);
display(['Angle : ', num2str(atand(S6(end,2)/S6(end,1)))]);
