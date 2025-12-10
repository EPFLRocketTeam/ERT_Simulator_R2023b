%% Initialize

close all
clear all

% load directories
addpath(genpath('../Declarations'),...
        genpath('../Functions'),...
        genpath('../Snippets'),...
        genpath('../Simulator_3D'));
    
% load defintions
Rocket_USA = rocketReader('Rocket_Definition_Eiger_I_Final.txt');
Environment_USA = environnementReader('Environnement_Definition_USA.txt');
simulationOutputs = SimOutputReader('Simulation_outputs.txt');

%%  Simulate

simulatior3D = Simulator3D(Rocket_USA, Environment_USA, simulationOutputs);

% ------------------------------------------------------------------------
% 6DOF Rail Simulation
%--------------------------------------------------------------------------

[railTime, railState] = simulatior3D.RailSim();

% ------------------------------------------------------------------------
% 6DOF Flight Simulation
%--------------------------------------------------------------------------

[flightTime, flightState] = simulatior3D.FlightSim([railTime(end) 40], railState(end,2));

% ------------------------------------------------------------------------
% 3DOF Recovery Drogue
%--------------------------------------------------------------------------

[drogueTime, drogueState] = simulatior3D.DrogueParaSim(flightTime(end), flightState(end,1:3)', flightState(end, 4:6)');

% ------------------------------------------------------------------------
% 3DOF Recovery Main
%--------------------------------------------------------------------------

[mainChuteTime, mainChuteState] = simulatior3D.MainParaSim(drogueTime(end), drogueState(end,1:3)', drogueState(end, 4:6)');

%% Generate data
T0 = [0 3 5]';
H0 = [0 0 0]';
V0 = [0 0 0]';

% time = [T0; T0(end)+railTime(2:end); T0(end)+flightTime(2:end); T0(end)+drogueTime(2:end); T0(end)+mainChuteTime(2:end)];
% altitude = [H0; railState(2:end,1);flightState(2:end,3);drogueState(2:end,3);mainChuteState(2:end,3)]+Environment_USA.startAltitude;
% pressure = zeros(size(altitude));
% for i = 1:length(altitude)
%    [~, ~, pressure(i), ~, ~] = stdAtmos(altitude(i),Environment_USA);
% end
% velocity = [V0; railState(2:end,2);flightState(2:end,6);drogueState(2:end,6);mainChuteState(2:end,6)];
% acceleration = diff(velocity)./diff(time);

time = [T0; T0(end)+railTime(2:end); T0(end)+flightTime(2:end)];
altitude = [H0; railState(2:end,1);flightState(2:end,3)]+Environment_USA.startAltitude;
pressure = zeros(size(altitude));
for i = 1:length(altitude)
   [~, ~, pressure(i), ~, ~] = stdAtmos(altitude(i),Environment_USA);
end
velocity = [V0; railState(2:end,2);flightState(2:end,6)];
acceleration = diff(velocity)./diff(time);

%% Sample simulation
t = (0:0.1:time(end))';
altitude_s = interp1(time, altitude, t, 'linear', 'extrap');
pressure_s = interp1(time, pressure, t, 'linear', 'extrap');
acceleration_s = interp1(time(1:end-1), acceleration, t, 'linear', 'extrap');
velocity_s = interp1(time, velocity, t, 'linear', 'extrap');

%% Add noise to samples
n_samples = length(t);

%% Create file

% open file
headerId = fopen('SimDataUSA.h', 'w');
% write header data
fprintf(headerId, '#ifndef INCLUDE_SIM_DATA_ \n');
fprintf(headerId, '#define INCLUDE_SIM_DATA_ \n\n');
fprintf(headerId, ['#define SIM_TAB_HEIGHT ' num2str(length(t)) '\n']);
fprintf(headerId, ['#define SIM_TAB_WIDTH ' num2str(4) '\n']);
fprintf(headerId, '#define SIM_TIMESTAMP 0\n');
fprintf(headerId, '#define SIM_ALTITUDE 1\n');
fprintf(headerId, '#define SIM_PRESSURE 2\n');
fprintf(headerId, '#define SIM_ACCELX 3\n');
fprintf(headerId, '#define SIM_VELOCITYX 4\n');
fprintf(headerId, ['static const float32_t SimData[' num2str(length(t)) '][' num2str(5) '] = {\n']);

% populate Array
for i = 1:length(t)
   fprintf(headerId, [writeCArray([t(i)*1000 altitude_s(i) pressure_s(i) acceleration_s(i)/9.81 velocity_s(i)]) ',\n']); 
end

fprintf(headerId, '};\n');
fprintf(headerId, '#endif');

fclose(headerId);

%% Plot table values
figure;
plot(t, altitude_s);
title 'Altitude'; xlabel 't [s]'; ylabel 'h [m]';

figure;
plot(t, velocity_s);
title 'Velocity'; xlabel 't [s]'; ylabel 'v [m/s]';

figure;
plot(t, acceleration_s);
title 'Acceleration'; xlabel 't [s]'; ylabel 'a [m/s2]';

figure;
plot(t, pressure_s);
title 'Pressure'; xlabel 't [s]'; ylabel 'P [Pa]';

figure;
plot(altitude_s-altitude_s(1), velocity_s);
title 'Speed vs. altitude'; xlabel 'h [m]'; ylabel 'v [m/s]';

%% write csv with data
csvwrite('SimDataUSA.csv', [t*1000 altitude_s pressure_s acceleration_s/9.81 velocity_s]);

%% CAN Data Generation
id_can = (1:length(t)*4)';
time_can = kron(t, ones(4,1));
data_can = kron((altitude_s-altitude_s(1))*1000, [1;0;0;0]);
data_can = data_can + kron(velocity_s*1000, [0;1;0;0]);
data_can = data_can + kron(pressure_s, [0;0;1;0]);
data_can = data_can + kron(acceleration_s*100, [0;0;0;1]);
data_code = repmat([42; 45; 0; 3], length(altitude_s), 1);
data_tens = 10*ones(size(data_can));
data_zeros = zeros(size(data_can));
csvwrite('SimDataUSA_CAN.csv', [id_can time_can data_code data_can data_tens data_zeros ]);