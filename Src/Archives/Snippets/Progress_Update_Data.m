clear all;
close all;

% Initialize
close all; clear all;
addpath(genpath('../Declarations'),...
        genpath('../Functions'),...
        genpath('../Simulator_3D'),...
        genpath('../Snippets'));

% Rocket Definition
Rocket = rocketReader('Rocket/Rocket_Definition_Eiger_I_Final.txt');

Payload_mass = 4;

% Conversion 
m_to_in = 39.3700787;
kg_to_lbs = 2.20462262185;

%% Rocket Information

Airframe_Length_in = Rocket.totalLength * m_to_in;
Airframe_Diameter_in = Rocket.maxDiameter * m_to_in;
Fin_span_in = Rocket.finSpan * m_to_in;
% LV weight with casing and without payload
Vehicle_wieght_lbs = (Rocket.emptyMass + Rocket.casing_mass - Payload_mass) * kg_to_lbs;
Propellant_weight_lbs = Rocket.propel_mass * kg_to_lbs;
Payload_weight_lbs = Payload_mass * kg_to_lbs;
% Sum below is identical to un-commented calculation
% Liftoff_weight_lbs = Vehicle_wieght_lbs + Propellant_weight_lbs + Payload_weight_lbs;
Liftoff_weight_lbs = (Rocket.emptyMass + Rocket.motor_mass) * kg_to_lbs;

emptyInertianformation = [Airframe_Length_in;
    Airframe_Diameter_in;
    Fin_span_in;
    Vehicle_wieght_lbs;
    Propellant_weight_lbs;
    Payload_weight_lbs;
    Liftoff_weight_lbs];

display(['Airframe_Length_in = ' num2str(Airframe_Length_in)]);
display(['Airframe_Diameter_in = ' num2str(Airframe_Diameter_in)]);
display(['Fin_span_in = ' num2str(Fin_span_in)]);
display(['Vehicle_wieght_lbs = ' num2str(Vehicle_wieght_lbs)]);
display(['Propellant_weight_lbs = ' num2str(Propellant_weight_lbs)]);
display(['Payload_weight_lbs = ' num2str(Payload_weight_lbs)]);
display(['Liftoff_weight_lbs = ' num2str(Liftoff_weight_lbs)]);

%% Predicted Flight Analysis and Data

% Rocket Definition
Environment = environnementReader('Environment/Environnement_Definition_USA.txt');
simulationOutputs = SimOutputReader('Simulation/Simulation_outputs.txt');

simulatior3D = Simulator3D(Rocket, Environment, simulationOutputs);

warning('off','all')

%--------------------------------------------------------------------------
% 6DOF Rail Simulation
%--------------------------------------------------------------------------

[railTime, railState] = simulatior3D.RailSim();

%--------------------------------------------------------------------------
% 6DOF Flight Simulation
%--------------------------------------------------------------------------

[burnTime, burnState, burnTimeEvents, burnStateEvents, burnEventIndices] = simulatior3D.FlightSim([railTime(end) simulatior3D.Rocket.Burn_Time(end)], railState(end, 2));

%simulatior3D.Rocket.coneMode = 'off';

[coastTime, coastState, coastTimeEvents, coastStateEvents, coastEventIndices] = simulatior3D.FlightSim([burnTime(end) 40], burnState(end, 1:3)', burnState(end, 4:6)', burnState(end, 7:10)', burnState(end, 11:13)');

flightTime = [burnTime; coastTime(2:end)];
flightState = [burnState; coastState(2:end, :)];

% Constants
g0 = 9.80665; %[m/sec^2] gravity at sea level
m_to_feet = 3.2808399;

% Results

% Considering peak thrust at liftoff
Liftoff_thrust_to_weight_ratio = max(Rocket.Thrust_Force) / ((Rocket.emptyMass + Rocket.motor_mass) * g0);
Launch_rail_departure_velocity_ft = railState(end,2) * m_to_feet;

Stability = (simulatior3D.simAuxResults.Xcp - simulatior3D.simAuxResults.CM)./Rocket.maxDiameter;
% Cut values near apogee, when the rocket's speed is below 50 m/s
% (arbitrary, value chosen from analysis)
Stability = Stability(1:length(burnState));

Min_static_margin_during_boost = min(Stability);

Max_acceleration_g = max(diff(flightState(:,6))./diff(flightTime)) / g0;

Max_speed_ft = max(flightState(:,6)) * m_to_feet;

Predicted_apogee_ft = flightState(end,3) * m_to_feet;

Predicted_Flight_Data_and_Analysis = [Liftoff_thrust_to_weight_ratio;
    Launch_rail_departure_velocity_ft;
    Min_static_margin_during_boost;
    Max_acceleration_g;
    Max_speed_ft;
    Predicted_apogee_ft];

display(['Liftoff_thrust_to_weight_ratio = ' num2str(Liftoff_thrust_to_weight_ratio)]);
display(['Launch_rail_departure_velocity_ft = ' num2str(Launch_rail_departure_velocity_ft)]);
display(['Min_static_margin_during_boost = ' num2str(Min_static_margin_during_boost)]);
display(['Max_acceleration_g = ' num2str(Max_acceleration_g)]);
display(['Max_speed_ft = ' num2str(Max_speed_ft)]);
display(['Predicted_apogee = ' num2str(Predicted_apogee_ft)]);

%% End

warning('on','all')