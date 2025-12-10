%% Stability analysis
% https://apogeerockets.com/education/downloads/Newsletter197.pdf
% https://www.apogeerockets.com/education/downloads/Newsletter195.pdf
% https://www.apogeerockets.com/education/downloads/Newsletter193.pdf
% https://www.apogeerockets.com/downloads/barrowman_report.pdf (pas utilis�
% directement)
% Formules tir�es des documents ci-dessus et des fichiers Main_3D et
% Simulator3D.

clear all; close all; clc;
addpath(genpath('..\Declarations'),...
        genpath('..\Functions'),...
        genpath('..\Snippets'),...
        genpath('..\Simulator_1D'),...
        genpath('..\Simulator_3D'));
% Rocket Definition
Rocket = rocketReader('WH_test.txt');
Environment = environnementReader('Environment\Environnement_Definition_Euroc.txt');
simulationOutputs = SimOutputReader('Simulation\Simulation_outputs.txt');

warning('off','all')
Error = 0;

%% ========================================================================
% Nominal case
% =========================================================================

simulatior3D = Simulator3D(Rocket, Environment, simulationOutputs);

% -------------------------------------------------------------------------
% 6DOF Rail Simulation
%--------------------------------------------------------------------------

[railTime, railState] = simulatior3D.RailSim();

% -------------------------------------------------------------------------
% 6DOF Flight Simulation
%--------------------------------------------------------------------------

[burnTime, burnState, burnTimeEvents, burnStateEvents, burnEventIndices] = simulatior3D.FlightSim([railTime(end) simulatior3D.Rocket.Burn_Time(end)], railState(end, 2));

[coastTime, coastState, coastTimeEvents, coastStateEvents, coastEventIndices] = simulatior3D.FlightSim([burnTime(end) 40], burnState(end, 1:3)', burnState(end, 4:6)', burnState(end, 7:10)', burnState(end, 11:13)');

flightTime = [burnTime; coastTime(2:end)];
flightState = [burnState; coastState(2:end, :)];

% -------------------------------------------------------------------------
% Results
% -------------------------------------------------------------------------

% Speed off rail
V = railState(end, 2);
% Local speed of sound and density of air
[~,a,~,density] = stdAtmos(Environment.startAltitude + flightState(1, 3), Environment);
% Mach number
M = V / a;
alpha = 0;
theta = 0;
[Calpha, CP] = barrowmanLift(Rocket, alpha, M, theta);

CNa2A = 0;
W = simulatior3D.simAuxResults.CM(1);
for i = 1:length(Calpha)
    CNa2A = CNa2A + Calpha(i) * (CP(i) - W)^2;
end
d = max(Rocket.stageDiameters);
Ar = pi/4*d^2;

C2A = density * V * Ar / 2 * CNa2A;

[~,dMdt] = Mass_Non_Lin(railTime(end), Rocket);
Lne = Rocket.stagePositions(end);

C2R = dMdt * (Lne - W)^2;

% C2A Aerodynamic Damping Moment Coefficient
% C2R Propulsive Damping Moment Coefficient
% C2 Damping Moment Coefficient
C2 = C2A + C2R;

CNa = sum(Calpha);
P = simulatior3D.simAuxResults.Xcp(1);

C1 = density / 2 * V^2 * Ar * CNa * (P - W);

Il = simulatior3D.simAuxResults.Il(1);

% Damping ratio
epsilon = C2 / (2 * sqrt(C1 * Il));

display('=============== Nominal case');
% display(['CG - Nominal case : ' num2str(Rocket.emptyCenterOfMass)]);
% display(['Il initial - Nominal case : ' num2str(Rocket.emptyInertia)]);
% display(['Rho - Nominal case : ' num2str(density)]);

if norm(V)>=20
    Status = 'OK';
else
    Status = 'ERROR';
    Error = Error + 1;
end
display(['Speed - Nominal case : ' num2str(norm(V)) ' ' Status]);

display(['CN_alpha - Nominal case : ' num2str(Calpha(end))]);

if (P-W)/d>=1.5
    if (P-W)/d>6
        Status = 'WARNING: Value is high';
    else
        Status = 'OK';
    end
elseif (P-W)/d<0
    Status = 'ERROR: Negative value';
    Error = Error + 1;
else
    Status = 'ERROR: Value is too small';
    Error = Error + 1;
end
display(['Stability - Nominal case : ' num2str((P-W)/d) ' ' Status]);
display(['CG : ' num2str(W) 'm from nose tip']);
display(['CP : ' num2str(P) 'm from nose tip']);

if epsilon>=0.05 && epsilon<0.3
    Status = 'OK';
else
    Status = 'ERROR: Value is out of bounds';
    Error = Error +1;
end
display(['Damping ratio - Nominal case : ' num2str(epsilon) ' ' Status]);


%% ========================================================================
% Max speed
% =========================================================================

[maxi,index] = max(flightState(:,6));

% Max speed
X = flightState(index, 1:3);
V = flightState(index, 4:6);
% Local speed of sound and density of air
[~,a,~,density] = stdAtmos(Environment.startAltitude + flightState(index, 3), Environment);
% Mach number
M = norm(V) / a;

C = quat2rotmat(normalizeVect(flightState(index, 7:10)'));

RA = C*[0,0,1]'; % Roll Axis
Vcm = V -...
          ... % Wind as computed by windmodel
windModel(flightTime(index), Environment.Turb_I,Environment.V_inf*Environment.V_dir,...
Environment.Turb_model,X(3))';
alpha = atan2(norm(cross(RA, Vcm)), dot(RA, Vcm));
angle = rot2anglemat(C);
theta = angle(3);

[Calpha, CP] = barrowmanLift(Rocket, alpha, M, theta);

CNa2A = 0;
W = simulatior3D.simAuxResults.CM(index);
for i = 1:length(Calpha)
    CNa2A = CNa2A + Calpha(i) * (CP(i) - W)^2;
end
d = max(Rocket.stageDiameters);
Ar = pi/4*d^2;

C2A = density * norm(V) * Ar / 2 * CNa2A;

[~,dMdt] = Mass_Non_Lin(flightTime(index), Rocket);
Lne = Rocket.stagePositions(end);

C2R = dMdt * (Lne - W)^2;

% C2A Aerodynamic Damping Moment Coefficient
% C2R Propulsive Damping Moment Coefficient
% C2 Damping Moment Coefficient
C2 = C2A + C2R;

CNa = sum(Calpha);
P = simulatior3D.simAuxResults.Xcp(index);

C1 = density / 2 * norm(V)^2 * Ar * CNa * (P - W);

Il = simulatior3D.simAuxResults.Il(index);

% Damping ratio
epsilon = C2 / (2 * sqrt(C1 * Il));

display('=============== Max speed case');
% display(['CG - Max speed : ' num2str(Rocket.emptyCenterOfMass)]);
% display(['Il initial - Max speed : ' num2str(Rocket.emptyInertia)]);
% display(['Rho - Max speed : ' num2str(density)]);

if norm(V)>=20
    Status = 'OK';
else
    Status = 'ERROR';
    Error = Error + 1;
end
display(['Speed - Max speed : ' num2str(norm(V)) ' ' Status]);

display(['CN_alpha - Max speed : ' num2str(Calpha(end)) ' ' Status]);

if (P-W)/d>=1.5
    if (P-W)/d>6
        Status = 'WARNING: Value is high';
    else
        Status = 'OK';
    end
elseif (P-W)/d<0
    Status = 'ERROR: Negative value';
    Error = Error + 1;
else
    Status = 'ERROR: Value is too small';
    Error = Error + 1;
end
display(['Stability - Max speed case : ' num2str((P-W)/d) ' ' Status]);
display(['CG : ' num2str(W) 'm from nose tip']);
display(['CP : ' num2str(P) 'm from nose tip']);

if epsilon>=0.05 && epsilon<0.3
    Status = 'OK';
else
    Status = 'ERROR: Value is out of bounds';
    Error = Error +1;
end
display(['Damping ratio - Max speed case : ' num2str(epsilon) ' ' Status]);

%% ========================================================================
% Extra values
% =========================================================================

display(['Apogee : ' num2str(flightState(end, 3))]);

Stability = (simulatior3D.simAuxResults.Xcp - simulatior3D.simAuxResults.CM)./d;
% Cut values near apogee, when the rocket's speed is below 50 m/s
% (arbitrary, value chosen from analysis)
Stability = Stability(1:length(burnState) + find(coastState(:,6) < 50,1));

display(['Min Static Margin : ' num2str(min(Stability))]);
display(['Max Static Margin : ' num2str(max(Stability))]);

%% ========================================================================
% Worst case
% =========================================================================

% ROCKET CHANGES
Rocket.emptyCenterOfMass = Rocket.emptyCenterOfMass * 1.05;
Rocket.emptyInertia = Rocket.emptyInertia * 1.15;
% Speed off rail
V = 20;

simulatior3D = Simulator3D(Rocket, Environment, simulationOutputs);

% -------------------------------------------------------------------------
% 6DOF Rail Simulation
%--------------------------------------------------------------------------

[railTime, railState] = simulatior3D.RailSim();

% -------------------------------------------------------------------------
% 6DOF Flight Simulation
%--------------------------------------------------------------------------

[burnTime, burnState, burnTimeEvents, burnStateEvents, burnEventIndices] = simulatior3D.FlightSim([railTime(end) simulatior3D.Rocket.Burn_Time(end)], V);

[coastTime, coastState, coastTimeEvents, coastStateEvents, coastEventIndices] = simulatior3D.FlightSim([burnTime(end) 40], burnState(end, 1:3)', burnState(end, 4:6)', burnState(end, 7:10)', burnState(end, 11:13)');

flightTime = [burnTime; coastTime(2:end)];
flightState = [burnState; coastState(2:end, :)];

% -------------------------------------------------------------------------
% Results
% -------------------------------------------------------------------------

% Local speed of sound and density of air
[~,a,~,density] = stdAtmos(Environment.startAltitude + flightState(1, 3), Environment);
% CHANGE DENSITY
density = density * 0.99;
% Mach number
M = V / a;
alpha = 0;
theta = 0;
[Calpha, CP] = barrowmanLift(Rocket, alpha, M, theta);
% CHANGE CN_alpha FOR THE FINS
Calpha(end) = Calpha(end)*0.95;

CNa2A = 0;
W = simulatior3D.simAuxResults.CM(1);
for i = 1:length(Calpha)
    CNa2A = CNa2A + Calpha(i) * (CP(i) - W)^2;
end
d = max(Rocket.stageDiameters);
Ar = pi/4*d^2;

C2A = density * V * Ar / 2 * CNa2A;

[~,dMdt] = Mass_Non_Lin(railTime(end), Rocket);
Lne = Rocket.stagePositions(end);

C2R = dMdt * (Lne - W)^2;

% C2A Aerodynamic Damping Moment Coefficient
% C2R Propulsive Damping Moment Coefficient
% C2 Damping Moment Coefficient
C2 = C2A + C2R;

CNa = sum(Calpha);
P = simulatior3D.simAuxResults.Xcp(1);

C1 = density / 2 * V^2 * Ar * CNa * (P - W);

Il = simulatior3D.simAuxResults.Il(1);

% Damping ratio
epsilon = C2 / (2 * sqrt(C1 * Il));

display('=============== Worst case');
% display(['CG - Worst case : ' num2str(Rocket.emptyCenterOfMass)]);
% display(['Il initial - Worst case : ' num2str(Rocket.emptyInertia)]);
% display(['Rho - Worst case : ' num2str(density)]);

if norm(V)>=20
    Status = 'OK';
else
    Status = 'ERROR';
    Error = Error + 1;
end
display(['Speed - Worst case : ' num2str(norm(V)) ' ' Status]);

display(['CN_alpha - Worst case : ' num2str(Calpha(end)) ' ' Status]);

if (P-W)/d>=1.5
    if (P-W)/d>6
        Status = 'WARNING: Value is high';
    else
        Status = 'OK';
    end
elseif (P-W)/d<0
    Status = 'ERROR: Negative value';
    Error = Error + 1;
else
    Status = 'ERROR: Value is too small';
    Error = Error + 1;
end
display(['Stability - Worst case : ' num2str((P-W)/d) ' ' Status]);
display(['CG : ' num2str(W) 'm from nose tip']);
display(['CP : ' num2str(P) 'm from nose tip']);

if epsilon>=0.05 && epsilon<0.3
    Status = 'OK';
else
    Status = 'ERROR: Value is out of bounds';
    Error = Error +1;
end
display(['Damping ratio - Worst case : ' num2str(epsilon) ' ' Status]);


%% ========================================================================
% Worst case Max speed
% =========================================================================

[maxi,index] = max(flightState(:,6));

% Max speed
X = flightState(index, 1:3);
V = flightState(index, 4:6);
% Local speed of sound and density of air
[~,a,~,density] = stdAtmos(Environment.startAltitude + flightState(index, 3), Environment);
% CHANGE DENSITY
density = density * 0.85;
% Mach number
M = norm(V) / a;

C = quat2rotmat(normalizeVect(flightState(index, 7:10)'));

RA = C*[0,0,1]'; % Roll Axis
Vcm = V -...
          ... % Wind as computed by windmodel
windModel(flightTime(index), Environment.Turb_I,Environment.V_inf*Environment.V_dir,...
Environment.Turb_model,X(3))';
alpha = atan2(norm(cross(RA, Vcm)), dot(RA, Vcm));
angle = rot2anglemat(C);
theta = angle(3);

[Calpha, CP] = barrowmanLift(Rocket, alpha, M, theta);
% CHANGE CN_alpha FOR THE FINS
Calpha(end) = Calpha(end)*0.95;

CNa2A = 0;
W = simulatior3D.simAuxResults.CM(index);
for i = 1:length(Calpha)
    CNa2A = CNa2A + Calpha(i) * (CP(i) - W)^2;
end
d = max(Rocket.stageDiameters);
Ar = pi/4*d^2;

C2A = density * norm(V) * Ar / 2 * CNa2A;

[~,dMdt] = Mass_Non_Lin(flightTime(index), Rocket);
Lne = Rocket.stagePositions(end);

C2R = dMdt * (Lne - W)^2;

% C2A Aerodynamic Damping Moment Coefficient
% C2R Propulsive Damping Moment Coefficient
% C2 Damping Moment Coefficient
C2 = C2A + C2R;

CNa = sum(Calpha);
P = simulatior3D.simAuxResults.Xcp(index);

C1 = density / 2 * norm(V)^2 * Ar * CNa * (P - W);

Il = simulatior3D.simAuxResults.Il(index);

% Damping ratio
epsilon = C2 / (2 * sqrt(C1 * Il));

display('=============== Worst case max speed');
% display(['CG - Worst case Max speed : ' num2str(Rocket.emptyCenterOfMass)]);
% display(['Il initial - Worst case Max speed : ' num2str(Rocket.emptyInertia)]);
% display(['Rho - Worst case Max speed : ' num2str(density)]);

if norm(V)>=20
    Status = 'OK';
else
    Status = 'ERROR';
    Error = Error + 1;
end
display(['Speed - Worst case Max speed : ' num2str(norm(V)) ' ' Status]);

display(['CN_alpha - Worst case Max speed : ' num2str(Calpha(end)) ' ' Status]);

if (P-W)/d>=1.5
    if (P-W)/d>6
        Status = 'WARNING: Value is high';
    else
        Status = 'OK';
    end
elseif (P-W)/d<0
    Status = 'ERROR: Negative value';
    Error = Error + 1;
else
    Status = 'ERROR: Value is too small';
    Error = Error + 1;
end
display(['Stability - Worst case Max speed case : ' num2str((P-W)/d) ' ' Status]);
display(['CG : ' num2str(W) 'm from nose tip']);
display(['CP : ' num2str(P) 'm from nose tip']);

if epsilon>=0.05 && epsilon<0.3
    Status = 'OK';
else
    Status = 'ERROR: Value is out of bounds';
    Error = Error +1;
end
display(['Damping ratio - Worst case Max speed case : ' num2str(epsilon) ' ' Status]);

%% ========================================================================
% Extra values
% =========================================================================

display(['Apogee : ' num2str(flightState(end, 3))]);

Stability = (simulatior3D.simAuxResults.Xcp - simulatior3D.simAuxResults.CM)./d;
% Cut values near apogee, when the rocket's speed is below 50 m/s
% (arbitrary, value chosen from analysis)
Stability = Stability(1:length(burnState) + find(coastState(:,6) < 50,1));

display(['Min Static Margin : ' num2str(min(Stability))]);
display(['Max Static Margin : ' num2str(max(Stability))]);

%% End

if Error == 0
    display('All good !');
else
    display([num2str(Error) ' errors']);
end

warning('on','all')