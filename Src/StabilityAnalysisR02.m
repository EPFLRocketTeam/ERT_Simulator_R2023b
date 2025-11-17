%% Stability analysis
% https://apogeerockets.com/education/downloads/Newsletter197.pdf
% https://www.apogeerockets.com/education/downloads/Newsletter195.pdf
% https://www.apogeerockets.com/education/downloads/Newsletter193.pdf
% https://www.apogeerockets.com/downloads/barrowman_report.pdf (pas utilisé
% directement)
% Formules tirées des documents ci-dessus et des fichiers Main_3D et
% Simulator3D.
function [results] = StabilityAnalysisR02()

clear all; close all; clc;
addpath(genpath('.\Declarations'),...
        genpath('.\Functions'),...
        genpath('.\Snippets'),...
        genpath('.\Simulator_3D'),...
        genpath('.\Archives\Models'));
    
% Initialize results structure
results = struct();

% Rocket Definition
Rocket = rocketReader('WH_test.txt');
Environment = environnementReader('Environment\Environnement_Definition_Euroc.txt');
SimOutputs = SimOutputReader('Simulation\Simulation_outputs.txt');
warning('off','all')

%% ========================================================================
% Nominal case
% =========================================================================

SimObj = Simulator3D(Rocket, Environment, SimOutputs);

% -------------------------------------------------------------------------
% 6DOF Rail Simulation
%--------------------------------------------------------------------------

[T1_nom, S1_nom] = SimObj.RailSim();

% -------------------------------------------------------------------------
% 6DOF Flight Simulation
%--------------------------------------------------------------------------

[T2_1_nom, S2_1_nom, ~, ~, ~] = SimObj.FlightSim([T1_nom(end) SimObj.Rocket.Burn_Time(end)], S1_nom(end, 2));

[T2_2_nom, S2_2_nom, ~, ~, ~] = SimObj.FlightSim([T2_1_nom(end) 40], S2_1_nom(end, 1:3)', S2_1_nom(end, 4:6)', S2_1_nom(end, 7:10)', S2_1_nom(end, 11:13)');

T2_nom = [T2_1_nom; T2_2_nom(2:end)];
S2_nom = [S2_1_nom; S2_2_nom(2:end, :)];

% -------------------------------------------------------------------------
% Results - Nominal case
% -------------------------------------------------------------------------

% Speed off rail
V_nom = S1_nom(end, 2);
% Local speed of sound and density of air
[~,a_nom,~,rho_nom] = stdAtmos(Environment.Start_Altitude + S2_nom(1, 3), Environment);
% Mach number
M_nom = V_nom / a_nom;
alpha_nom = 0;
theta_nom = 0;
[Calpha_nom, CP_nom] = barrowmanLift(Rocket, alpha_nom, M_nom, theta_nom);

CNa2A_nom = 0;
W_nom = SimObj.SimAuxResults.CM(1);
for i = 1:length(Calpha_nom)
    CNa2A_nom = CNa2A_nom + Calpha_nom(i) * (CP_nom(i) - W_nom)^2;
end
d_nom = max(Rocket.diameters);
Ar_nom = pi/4*d_nom^2;

C2A_nom = rho_nom * V_nom * Ar_nom / 2 * CNa2A_nom;

[~,dMdt_nom] = Mass_Non_Lin(T1_nom(end), Rocket);
Lne_nom = Rocket.stage_z(end);

C2R_nom = dMdt_nom * (Lne_nom - W_nom)^2;

% C2A Aerodynamic Damping Moment Coefficient
% C2R Propulsive Damping Moment Coefficient
% C2 Damping Moment Coefficient
C2_nom = C2A_nom + C2R_nom;

CNa_nom = sum(Calpha_nom);
P_nom = SimObj.SimAuxResults.Xcp(1);

C1_nom = rho_nom / 2 * V_nom^2 * Ar_nom * CNa_nom * (P_nom - W_nom);

Il_nom = SimObj.SimAuxResults.Il(1);

% Damping ratio
epsilon_nom = C2_nom / (2 * sqrt(C1_nom * Il_nom));

% Display Nominal Case results
display('=============== Nominal case');
display(['Speed - Nominal case : ' num2str(norm(V_nom))]);
display(['CN_alpha - Nominal case : ' num2str(Calpha_nom(end))]);
display(['Stability - Nominal case : ' num2str((P_nom-W_nom)/d_nom) ]);
display(['CG : ' num2str(W_nom) 'm from nose tip']);
display(['CP : ' num2str(P_nom) 'm from nose tip']);
display(['Damping ratio - Nominal case : ' num2str(epsilon_nom)]);

% Store Nominal Case results
results.V_nom = norm(V_nom);
results.Calpha_fin_nom = Calpha_nom(end);
results.Stability_nom = (P_nom - W_nom) / d_nom;
results.epsilon_nom = epsilon_nom;
results.W_nom = W_nom;
results.P_nom = P_nom;

%% ========================================================================
% Max speed case
% =========================================================================

[~,index_max] = max(S2_nom(:,6));

% Max speed
X_max = S2_nom(index_max, 1:3);
V_max = S2_nom(index_max, 4:6);
% Local speed of sound and density of air
[~,a_max,~,rho_max] = stdAtmos(Environment.Start_Altitude + S2_nom(index_max, 3), Environment);
% Mach number
M_max = norm(V_max) / a_max;

C_max = quat2rotmat(normalizeVect(S2_nom(index_max, 7:10)'));

% Roll Axis
RA_max = C_max*[0,0,1]';
% Wind as computed by windmodel
Vcm_max = V_max - windModel(T2_nom(index_max), Environment.Turb_I,Environment.V_inf*Environment.V_dir,Environment.Turb_model,X_max(3))';
alpha_max = atan2(norm(cross(RA_max, Vcm_max)), dot(RA_max, Vcm_max));
angle_max = rot2anglemat(C_max);
theta_max = angle_max(3);

[Calpha_max, CP_max] = barrowmanLift(Rocket, alpha_max, M_max, theta_max);

CNa2A_max = 0;
W_max = SimObj.SimAuxResults.CM(index_max);
for i = 1:length(Calpha_max)
    CNa2A_max = CNa2A_max + Calpha_max(i) * (CP_max(i) - W_max)^2;
end
d_max = max(Rocket.diameters);
Ar_max = pi/4*d_max^2;

C2A_max = rho_max * norm(V_max) * Ar_max / 2 * CNa2A_max;
[~,dMdt_max] = Mass_Non_Lin(T2_nom(index_max), Rocket);
Lne_max = Rocket.stage_z(end);

C2R_max = dMdt_max * (Lne_max - W_max)^2;

% C2A Aerodynamic Damping Moment Coefficient
% C2R Propulsive Damping Moment Coefficient
% C2 Damping Moment Coefficient
C2_max = C2A_max + C2R_max;

CNa_max = sum(Calpha_max);
P_max = SimObj.SimAuxResults.Xcp(index_max);

C1_max = rho_max / 2 * norm(V_max)^2 * Ar_max * CNa_max * (P_max - W_max);

Il_max = SimObj.SimAuxResults.Il(index_max);

% Damping ratio
epsilon_max = C2_max / (2 * sqrt(C1_max * Il_max));

% Display Max Speed results
display('=============== Max speed case');
display(['Speed - Max speed case : ' num2str(norm(V_max))]);
display(['CN_alpha - Max speed case : ' num2str(Calpha_max(end))]);
display(['Stability - Max speed case : ' num2str((P_max-W_max)/d_max) ]);
display(['CG : ' num2str(W_max) 'm from nose tip']);
display(['CP : ' num2str(P_max) 'm from nose tip']);
display(['Damping ratio - Max speed case : ' num2str(epsilon_max)]);

% Store Max Speed results
results.V_max = norm(V_max);
results.Calpha_fin_max = Calpha_max(end);
results.Stability_max = (P_max - W_max) / d_max;
results.epsilon_max = epsilon_max;
results.W_max = W_max;
results.P_max = P_max;
results.Apogee_nom = S2_nom(end, 3);

%% ========================================================================
% Worst case (Rail Exit)
% =========================================================================

% ROCKET CHANGES
% Copy Rocket
Rocket_wc_rail = Rocket;
Rocket_wc_rail.rocket_cm = Rocket_wc_rail.rocket_cm * 1.05;
Rocket_wc_rail.rocket_I = Rocket_wc_rail.rocket_I * 1.15;
% Speed off rail
V_wc_rail = 20; % Initial speed off rail for worst case

SimObj_wc_rail = Simulator3D(Rocket_wc_rail, Environment, SimOutputs);

% -------------------------------------------------------------------------
% 6DOF Rail Simulation
%--------------------------------------------------------------------------

[T1_wc_rail, S1_wc_rail] = SimObj_wc_rail.RailSim();

% -------------------------------------------------------------------------
% 6DOF Flight Simulation
%--------------------------------------------------------------------------

[T2_1_wc_rail, S2_1_wc_rail, ~, ~, ~] = SimObj_wc_rail.FlightSim([T1_wc_rail(end) SimObj_wc_rail.Rocket.Burn_Time(end)], V_wc_rail);

[T2_2_wc_rail, S2_2_wc_rail, ~, ~, ~] = SimObj_wc_rail.FlightSim([T2_1_wc_rail(end) 40], S2_1_wc_rail(end, 1:3)', S2_1_wc_rail(end, 4:6)', S2_1_wc_rail(end, 7:10)', S2_1_wc_rail(end, 11:13)');

T2_wc_rail = [T2_1_wc_rail; T2_2_wc_rail(2:end)];
S2_wc_rail = [S2_1_wc_rail; S2_2_wc_rail(2:end, :)];

% -------------------------------------------------------------------------
% Results - Worst case (Rail Exit)
% -------------------------------------------------------------------------

% Local speed of sound and density of air
[~,a_wc_rail,~,rho_wc_rail_initial] = stdAtmos(Environment.Start_Altitude + S2_wc_rail(1, 3), Environment);
% CHANGE DENSITY
rho_wc_rail = rho_wc_rail_initial * 0.99;
% Mach number
M_wc_rail = V_wc_rail / a_wc_rail;
alpha_wc_rail = 0;
theta_wc_rail = 0;
[Calpha_wc_rail, CP_wc_rail] = barrowmanLift(Rocket_wc_rail, alpha_wc_rail, M_wc_rail, theta_wc_rail);
% CHANGE CN_alpha FOR THE FINS
Calpha_wc_rail(end) = Calpha_wc_rail(end)*0.95;

CNa2A_wc_rail = 0;
W_wc_rail = SimObj_wc_rail.SimAuxResults.CM(1);
for i = 1:length(Calpha_wc_rail)
    CNa2A_wc_rail = CNa2A_wc_rail + Calpha_wc_rail(i) * (CP_wc_rail(i) - W_wc_rail)^2;
end
d_wc_rail = max(Rocket_wc_rail.diameters);
Ar_wc_rail = pi/4*d_wc_rail^2;

C2A_wc_rail = rho_wc_rail * V_wc_rail * Ar_wc_rail / 2 * CNa2A_wc_rail;

[~,dMdt_wc_rail] = Mass_Non_Lin(T1_wc_rail(end), Rocket_wc_rail);
Lne_wc_rail = Rocket_wc_rail.stage_z(end);

C2R_wc_rail = dMdt_wc_rail * (Lne_wc_rail - W_wc_rail)^2;

% C2A Aerodynamic Damping Moment Coefficient
% C2R Propulsive Damping Moment Coefficient
% C2 Damping Moment Coefficient
C2_wc_rail = C2A_wc_rail + C2R_wc_rail;

CNa_wc_rail = sum(Calpha_wc_rail);
P_wc_rail = SimObj_wc_rail.SimAuxResults.Xcp(1);

C1_wc_rail = rho_wc_rail / 2 * V_wc_rail^2 * Ar_wc_rail * CNa_wc_rail * (P_wc_rail - W_wc_rail);

Il_wc_rail = SimObj_wc_rail.SimAuxResults.Il(1);

% Damping ratio
epsilon_wc_rail = C2_wc_rail / (2 * sqrt(C1_wc_rail * Il_wc_rail));

% Display Worst Case (Rail Exit) results
display('=============== Worst case');
display(['Speed - Worst case : ' num2str(norm(V_wc_rail))]);
display(['CN_alpha - Worst case : ' num2str(Calpha_wc_rail(end))]);
display(['Stability - Worst case : ' num2str((P_wc_rail-W_wc_rail)/d_wc_rail) ]);
display(['CG : ' num2str(W_wc_rail) 'm from nose tip']);
display(['CP : ' num2str(P_wc_rail) 'm from nose tip']);
display(['Damping ratio - Worst case : ' num2str(epsilon_wc_rail)]);

% Store Worst Case (Rail Exit) results
results.V_wc_rail = norm(V_wc_rail);
results.Calpha_fin_wc_rail = Calpha_wc_rail(end);
results.Stability_wc_rail = (P_wc_rail - W_wc_rail) / d_wc_rail;
results.epsilon_wc_rail = epsilon_wc_rail;
results.W_wc_rail = W_wc_rail;
results.P_wc_rail = P_wc_rail;

%% ========================================================================
% Worst case Max speed
% =========================================================================

[~,index_wc_max] = max(S2_wc_rail(:,6));

% Max speed
X_wc_max = S2_wc_rail(index_wc_max, 1:3);
V_wc_max = S2_wc_rail(index_wc_max, 4:6);
% Local speed of sound and density of air
[~,a_wc_max,~,rho_wc_max_initial] = stdAtmos(Environment.Start_Altitude + S2_wc_rail(index_wc_max, 3), Environment);
% CHANGE DENSITY
rho_wc_max = rho_wc_max_initial * 0.85;
% Mach number
M_wc_max = norm(V_wc_max) / a_wc_max;

C_wc_max = quat2rotmat(normalizeVect(S2_wc_rail(index_wc_max, 7:10)'));

% Roll Axis
RA_wc_max = C_wc_max*[0,0,1]';
% Wind as computed by windmodel
Vcm_wc_max = V_wc_max - windModel(T2_wc_rail(index_wc_max), Environment.Turb_I,Environment.V_inf*Environment.V_dir,Environment.Turb_model,X_wc_max(3))';
alpha_wc_max = atan2(norm(cross(RA_wc_max, Vcm_wc_max)), dot(RA_wc_max, Vcm_wc_max));
angle_wc_max = rot2anglemat(C_wc_max);
theta_wc_max = angle_wc_max(3);

[Calpha_wc_max, CP_wc_max] = barrowmanLift(SimObj_wc_rail.Rocket, alpha_wc_max, M_wc_max, theta_wc_max);
% CHANGE CN_alpha FOR THE FINS
Calpha_wc_max(end) = Calpha_wc_max(end)*0.95;

CNa2A_wc_max = 0;
W_wc_max = SimObj_wc_rail.SimAuxResults.CM(index_wc_max);
for i = 1:length(Calpha_wc_max)
    CNa2A_wc_max = CNa2A_wc_max + Calpha_wc_max(i) * (CP_wc_max(i) - W_wc_max)^2;
end
d_wc_max = max(SimObj_wc_rail.Rocket.diameters);
Ar_wc_max = pi/4*d_wc_max^2;

C2A_wc_max = rho_wc_max * norm(V_wc_max) * Ar_wc_max / 2 * CNa2A_wc_max;

[~,dMdt_wc_max] = Mass_Non_Lin(T2_wc_rail(index_wc_max), SimObj_wc_rail.Rocket);
Lne_wc_max = SimObj_wc_rail.Rocket.stage_z(end);

C2R_wc_max = dMdt_wc_max * (Lne_wc_max - W_wc_max)^2;

% C2A Aerodynamic Damping Moment Coefficient
% C2R Propulsive Damping Moment Coefficient
% C2 Damping Moment Coefficient
C2_wc_max = C2A_wc_max + C2R_wc_max;

CNa_wc_max = sum(Calpha_wc_max);
P_wc_max = SimObj_wc_rail.SimAuxResults.Xcp(index_wc_max);

C1_wc_max = rho_wc_max / 2 * norm(V_wc_max)^2 * Ar_wc_max * CNa_wc_max * (P_wc_max - W_wc_max);

Il_wc_max = SimObj_wc_rail.SimAuxResults.Il(index_wc_max);

% Damping ratio
epsilon_wc_max = C2_wc_max / (2 * sqrt(C1_wc_max * Il_wc_max));

% Display Worst Case Max Speed results
display('=============== Worst case Max speed');
display(['Speed - Worst case Max speed : ' num2str(norm(V_wc_max))]);
display(['CN_alpha - Worst case Max speed : ' num2str(Calpha_wc_max(end))]);
display(['Stability - Worst case Max speed : ' num2str((P_wc_max-W_wc_max)/d_wc_max) ]);
display(['CG : ' num2str(W_wc_max) 'm from nose tip']);
display(['CP : ' num2str(P_wc_max) 'm from nose tip']);
display(['Damping ratio - Worst case Max speed : ' num2str(epsilon_wc_max)]);

% Store Worst Case Max Speed results
results.V_wc_max = norm(V_wc_max);
results.Calpha_fin_wc_max = Calpha_wc_max(end);
results.Stability_wc_max = (P_wc_max - W_wc_max) / d_wc_max;
results.epsilon_wc_max = epsilon_wc_max;
results.W_wc_max = W_wc_max;
results.P_wc_max = P_wc_max;
results.Apogee_wc = S2_wc_rail(end, 3);

%% ========================================================================
% Extra values
% =========================================================================
% Nominal
d_common = max(Rocket.diameters);

Stability_nom_full = (SimObj.SimAuxResults.Xcp - SimObj.SimAuxResults.CM)./d_common;
% Cut values near apogee, when the rocket's speed is below 50 m/s
% (arbitrary, value chosen from analysis)
Stability_nom_cut = Stability_nom_full(1:length(S2_1_nom) + find(S2_2_nom(:,6) < 50,1));
results.Min_Stability_nom = min(Stability_nom_cut);
results.Max_Stability_nom = max(Stability_nom_full);

% Worst Case
Stability_wc_full = (SimObj_wc_rail.SimAuxResults.Xcp - SimObj_wc_rail.SimAuxResults.CM)./d_common;
Stability_wc_cut = Stability_wc_full(1:length(S2_1_wc_rail) + find(S2_2_wc_rail(:,6) < 50,1));
results.Min_Stability_wc = min(Stability_wc_cut);
results.Max_Stability_wc = max(Stability_wc_full);

% Display Apogee and Min/Max Static Margin
display(['Apogee (Nominal) : ' num2str(results.Apogee_nom)]);
display(['Min Static Margin (Nominal, cut) : ' num2str(results.Min_Stability_nom)]);
display(['Max Static Margin (Nominal) : ' num2str(results.Max_Stability_nom)]);
display(['Apogee (Worst Case) : ' num2str(results.Apogee_wc)]);
display(['Min Static Margin (Worst Case, cut) : ' num2str(results.Min_Stability_wc)]);
display(['Max Static Margin (Worst Case) : ' num2str(results.Max_Stability_wc)]);

warning('on','all')

end