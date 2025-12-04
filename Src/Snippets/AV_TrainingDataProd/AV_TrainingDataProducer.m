%% Description

%       Utilisation:
% Normalement, simplement lancer le programme. Il produit ensuite un
% fichier.csv.
%Le fichier comporte 15 colonnes:

    % colonne 1: temps de la mesure (s)
    % colonnes 2-4: Déplacement en x, y, z (m)
    % colonnes 5-7: Vitesse en x, y, z (m/s)
    % colonnes 8-10: Accélération en x, y, z (m/s^2)
    % 
    % Les trois sont dans le référentiel du sol.
    % 
    % colonnes 11-13: rotation angulaire en x, y, z (rad/s). Dans le ref de
    % la fusée. (?)
    % colonne 14: pression atmosphérique (Pa)
    % colonne 15: phase de vol: 1 correspond à rail, 2 montée avec moteur
    % allumé, 22 montée après burnout, 3 descente avec drogue, 4 descente 
    % avec main.

%       REMARQUES
% - Le rail est simulé en 1D -> pas de rotation et que vitesse/déplacement 
% selon z (en vrai y a un petit angle de 5° avec la normal du sol, mais 
% c'est ici négligeable).
% - Il faudrait vérifier la conversion de l'évolution des quaternions en
% vitesse angulaire.
% - Les phases 3 et 4 de vol ne prennent pas en compte les rotations de la
% fusée (simulateur 3degrés de liberté), les vitesses angulaires 
% valent donc toutes zéros.
% - Source utile pour les quaternions: 
% https://www.astro.rug.nl/software/kapteyn-beta/_downloads/attitude.pdf

%% Rocket Simulator 3D
% Initialize
close all; clear all; clc;
addpath(genpath('./Declarations'),...
        genpath('./Functions'),...
        genpath('./Snippets'),...
        genpath('./Simulator_3D'),...
        genpath('./Calibration'));
% Rocket Definition
% Rocket = rocketReader('Nordend_EUROC.txt');
% Environment = environnementReader('Environment/Environnement_Definition_Wasserfallen.txt');

Rocket = rocketReader('Nordend_N1332.txt');
Environment = environnementReader('Calibration/Environnement_Definition_EuRoC.txt');

SimOutputs = SimOutputReader('Simulation/Simulation_outputs.txt');

SimObj = Simulator3D(Rocket, Environment, SimOutputs);

%% ------------------------------------------------------------------------
% 6DOF Rail Simulation
%--------------------------------------------------------------------------

[T1, S1] = SimObj.RailSim();

display(['Launch rail departure velocity : ' num2str(S1(end,2))]);
display(['Launch rail departure time : ' num2str(T1(end))]);

%% ------------------------------------------------------------------------
% 6DOF Flight Simulation
%--------------------------------------------------------------------------

[T2_1, S2_1, T2_1E, S2_1E, I2_1E] = SimObj.FlightSim([T1(end) SimObj.Rocket.Burn_Time(end)], S1(end, 2));

%SimObj.Rocket.coneMode = 'off';

[T2_2, S2_2, T2_2E, S2_2E, I2_2E] = SimObj.FlightSim([T2_1(end) 40], S2_1(end, 1:3)', S2_1(end, 4:6)', S2_1(end, 7:10)', S2_1(end, 11:13)');

T2 = [T2_1; T2_2(2:end)];
S2 = [S2_1; S2_2(2:end, :)];

T_1_2 = [T1;T2];
S_1_2 = [S1;S2(:,3) S2(:,6)];

display(['Apogee AGL : ' num2str(S2(end,3))]);
display(['Apogee AGL @t = ' num2str(T2(end))]);
[maxi,index] = max(S2(:,6));
display(['Max speed : ' num2str(maxi)]);
display(['Max speed @t = ' num2str(T2(index))]);
[~,a,~,rho,nu] = atmosphere(S2(index,3),Environment);
Fd = 0.5*SimObj.SimAuxResults.Cd(index)*rho*pi*Rocket.maxDiameter^2/4*maxi^2;
display(['Max drag force = ' num2str(Fd)]);
display(['Max drag force along rocket axis = ' num2str(Fd*cos(SimObj.SimAuxResults.Delta(index)))]);
C_Dab = drag_shuriken(Rocket, 0, SimObj.SimAuxResults.Delta(index), maxi, nu);
F_Dab = 0.5*C_Dab*rho*pi*Rocket.maxDiameter^2/4*maxi^2;
display(['AB drag force at max speed = ' num2str(F_Dab)]);
display(['Max Mach number : ' num2str(maxi/a)]);
[maxi,index] = max(diff(S_1_2(:,2))./diff(T_1_2));
display(['Max acceleration : ' num2str(maxi)]);
display(['Max g : ' num2str(maxi/9.81)]);
display(['Max g @t = ' num2str(T_1_2(index))]);


%% ------------------------------------------------------------------------
% 3DOF Recovery Drogue
%--------------------------------------------------------------------------

[T3, S3, T3E, S3E, I3E] = SimObj.DrogueParaSim(T2(end), S2(end,1:3)', S2(end, 4:6)');

%% ------------------------------------------------------------------------
% 3DOF Recovery Main
%--------------------------------------------------------------------------
% 
[T4, S4, T4E, S4E, I4E] = SimObj.MainParaSim(T3(end), S3(end,1:3)', S3(end, 4:6)');

T_3_4 = [T3; T4];
S_3_4 = [S3; S4];
%% ------------------------------------------------------------------------
% Flight phase vectors
%--------------------------------------------------------------------------

% Vecteur permettant de suivre la phase de vol. 1 pour la phase de rail,
% 2 pour la montée avec moteur allumé, ...
event_vec_rail = ones(length(T1), 1);
event_vec_burn = ones(length(T2_1), 1)*2;
event_vec_burnout_up = ones(length(T2_2(2:end)), 1)*22;
event_vec_2 = [event_vec_burn; event_vec_burnout_up];

event_vec_drogue = ones(length(T3), 1)*3;
event_vec_main = ones(length(T4), 1)*4;
event_vec_3_4 = [event_vec_drogue; event_vec_main];

%% ------------------------------------------------------------------------
% calcul des vitesses angulaires à partir des quaternions.
%--------------------------------------------------------------------------
% Calculer la dérivée des quaternions
dt = T2(2:end)-T2(1:end-1); % dt est le pas de temps entre les échantillons
dq = (S2(2:end, 7:10)-S2(1:end-1, 7:10))./ dt; % différenciation numérique

% Convertir les dérivées des quaternions en vitesse angulaire
angular_velocity = zeros(size(dq, 1), 3); % vecteur pour stocker la vitesse angulaire
for i = 1:size(dq, 1)
    % Formule de la vitesse angulaire en termes de quaternions
    angular_velocity(i, :) = 2 * quatratematrix(S2(i+1, 7:10)) * dq(i, :)';
end
angular_velocity = [0 0 0; angular_velocity];

%% ------------------------------------------------------------------------
% calcul des accélérations
%--------------------------------------------------------------------------
dt1 = T1(2:end)-T1(1:end-1);
dt2 = T2(2:end)-T2(1:end-1);
dt34 = T_3_4(2:end)-T_3_4(1:end-1);

S1dot = [(S1(2:end,:)-S1(1:end-1,:))./dt1; (S1(end,:)-S1(end-1,:))./dt1(end)];
S2dot = [0 0 S1dot(end,1) 0 0 S1dot(end,2); (S2(2:end,1:6)-S2(1:end-1, 1:6))./dt2 ];
S34dot = [S2dot(end,:); (S_3_4(2:end,1:6)-S_3_4(1:end-1, 1:6))./dt34 ];

%% ------------------------------------------------------------------------
% calcul pression atmosphérique
%--------------------------------------------------------------------------
P1 = zeros(size(S1,1),1);
P2 = zeros(size(S2,1),1);
P34 = zeros(size(S_3_4,1),1);

for i = 1:size(S1,1)
    [T,a1,P1(i),rho1,nu1] = atmosphere(S1(i,1)+Environment.Start_Altitude, Environment);
end

for i = 1:size(S2,1)
    [T,a2,P2(i),rho2,nu2] = atmosphere(S2(i,3)+Environment.Start_Altitude, Environment);
end

for i = 1:size(S_3_4,1)
    [T,a34,P34(i),rho34,nu34] = atmosphere(S_3_4(i,3)+Environment.Start_Altitude, Environment);
end

%% ------------------------------------------------------------------------
% writing
%--------------------------------------------------------------------------

align_2_T1 = zeros(length(T1),2);
align_3_T1 = zeros(length(T1), 3);
align_3_T34 = zeros(length(T_3_4), 3);

% Mise en forme résultats de la  montée
up_results = [T1 align_2_T1 S1(:,1) align_2_T1 S1(:,2) align_2_T1 S1dot(:,2) align_3_T1 P1 event_vec_rail; ...
    T2 S2(:,1:6) S2dot(:,4:6) angular_velocity P2 event_vec_2 ];
% Mise en forme résultats de la  descente
down_results = [T_3_4 S_3_4 S34dot(:,4:6) align_3_T34 P34 event_vec_3_4];


sim_results = [up_results; down_results];

writematrix(sim_results, "./AV_testData2.csv")


%% ------------------------------------------------------------------------
% functions
%--------------------------------------------------------------------------

% pas nécessaire
% function Q = quatmatrice(q)
%     Q=[q(1) -q(2) -q(3) -q(4);...
%        q(2) q(1) -q(4) q(3);...
%        q(3) q(4) q(1) -q(2);...
%        q(4) -q(3) q(2) q(1)];
% end 

% matrice pour passer à vitesse angulaire dans ref frame rocket voir
% https://www.astro.rug.nl/software/kapteyn-beta/_downloads/attitude.pdf
function W = quatratematrix(q)  
    W=[-q(2) q(1) q(4) -q(3);...
       -q(3) -q(4) q(1) q(2);...
       -q(4) q(3) -q(2) q(1)];
end 