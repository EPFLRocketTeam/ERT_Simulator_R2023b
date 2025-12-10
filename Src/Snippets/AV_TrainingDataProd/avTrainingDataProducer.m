%% Description

%       Utilisation:
% Normalement, simplement lancer le programme. Il produit ensuite un
% fichier.csv.
% Le fichier comporte 15 colonnes:

    % colonne 1: temps de la mesure (s)
    % colonnes 2-4: Déplacement en x, y, z (m)
    % colonnes 5-7: Vitesse en x, y, z (m/s)
    % colonnes 8-10: Accélération en x, y, z (m/s^2)
    % 
    % Les trois sont dans le référentiel du sol.
    % 
    % colonnes 11-13: rotation angulaire en x, y, z (rad/s). Dans le ref de
    % la fusée.
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
rocket = rocketReader('Nordend_EUROC.txt');
environment = environnementReader('Environment/Environnement_Definition_EuRoC.txt');
simOutputs = SimOutputReader('Simulation/Simulation_outputs.txt');

simObj = Simulator3D(rocket, environment, simOutputs);

%% ------------------------------------------------------------------------
% 6DOF Rail Simulation
%--------------------------------------------------------------------------
simObj.Rocket.motor_state = 'on';
[timeRail, stateRail] = simObj.RailSim();

display(['Launch rail departure velocity : ' num2str(stateRail(end,2))]);
display(['Launch rail departure time : ' num2str(timeRail(end))]);

%% ------------------------------------------------------------------------
% 6DOF Flight Simulation
%--------------------------------------------------------------------------

[timeBurn, stateBurn, timeBurnEvents, stateBurnEvents, indexBurnEvents] = ...
    simObj.FlightSim([timeRail(end) simObj.Rocket.Burn_Time(end)], stateRail(end, 2));

% simulatior3D.Rocket.coneMode = 'off';

[timeCoast, stateCoast, timeCoastEvents, stateCoastEvents, indexCoastEvents] = ...
    simObj.FlightSim([timeBurn(end) 40], stateBurn(end, 1:3)', ...
    stateBurn(end, 4:6)', stateBurn(end, 7:10)', stateBurn(end, 11:13)');

timeAscent = [timeBurn; timeCoast(2:end)];
stateAscent = [stateBurn; stateCoast(2:end, :)];

timeUpToBurnout = [timeRail; timeAscent];
stateUpToBurnout = [stateRail; stateAscent(:,3) stateAscent(:,6)];

display(['Apogee AGL : ' num2str(stateAscent(end,3))]);
display(['Apogee AGL @t = ' num2str(timeAscent(end))]);
[maxSpeed, speedIndex] = max(stateAscent(:,6));
display(['Max speed : ' num2str(maxSpeed)]);
display(['Max speed @t = ' num2str(timeAscent(speedIndex))]);
[~, speedOfSound, ~, airDensity, kinematicViscosity] = ...
    atmosphere(stateAscent(speedIndex,3), environment);
dragForce = 0.5 * simObj.simAuxResults.Cd(speedIndex) * airDensity * ...
    pi * rocket.maxDiameter^2 / 4 * maxSpeed^2;
display(['Max drag force = ' num2str(dragForce)]);
display(['Max drag force along rocket axis = ' ...
    num2str(dragForce * cos(simObj.simAuxResults.Delta(speedIndex)))]);
dragCoefficientAb = drag_shuriken(rocket, 0, ...
    simObj.simAuxResults.Delta(speedIndex), maxSpeed, kinematicViscosity);
dragForceAb = 0.5 * dragCoefficientAb * airDensity * ...
    pi * rocket.maxDiameter^2 / 4 * maxSpeed^2;
display(['AB drag force at max speed = ' num2str(dragForceAb)]);
display(['Max Mach number : ' num2str(maxSpeed / speedOfSound)]);
[maxAcceleration, accelIndex] = max(diff(stateUpToBurnout(:,2)) ./ diff(timeUpToBurnout));
display(['Max acceleration : ' num2str(maxAcceleration)]);
display(['Max g : ' num2str(maxAcceleration / 9.81)]);
display(['Max g @t = ' num2str(timeUpToBurnout(accelIndex))]);


%% ------------------------------------------------------------------------
% 3DOF Recovery Drogue
%--------------------------------------------------------------------------

[timeDrogue, stateDrogue, timeDrogueEvents, stateDrogueEvents, indexDrogueEvents] = ...
    simObj.DrogueParaSim(timeAscent(end), stateAscent(end,1:3)', stateAscent(end, 4:6)');

%% ------------------------------------------------------------------------
% 3DOF Recovery Main
%--------------------------------------------------------------------------

[timeMain, stateMain, timeMainEvents, stateMainEvents, indexMainEvents] = ...
    simObj.MainParaSim(timeDrogue(end), stateDrogue(end,1:3)', stateDrogue(end, 4:6)');

timeDescent = [timeDrogue; timeMain];
stateDescent = [stateDrogue; stateMain];

%% ------------------------------------------------------------------------
% Flight phase vectors
%--------------------------------------------------------------------------

% Vector to track flight phase. 1 for rail phase,
% 2 for powered ascent, ...
eventVectorRail = ones(length(timeRail), 1);
eventVectorBurn = ones(length(timeBurn), 1) * 2;
eventVectorCoast = ones(length(timeCoast(2:end)), 1) * 22;
eventVectorAscent = [eventVectorBurn; eventVectorCoast];

eventVectorDrogue = ones(length(timeDrogue), 1) * 3;
eventVectorMain = ones(length(timeMain), 1) * 4;
eventVectorDescent = [eventVectorDrogue; eventVectorMain];

%% ------------------------------------------------------------------------
% Calculate angular velocities from quaternions
%--------------------------------------------------------------------------
% Compute quaternion derivatives
timeStep = timeAscent(2:end) - timeAscent(1:end-1);
dq = (stateAscent(2:end, 7:10) - stateAscent(1:end-1, 7:10)) ./ timeStep;

% Convert quaternion derivatives to angular velocity
angularVelocity = zeros(size(dq, 1), 3);
for i = 1:size(dq, 1)
    angularVelocity(i, :) = 2 * quatRateMatrix(stateAscent(i+1, 7:10)) * dq(i, :)';
end
angularVelocity = [0 0 0; angularVelocity];

%% ------------------------------------------------------------------------
% Calculate accelerations
%--------------------------------------------------------------------------
dtRail = timeRail(2:end) - timeRail(1:end-1);
dtAscent = timeAscent(2:end) - timeAscent(1:end-1);
dtDescent = timeDescent(2:end) - timeDescent(1:end-1);

stateDotRail = [(stateRail(2:end,:) - stateRail(1:end-1,:)) ./ dtRail; 
                (stateRail(end,:) - stateRail(end-1,:)) ./ dtRail(end)];
stateDotAscent = [0 0 stateDotRail(end,1) 0 0 stateDotRail(end,2); 
                  (stateAscent(2:end,1:6) - stateAscent(1:end-1, 1:6)) ./ dtAscent];
stateDotDescent = [stateDotAscent(end,:); 
                   (stateDescent(2:end,1:6) - stateDescent(1:end-1, 1:6)) ./ dtDescent];

%% ------------------------------------------------------------------------
% Calculate atmospheric pressure
%--------------------------------------------------------------------------
pressureRail = zeros(size(stateRail,1),1);
pressureAscent = zeros(size(stateAscent,1),1);
pressureDescent = zeros(size(stateDescent,1),1);

for i = 1:size(stateRail,1)
    [~, ~, pressureRail(i), ~, ~] = ...
        atmosphere(stateRail(i,1) + environment.startAltitude, environment);
end

for i = 1:size(stateAscent,1)
    [~, ~, pressureAscent(i), ~, ~] = ...
        atmosphere(stateAscent(i,3) + environment.startAltitude, environment);
end

for i = 1:size(stateDescent,1)
    [~, ~, pressureDescent(i), ~, ~] = ...
        atmosphere(stateDescent(i,3) + environment.startAltitude, environment);
end

%% ------------------------------------------------------------------------
% Writing results
%--------------------------------------------------------------------------

zeroCol2Rail = zeros(length(timeRail),2);
zeroCol3Rail = zeros(length(timeRail), 3);
zeroCol3Descent = zeros(length(timeDescent), 3);

% Format ascent results
ascentResults = [timeRail zeroCol2Rail stateRail(:,1) zeroCol2Rail ...
                 stateRail(:,2) zeroCol2Rail stateDotRail(:,2) ...
                 zeroCol3Rail pressureRail eventVectorRail; ...
                 timeAscent stateAscent(:,1:6) stateDotAscent(:,4:6) ...
                 angularVelocity pressureAscent eventVectorAscent];

% Format descent results
descentResults = [timeDescent stateDescent stateDotDescent(:,4:6) ...
                  zeroCol3Descent pressureDescent eventVectorDescent];

simResults = [ascentResults; descentResults];

writematrix(simResults, "./AV_testData2.csv")

%% ------------------------------------------------------------------------
% Functions
%--------------------------------------------------------------------------

function W = quatRateMatrix(q)
    % Matrix for converting to angular velocity in rocket reference frame
    % See: https://www.astro.rug.nl/software/kapteyn-beta/_downloads/attitude.pdf
    W = [-q(2) q(1) q(4) -q(3); ...
         -q(3) -q(4) q(1) q(2); ...
         -q(4) q(3) -q(2) q(1)];
end