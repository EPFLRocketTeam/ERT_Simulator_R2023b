%% Stability analysis
% https://apogeerockets.com/education/downloads/Newsletter197.pdf
% https://www.apogeerockets.com/education/downloads/Newsletter195.pdf
% https://www.apogeerockets.com/education/downloads/Newsletter193.pdf
% https://www.apogeerockets.com/downloads/barrowman_report.pdf (pas utilisé
% directement)
% Formules tirées des documents ci-dessus et des fichiers Main_3D et
% Simulator3D.
function [results] = stabilityAnalysisR02()
clear all; close all; clc;
addpath(genpath('.\Declarations'),...
        genpath('.\Functions'),...
        genpath('.\Snippets'),...
        genpath('.\Simulator_3D'),...
        genpath('.\Archives\Models'));
    
% Initialize results structure
results = struct();
% Rocket Definition
rocket = rocketReader('WH_test.txt');
environment = environnementReader('Environment\Environnement_Definition_Euroc.txt');
simOutputs = SimOutputReader('Simulation\Simulation_outputs.txt');
warning('off','all')
errorCount = 0;
%% ========================================================================
% Nominal case
% =========================================================================
simObj = Simulator3D(rocket, environment, simOutputs);
% -------------------------------------------------------------------------
% 6DOF Rail Simulation
%--------------------------------------------------------------------------
[t1Nom, s1Nom] = simObj.RailSim();
% -------------------------------------------------------------------------
% 6DOF Flight Simulation
%--------------------------------------------------------------------------
[t2_1Nom, s2_1Nom, ~, ~, ~] = simObj.FlightSim([t1Nom(end) simObj.Rocket.Burn_Time(end)], s1Nom(end, 2));
[t2_2Nom, s2_2Nom, ~, ~, ~] = simObj.FlightSim([t2_1Nom(end) 40], s2_1Nom(end, 1:3)', s2_1Nom(end, 4:6)', s2_1Nom(end, 7:10)', s2_1Nom(end, 11:13)');
t2Nom = [t2_1Nom; t2_2Nom(2:end)];
s2Nom = [s2_1Nom; s2_2Nom(2:end, :)];
% -------------------------------------------------------------------------
% Results - Nominal case
% -------------------------------------------------------------------------
% Speed off rail
vNom = s1Nom(end, 2);
% Local speed of sound and density of air
[~,aNom,~,rhoNom] = stdAtmos(environment.Start_Altitude + s2Nom(1, 3), environment);
% Mach number
mNom = vNom / aNom;
alphaNom = 0;
thetaNom = 0;
[cAlphaNom, cpNom] = barrowmanLift(rocket, alphaNom, mNom, thetaNom);
cNa2ANom = 0;
wNom = simObj.SimAuxResults.CM(1);
for i = 1:length(cAlphaNom)
    cNa2ANom = cNa2ANom + cAlphaNom(i) * (cpNom(i) - wNom)^2;
end
dNom = max(rocket.stageDiameters);
arNom = pi/4*dNom^2;
c2ANom = rhoNom * vNom * arNom / 2 * cNa2ANom;
[~,dMdtNom] = Mass_Non_Lin(t1Nom(end), rocket);
lneNom = rocket.stagePositions(end);
c2RNom = dMdtNom * (lneNom - wNom)^2;
% c2A Aerodynamic Damping Moment Coefficient
% c2R Propulsive Damping Moment Coefficient
% c2 Damping Moment Coefficient
c2Nom = c2ANom + c2RNom;
cNaNom = sum(cAlphaNom);
pNom = simObj.SimAuxResults.Xcp(1);
c1Nom = rhoNom / 2 * vNom^2 * arNom * cNaNom * (pNom - wNom);
ilNom = simObj.SimAuxResults.Il(1);
% Damping ratio
epsilonNom = c2Nom / (2 * sqrt(c1Nom * ilNom));
% Display Nominal Case results
display('=============== Nominal case');
if norm(vNom)>=20
    status = 'OK';
else
    status = 'ERROR';
    errorCount = errorCount + 1;
end
display(['Speed - Nominal case : ' num2str(norm(vNom)) ' ' status]);
display(['CN_alpha - Nominal case : ' num2str(cAlphaNom(end)) ' ' status]);
if (pNom-wNom)/dNom>=1.5
    if (pNom-wNom)/dNom>6
        status = 'WARNING: Value is high';
    else
        status = 'OK';
    end
elseif (pNom-wNom)/dNom<0
    status = 'ERROR: Negative value';
    errorCount = errorCount + 1;
else
    status = 'ERROR: Value is too small';
    errorCount = errorCount + 1;
end
display(['Stability - Nominal case : ' num2str((pNom-wNom)/dNom)  ' ' status ]);
display(['CG : ' num2str(wNom) 'm from nose tip']);
display(['CP : ' num2str(pNom) 'm from nose tip']);
if epsilonNom>=0.05 && epsilonNom<0.3
    status = 'OK';
else
    status = 'ERROR: Value is out of bounds';
    errorCount = errorCount +1;
end
display(['Damping ratio - Nominal case : ' num2str(epsilonNom)  ' ' status]);
% Store Nominal Case results
results.vNom = norm(vNom);
results.cAlphaFinNom = cAlphaNom(end);
results.stabilityNom = (pNom - wNom) / dNom;
results.epsilonNom = epsilonNom;
results.wNom = wNom;
results.pNom = pNom;
%% ========================================================================
% Max speed case
% =========================================================================
[~,indexMax] = max(s2Nom(:,6));
% Max speed
xMax = s2Nom(indexMax, 1:3);
vMax = s2Nom(indexMax, 4:6);
% Local speed of sound and density of air
[~,aMax,~,rhoMax] = stdAtmos(environment.Start_Altitude + s2Nom(indexMax, 3), environment);
% Mach number
mMax = norm(vMax) / aMax;
cMax = quat2rotmat(normalizeVect(s2Nom(indexMax, 7:10)'));
% Roll Axis
raMax = cMax*[0,0,1]';
% Wind as computed by windmodel
vCmMax = vMax - windModel(t2Nom(indexMax), environment.Turb_I,environment.V_inf*environment.V_dir,environment.Turb_model,xMax(3))';
alphaMax = atan2(norm(cross(raMax, vCmMax)), dot(raMax, vCmMax));
angleMax = rot2anglemat(cMax);
thetaMax = angleMax(3);
[cAlphaMax, cpMax] = barrowmanLift(rocket, alphaMax, mMax, thetaMax);
cNa2AMax = 0;
wMax = simObj.SimAuxResults.CM(indexMax);
for i = 1:length(cAlphaMax)
    cNa2AMax = cNa2AMax + cAlphaMax(i) * (cpMax(i) - wMax)^2;
end
dMax = max(rocket.stageDiameters);
arMax = pi/4*dMax^2;
c2AMax = rhoMax * norm(vMax) * arMax / 2 * cNa2AMax;
[~,dMdtMax] = Mass_Non_Lin(t2Nom(indexMax), rocket);
lneMax = rocket.stagePositions(end);
c2RMax = dMdtMax * (lneMax - wMax)^2;
% c2A Aerodynamic Damping Moment Coefficient
% c2R Propulsive Damping Moment Coefficient
% c2 Damping Moment Coefficient
c2Max = c2AMax + c2RMax;
cNaMax = sum(cAlphaMax);
pMax = simObj.SimAuxResults.Xcp(indexMax);
c1Max = rhoMax / 2 * norm(vMax)^2 * arMax * cNaMax * (pMax - wMax);
ilMax = simObj.SimAuxResults.Il(indexMax);
% Damping ratio
epsilonMax = c2Max / (2 * sqrt(c1Max * ilMax));
% Display Max Speed results
display('=============== Max speed case');
if norm(vMax)>=20
    status = 'OK';
else
    status = 'ERROR';
    errorCount = errorCount + 1;
end
display(['Speed - Max speed case : ' num2str(norm(vMax)) ' ' status]);
display(['CN_alpha - Max speed case : ' num2str(cAlphaMax(end)) ' ' status]);
if (pMax-wMax)/dMax>=1.5
    if (pMax-wMax)/dMax>6
        status = 'WARNING: Value is high';
    else
        status = 'OK';
    end
elseif (pMax-wMax)/dMax<0
    status = 'ERROR: Negative value';
    errorCount = errorCount + 1;
else
    status = 'ERROR: Value is too small';
    errorCount = errorCount + 1;
end
display(['Stability - Max speed case : ' num2str((pMax-wMax)/dMax)  ' ' status]);
display(['CG : ' num2str(wMax) 'm from nose tip']);
display(['CP : ' num2str(pMax) 'm from nose tip']);
if epsilonMax>=0.05 && epsilonMax<0.3
    status = 'OK';
else
    status = 'ERROR: Value is out of bounds';
    errorCount = errorCount +1;
end
display(['Damping ratio - Max speed case : ' num2str(epsilonMax) ' ' status]);
% Store Max Speed results
results.vMax = norm(vMax);
results.cAlphaFinMax = cAlphaMax(end);
results.stabilityMax = (pMax - wMax) / dMax;
results.epsilonMax = epsilonMax;
results.wMax = wMax;
results.pMax = pMax;
results.apogeeNom = s2Nom(end, 3);
%% ========================================================================
% Worst case (Rail Exit)
% =========================================================================
% ROCKET CHANGES
% Copy Rocket
rocketWcRail = rocket;
rocketWcRail.emptyCenterOfMass = rocketWcRail.emptyCenterOfMass * 1.05;
rocketWcRail.emptyInertia = rocketWcRail.emptyInertia * 1.15;
% Speed off rail
vWcRail = 20; % Initial speed off rail for worst case
simObjWcRail = Simulator3D(rocketWcRail, environment, simOutputs);
% -------------------------------------------------------------------------
% 6DOF Rail Simulation
%--------------------------------------------------------------------------
[t1WcRail, s1WcRail] = simObjWcRail.RailSim();
% -------------------------------------------------------------------------
% 6DOF Flight Simulation
%--------------------------------------------------------------------------
[t2_1WcRail, s2_1WcRail, ~, ~, ~] = simObjWcRail.FlightSim([t1WcRail(end) simObjWcRail.Rocket.Burn_Time(end)], vWcRail);
[t2_2WcRail, s2_2WcRail, ~, ~, ~] = simObjWcRail.FlightSim([t2_1WcRail(end) 40], s2_1WcRail(end, 1:3)', s2_1WcRail(end, 4:6)', s2_1WcRail(end, 7:10)', s2_1WcRail(end, 11:13)');
t2WcRail = [t2_1WcRail; t2_2WcRail(2:end)];
s2WcRail = [s2_1WcRail; s2_2WcRail(2:end, :)];
% -------------------------------------------------------------------------
% Results - Worst case (Rail Exit)
% -------------------------------------------------------------------------
% Local speed of sound and density of air
[~,aWcRail,~,rhoWcRailInitial] = stdAtmos(environment.Start_Altitude + s2WcRail(1, 3), environment);
% CHANGE DENSITY
rhoWcRail = rhoWcRailInitial * 0.99;
% Mach number
mWcRail = vWcRail / aWcRail;
alphaWcRail = 0;
thetaWcRail = 0;
[cAlphaWcRail, cpWcRail] = barrowmanLift(rocketWcRail, alphaWcRail, mWcRail, thetaWcRail);
% CHANGE CN_alpha FOR THE FINS
cAlphaWcRail(end) = cAlphaWcRail(end)*0.95;
cNa2AWcRail = 0;
wWcRail = simObjWcRail.SimAuxResults.CM(1);
for i = 1:length(cAlphaWcRail)
    cNa2AWcRail = cNa2AWcRail + cAlphaWcRail(i) * (cpWcRail(i) - wWcRail)^2;
end
dWcRail = max(rocketWcRail.stageDiameters);
arWcRail = pi/4*dWcRail^2;
c2AWcRail = rhoWcRail * vWcRail * arWcRail / 2 * cNa2AWcRail;
[~,dMdtWcRail] = Mass_Non_Lin(t1WcRail(end), rocketWcRail);
lneWcRail = rocketWcRail.stagePositions(end);
c2RWcRail = dMdtWcRail * (lneWcRail - wWcRail)^2;
% c2A Aerodynamic Damping Moment Coefficient
% c2R Propulsive Damping Moment Coefficient
% c2 Damping Moment Coefficient
c2WcRail = c2AWcRail + c2RWcRail;
cNaWcRail = sum(cAlphaWcRail);
pWcRail = simObjWcRail.SimAuxResults.Xcp(1);
c1WcRail = rhoWcRail / 2 * vWcRail^2 * arWcRail * cNaWcRail * (pWcRail - wWcRail);
ilWcRail = simObjWcRail.SimAuxResults.Il(1);
% Damping ratio
epsilonWcRail = c2WcRail / (2 * sqrt(c1WcRail * ilWcRail));
% Display Worst Case (Rail Exit) results
display('=============== Worst case');
if norm(vWcRail)>=20
    status = 'OK';
else
    status = 'ERROR';
    errorCount = errorCount + 1;
end
display(['Speed - Worst case : ' num2str(norm(vWcRail)) ' ' status]);
display(['CN_alpha - Worst case : ' num2str(cAlphaWcRail(end))  ' ' status]);
if (pWcRail-wWcRail)/dWcRail>=1.5
    if (pWcRail-wWcRail)/dWcRail>6
        status = 'WARNING: Value is high';
    else
        status = 'OK';
    end
elseif (pWcRail-wWcRail)/dWcRail<0
    status = 'ERROR: Negative value';
    errorCount = errorCount + 1;
else
    status = 'ERROR: Value is too small';
    errorCount = errorCount + 1;
end
display(['Stability - Worst case : ' num2str((pWcRail-wWcRail)/dWcRail) ' ' status ]);
display(['CG : ' num2str(wWcRail) 'm from nose tip']);
display(['CP : ' num2str(pWcRail) 'm from nose tip']);
if epsilonWcRail>=0.05 && epsilonWcRail<0.3
    status = 'OK';
else
    status = 'ERROR: Value is out of bounds';
    errorCount = errorCount +1;
end
display(['Damping ratio - Worst case : ' num2str(epsilonWcRail) ' ' status]);
% Store Worst Case (Rail Exit) results
results.vWcRail = norm(vWcRail);
results.cAlphaFinWcRail = cAlphaWcRail(end);
results.stabilityWcRail = (pWcRail - wWcRail) / dWcRail;
results.epsilonWcRail = epsilonWcRail;
results.wWcRail = wWcRail;
results.pWcRail = pWcRail;
%% ========================================================================
% Worst case Max speed
% =========================================================================
[~,indexWcMax] = max(s2WcRail(:,6));
% Max speed
xWcMax = s2WcRail(indexWcMax, 1:3);
vWcMax = s2WcRail(indexWcMax, 4:6);
% Local speed of sound and density of air
[~,aWcMax,~,rhoWcMaxInitial] = stdAtmos(environment.Start_Altitude + s2WcRail(indexWcMax, 3), environment);
% CHANGE DENSITY
rhoWcMax = rhoWcMaxInitial * 0.85;
% Mach number
mWcMax = norm(vWcMax) / aWcMax;
cWcMax = quat2rotmat(normalizeVect(s2WcRail(indexWcMax, 7:10)'));
% Roll Axis
raWcMax = cWcMax*[0,0,1]';
% Wind as computed by windmodel
vCmWcMax = vWcMax - windModel(t2WcRail(indexWcMax), environment.Turb_I,environment.V_inf*environment.V_dir,environment.Turb_model,xWcMax(3))';
alphaWcMax = atan2(norm(cross(raWcMax, vCmWcMax)), dot(raWcMax, vCmWcMax));
angleWcMax = rot2anglemat(cWcMax);
thetaWcMax = angleWcMax(3);
[cAlphaWcMax, cpWcMax] = barrowmanLift(simObjWcRail.Rocket, alphaWcMax, mWcMax, thetaWcMax);
% CHANGE CN_alpha FOR THE FINS
cAlphaWcMax(end) = cAlphaWcMax(end)*0.95;
cNa2AWcMax = 0;
wWcMax = simObjWcRail.SimAuxResults.CM(indexWcMax);
for i = 1:length(cAlphaWcMax)
    cNa2AWcMax = cNa2AWcMax + cAlphaWcMax(i) * (cpWcMax(i) - wWcMax)^2;
end
dWcMax = max(simObjWcRail.Rocket.stageDiameters);
arWcMax = pi/4*dWcMax^2;
c2AWcMax = rhoWcMax * norm(vWcMax) * arWcMax / 2 * cNa2AWcMax;
[~,dMdtWcMax] = Mass_Non_Lin(t2WcRail(indexWcMax), simObjWcRail.Rocket);
lneWcMax = simObjWcRail.Rocket.stagePositions(end);
c2RWcMax = dMdtWcMax * (lneWcMax - wWcMax)^2;
% c2A Aerodynamic Damping Moment Coefficient
% c2R Propulsive Damping Moment Coefficient
% c2 Damping Moment Coefficient
c2WcMax = c2AWcMax + c2RWcMax;
cNaWcMax = sum(cAlphaWcMax);
pWcMax = simObjWcRail.SimAuxResults.Xcp(indexWcMax);
c1WcMax = rhoWcMax / 2 * norm(vWcMax)^2 * arWcMax * cNaWcMax * (pWcMax - wWcMax);
ilWcMax = simObjWcRail.SimAuxResults.Il(indexWcMax);
% Damping ratio
epsilonWcMax = c2WcMax / (2 * sqrt(c1WcMax * ilWcMax));
% Display Worst Case Max Speed results
display('=============== Worst case Max speed');
if norm(vWcMax)>=20
    status = 'OK';
else
    status = 'ERROR';
    errorCount = errorCount + 1;
end
display(['Speed - Worst case Max speed : ' num2str(norm(vWcMax)) ' ' status]);
display(['CN_alpha - Worst case Max speed : ' num2str(cAlphaWcMax(end)) ' ' status]);
if (pWcMax-wWcMax)/dWcMax>=1.5
    if (pWcMax-wWcMax)/dWcMax>6
        status = 'WARNING: Value is high';
    else
        status = 'OK';
    end
elseif (pWcMax-wWcMax)/dWcMax<0
    status = 'ERROR: Negative value';
    errorCount = errorCount + 1;
else
    status = 'ERROR: Value is too small';
    errorCount = errorCount + 1;
end
display(['Stability - Worst case Max speed : ' num2str((pWcMax-wWcMax)/dWcMax) ' ' status ]);
display(['CG : ' num2str(wWcMax) 'm from nose tip']);
display(['CP : ' num2str(pWcMax) 'm from nose tip']);
if epsilonWcMax>=0.05 && epsilonWcMax<0.3
    status = 'OK';
else
    status = 'ERROR: Value is out of bounds';
    errorCount = errorCount +1;
end
display(['Damping ratio - Worst case Max speed : ' num2str(epsilonWcMax)]);
% Store Worst Case Max Speed results
results.vWcMax = norm(vWcMax);
results.cAlphaFinWcMax = cAlphaWcMax(end);
results.stabilityWcMax = (pWcMax - wWcMax) / dWcMax;
results.epsilonWcMax = epsilonWcMax;
results.wWcMax = wWcMax;
results.pWcMax = pWcMax;
results.apogeeWc = s2WcRail(end, 3);
%% ========================================================================
% Extra values
% =========================================================================
% Nominal
dCommon = max(rocket.stageDiameters);
stabilityNomFull = (simObj.SimAuxResults.Xcp - simObj.SimAuxResults.CM)./dCommon;
% Cut values near apogee, when the rocket's speed is below 50 m/s
% (arbitrary, value chosen from analysis)
stabilityNomCut = stabilityNomFull(1:length(s2_1Nom) + find(s2_2Nom(:,6) < 50,1));
results.minStabilityNom = min(stabilityNomCut);
results.maxStabilityNom = max(stabilityNomFull);
% Worst Case
stabilityWcFull = (simObjWcRail.SimAuxResults.Xcp - simObjWcRail.SimAuxResults.CM)./dCommon;
stabilityWcCut = stabilityWcFull(1:length(s2_1WcRail) + find(s2_2WcRail(:,6) < 50,1));
results.minStabilityWc = min(stabilityWcCut);
results.maxStabilityWc = max(stabilityWcFull);
% Display Apogee and Min/Max Static Margin
display(['Apogee (Nominal) : ' num2str(results.apogeeNom)]);
display(['Min Static Margin (Nominal, cut) : ' num2str(results.minStabilityNom)]);
display(['Max Static Margin (Nominal) : ' num2str(results.maxStabilityNom)]);
display(['Apogee (Worst Case) : ' num2str(results.apogeeWc)]);
display(['Min Static Margin (Worst Case, cut) : ' num2str(results.minStabilityWc)]);
display(['Max Static Margin (Worst Case) : ' num2str(results.maxStabilityWc)]);
if errorCount == 0
    display('All good !');
else
    display([num2str(errorCount) ' errors']);
end
warning('on','all')
end