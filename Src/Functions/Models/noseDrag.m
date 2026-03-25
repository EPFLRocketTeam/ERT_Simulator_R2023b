function dragCoefficient = noseDrag(rocket, angleOfAttack, freeStreamVelocity, kinematicViscosity, speedOfSound)
% CALCULATEDRAG - Rocket drag calculation function based on Mandell's book "Topics
% on advanced model Rocketry" (unless otherwise specified). 
%
% LIMITATIONS:
% - Isn't usable for very low speeds (<0.1m/s)
% - Input velocity must be > 0
%
% INPUTS:
% - rocket  : Rocket object
% - angleOfAttack   : angle of attack [rad]
% - freeStreamVelocity    : Free stream velocity [m/s]
% - kinematicViscosity      : Dynamic viscosity [m2/s]
% - speedOfSound       : Speed of sound [m/s]
% OUTPUTS:
% - CD      : Drag coefficient
% REFERENCES:
% - Gordon K. Mandell, Topics in Advanced Model Rocketry, MIT Press, 1973
% - Hassan Arif, Identification and Control of a High Power Rocket, EPFL
% Semester Project Report, Professor Collin Jones, June 2017.

% -------------------------------------------------------------------------
% 0. Divergence 
% -------------------------------------------------------------------------
if freeStreamVelocity < 0.1
    freeStreamVelocity = 0.1;
end
% -------------------------------------------------------------------------
% 1. Geometrical Parameters
% -------------------------------------------------------------------------

maxDiameter = rocket.maxDiameter; % maximum rocket diameter 
maxCrossSectionalArea = rocket.maxCrossSectionArea; % maximum cross-sectional body area
% finChord = rocket.meanFinChord; % fin cord
% finExposedPlanformArea = rocket.exposedFinArea; % Exposed planform fin area
% bodyDiameterAtFinStation = rocket.finBodyDiameter; % body diameter at middle of fin station
% finVirtualPlanformArea = rocket.virtualFinArea; % Virtual fin planform area

% -------------------------------------------------------------------------
% 2. Reynolds Numbers (eq 191, p 458)
% -------------------------------------------------------------------------

% 2.1 Body 
reynoldsNumberBody = rocket.stagePositions(end) * freeStreamVelocity / kinematicViscosity;
reynoldsNumberBodyCritical = 5e5;
% 2.2 Fins
% reynoldsNumberFins = finChord * freeStreamVelocity / kinematicViscosity; 
% reynoldsNumberFinsCritical = 5.14e6;
% Critical values of the Reynolds number are selected as shown in Fig. 51,
% p.464


% -------------------------------------------------------------------------
% 3. Skin Friction Coefficients
% -------------------------------------------------------------------------

% 3.1 Body skin friction
% 3.1.1 turbulent skin friction for a flat plate (eq 102a, p 357)
skinFrictionCoeffTurbulentBody = 0.074 / (reynoldsNumberBody)^0.2;
% 3.1.2 laminar skin friction for a flat plate (eq 102b, p 357)
skinFrictionCoeffLaminarBody = 1.328 / sqrt(reynoldsNumberBody);
% 3.1.3 transitional flow factor for a flat plate (eq 100, p 356)
transitionFactorBody = reynoldsNumberBodyCritical * (skinFrictionCoeffTurbulentBody - skinFrictionCoeffLaminarBody);
% 3.1.4 transitional skin friction for body (eq 101, p 356)
skinFrictionCoeffTurbulentBody = skinFrictionCoeffTurbulentBody - transitionFactorBody / reynoldsNumberBody;

% 3.2 Fin skin friction
% 3.2.1 turbulent skin friction for a flat plate (eq 102a, p 357)
% skinFrictionCoeffTurbulentFins = 0.074 / (reynoldsNumberFins)^0.2;
% 3.2.2 laminar skin friction for a flat plate (eq 102b, p 357)
% skinFrictionCoeffLaminarFins = 1.328 / sqrt(reynoldsNumberFins);
% 3.2.3 transitional flow factor for a flat plate (eq 100, p 356)
% transitionFactorFins = reynoldsNumberFinsCritical * (skinFrictionCoeffTurbulentFins - skinFrictionCoeffLaminarFins); 
% 3.2.4 transitional skin friction for body (eq 101, p 356)
% skinFrictionCoeffTurbulentFins = skinFrictionCoeffTurbulentFins - transitionFactorFins / reynoldsNumberFins;


% -------------------------------------------------------------------------
% 4. 0?? AoA drag
% -------------------------------------------------------------------------

% 4.1 Wetted area ratio
% 4.1.1 ogive cone (eq 171c, p 439)
% 4.1.2 boattail cone (eq 172a, p 441)64 TER
totalWettedAreaRatio = 2 / maxDiameter^2 * sum((rocket.stageDiameters(2:end-1) + rocket.stageDiameters(3:end)) .* ...
    (rocket.stagePositions(3:end) - rocket.stagePositions(2:end-1)) .* ...
    sqrt(1 + ((rocket.stageDiameters(2:end-1) - rocket.stageDiameters(3:end)) ./ 2 ./ ...
    (rocket.stagePositions(3:end) - rocket.stagePositions(2:end-1))).^2));  %Repris la formule du dessus
if strcmp(rocket.coneMode, 'on')
    totalWettedAreaRatio = totalWettedAreaRatio + 2.67*rocket.stagePositions(2)/maxDiameter;
end

if rocket.stagePositions(2)/maxDiameter < 1.5
    display('WARNING: In drag coefficient calculation, ogive cone ratio is out of bounds. Drag estimation cannot be trusted.');
end
 
% 4.2 Body drag
% 4.2.1 (eq 161, p 431)
bodyFrictionDragCoeff = (1 + 60 / (rocket.stagePositions(end) / maxDiameter)^3 + 0.0025 * (rocket.stagePositions(end) / maxDiameter)) * totalWettedAreaRatio; % partially calculated body drag
if reynoldsNumberBody < reynoldsNumberBodyCritical
    bodyFrictionDragCoeff = skinFrictionCoeffLaminarBody * bodyFrictionDragCoeff; % body drag for laminar flow
else
    bodyFrictionDragCoeff = skinFrictionCoeffTurbulentBody * bodyFrictionDragCoeff; % body drag for turbulent flow
end
% 4.2.2 Base drag (eq 162, p 431)
baseDragCoeff = 0.029 * (rocket.stageDiameters(end) / maxDiameter)^3 / sqrt(bodyFrictionDragCoeff);
% 4.2.3 Body drag at 0° AoA (eq 160, p 431)
zeroAoaBodyDragCoeff = bodyFrictionDragCoeff + baseDragCoeff;

% 4.3 Fin drag
% 4.3.1 Fin drag at 0° AoA (eq 159, p 433)
% zeroAoaFinDragCoeff = 2 * (1 + 2 * rocket.finThickness / finChord) * rocket.numFins * finVirtualPlanformArea / maxCrossSectionalArea;
% if reynoldsNumberFins < reynoldsNumberFinsCritical
%     zeroAoaFinDragCoeff = zeroAoaFinDragCoeff * skinFrictionCoeffLaminarFins;
% else
%     zeroAoaFinDragCoeff = zeroAoaFinDragCoeff * skinFrictionCoeffTurbulentFins;
% end
zeroAoaFinDragCoeff = 0;

% 4.4 Launch lug drag mounted on body (eq 119 p 390)
% TODO consider launch lugs mounted near fins with a different drag
% coefficient.
launchLugDragCoeff = rocket.numLaunchLugs * 5.75 * rocket.lugSurfaceArea / maxCrossSectionalArea;
% 4.5 Fin and Body drag at 0 AoA (eq 158, p 430) plus launch lug drag
zeroAoaTotalDragCoeff = zeroAoaBodyDragCoeff + zeroAoaFinDragCoeff + launchLugDragCoeff;
% 4.6 Drag for nosecone failure
if strcmp(rocket.coneMode, 'off')
   zeroAoaTotalDragCoeff = zeroAoaTotalDragCoeff + 1 - bodyFrictionDragCoeff;
end

% -------------------------------------------------------------------------
% 5. Drag at AoA
% -------------------------------------------------------------------------

% 5.1 Body drag at AoA
% 5.1.1 factor tables as seen in Fig. 35 and 36 on p. 405
angleOfAttack = abs(angleOfAttack);
etaTable = [4 6 8 10 12 14 16 18 20 22 24; 0.6 0.63 0.66 0.68 0.71 0.725 0.74 0.75 0.758 0.77 0.775];
deltaKTable = [4 6 8 10 12 14 16 18 20; 0.78 0.86 0.92 0.94 0.96 0.97 0.975 0.98 0.982];
etaK = interp1(etaTable(1,:), etaTable(2,:), rocket.stagePositions(end) / maxDiameter, 'linear', 'extrap');
deltaK = interp1(deltaKTable(1,:), deltaKTable(2,:), rocket.stagePositions(end) / maxDiameter, 'linear', 'extrap');
% 5.1.2 Compute body drag at angle of attack alpha
% 5.1.2.1 x1 as defined by explanations of (eq 140, p 404)
shoulderPosition = rocket.stagePositions(find(diff(rocket.stageDiameters) == 0, 1, 'first')); % TO CHECK - OK
% 5.1.2.2 x0 as in (eq 140, p404)
referenceStationX0 = 0.55 * shoulderPosition + 0.36 * rocket.stagePositions(end); % Purely exp. values
% 5.1.2.3 Section Area at station x0
crossSectionalAreaAtX0 = pi * interp1(rocket.stagePositions, rocket.stageDiameters, referenceStationX0, 'linear')^2 / 4; % Why divided by 4 ?
% 5.1.2.4 Body drag at low AoA (eq 139, p. 404) %% ERROR IN THE BOOK !!!
bodyAoaDragCoeff = 2 * deltaK * crossSectionalAreaAtX0 / maxCrossSectionalArea * angleOfAttack * sin(angleOfAttack);
tempStages = [referenceStationX0, rocket.stagePositions(rocket.stagePositions > referenceStationX0)];
tempDiameters = [interp1(rocket.stagePositions, rocket.stageDiameters, referenceStationX0, 'linear'), rocket.stageDiameters(rocket.stagePositions > referenceStationX0)];
% 5.1.2.4 Body drag at high AoA (eq 142, p. 406)
bodyAoaDragCoeff = bodyAoaDragCoeff + 2 * angleOfAttack^2 * sin(angleOfAttack) / maxCrossSectionalArea * etaK * 1.2 * sum((tempDiameters(1, end-1) + tempDiameters(2:end)) / 2 .* (tempStages(2:end) - tempStages(1:end-1)));

% 5.2 Fin drag at AoA
% 5.2.1 Fin Exposed Surface Coefficient

% TODO: Consider rocket roll for lateral exposed fin surface

% finExposedSurfaceCoeff = 2;
% WHAT IS FESC ? 
% 5.2.2 induced fin drag, similar to (eq 145, p 413)
% inducedFinDragCoeff = 1.2 * angleOfAttack^2 * finVirtualPlanformArea / maxCrossSectionalArea * finExposedSurfaceCoeff;
% 5.2.3 Interference coefficients as estimated by Hassan (eq 34 and 35, p
% 12) based on Mandell Fig. 40 p 416.
% finSpanRatio = bodyDiameterAtFinStation / (2 * rocket.finSpan + bodyDiameterAtFinStation); % Total fin span ratio
% interferenceFactorBodyOnFin = 0.8065 * finSpanRatio^2 + 1.1553 * finSpanRatio; % Interference of body on fin lift
% interferenceFactorFinOnBody = 0.1935 * finSpanRatio^2 + 0.8174 * finSpanRatio + 1; % Interference of fins on body lift % WARNING ONLY VALID FOR GIVEN VALUES
% 5.2.4 Interference Drag Coefficient (eq 146, p 415)
% interferenceDragCoeff = (interferenceFactorBodyOnFin + interferenceFactorFinOnBody - 1) * 3.12 * finExposedPlanformArea / maxCrossSectionalArea * angleOfAttack^2 * finExposedSurfaceCoeff; % Interference drag
% 3.12 is dC_L/dalpha given by Hoerner
finAoaDragCoeff = inducedFinDragCoeff + interferenceDragCoeff;
% 5.3 Total drag at AoA (eq 148, p 417)
totalAoaDragCoeff = bodyAoaDragCoeff + finAoaDragCoeff;

% -------------------------------------------------------------------------
% 7. Drag of tumbeling body (c.f. Openrocket Documentation section 3.5)
% -------------------------------------------------------------------------
% finEfficiency = [0.5, 1, 1.5, 1.41, 1.81, 1.73, 1.9, 1.85];
% tumblingDragCoeffFins = 1.42 * finEfficiency(rocket.numFins);
% tumblingDragCoeffBody = 0.56;
% totalTumblingDragCoeff = (finExposedPlanformArea * tumblingDragCoeffFins + tumblingDragCoeffBody * maxDiameter * (rocket.stagePositions(end) - rocket.stagePositions(2))) / maxCrossSectionalArea;

totalTumblingDragCoeff = 0;

% -------------------------------------------------------------------------
% 6. Subsonic drag coefficient
% -------------------------------------------------------------------------
dragCoefficient = zeroAoaTotalDragCoeff + totalAoaDragCoeff;
% the calculated drag can't be more than the lateral drag of a tumbling
% body so it is cut-off to that value if it is larger. 
if dragCoefficient > totalTumblingDragCoeff
    dragCoefficient = totalTumblingDragCoeff;
end

% -------------------------------------------------------------------------
% 7. Compressible flow correction factor (for a sharp nose) (eq 214, p 482)
% -------------------------------------------------------------------------
machNumber = freeStreamVelocity/speedOfSound;
if machNumber>0.9 && machNumber<=1.05
    dragCoefficient = dragCoefficient*(1 + 35.5*(machNumber-0.9)^2);
elseif machNumber > 1.05 && machNumber <=2
    dragCoefficient = dragCoefficient*(1.27+0.53*exp(-5.2*(machNumber-1.05)));
elseif machNumber > 2 && ~(machNumber<=0.9)
    display('WARNING: In drag calculation, Mach number exceeds validity range.');
end

end