function Rocket = rocketReader(rocketFilePath)

% -------------------------------------------------------------------------
% 1. Read Rocket Configuration File
% -------------------------------------------------------------------------

fileId = fopen(rocketFilePath);

if fileId < 0
   error('ERROR: Rocket file not found: %s', rocketFilePath) 
end
Rocket.isHybrid = false;

% Rocket Motor on/off parameter
Rocket.motorState = 'off';

while ~feof(fileId)
    lineContent = fgetl(fileId);
    [lineIdentifier, lineData] = strtok(lineContent);
    
    switch lineIdentifier
        
        % Number of diameter stages in the rocket
        case 'numStages'
            lineDataNumeric = textscan(lineData, '%f');
            Rocket.numStages = lineDataNumeric{1}(1);
            
        % Diameters at each stage [m]
        case 'stageDiameters'
            lineDataNumeric = textscan(lineData, '%f');
            Rocket.stageDiameters = lineDataNumeric{1}';
            
        % Position from rocket tip for each diameter change [m]
        case 'stagePositions'
            lineDataNumeric = textscan(lineData, '%f');
            Rocket.stagePositions = lineDataNumeric{1}';
        
        % Cone inclusion in aerodynamic calculations
        case 'coneMode'
            lineDataString = textscan(lineData,'%s');
            Rocket.coneMode = lineDataString{1}{1};    
            
        % Number of fins    
        case 'numFins'
            lineDataNumeric = textscan(lineData, '%f');
            Rocket.numFins = lineDataNumeric{1}(1);
        
        % Distance from rocket tip to fin leading edge root [m]    
        case 'finRootPosition'
            lineDataNumeric = textscan(lineData, '%f');
            Rocket.finRootPosition = lineDataNumeric{1}(1);
        
        % Fin span [m]    
        case 'finSpan'
            lineDataNumeric = textscan(lineData, '%f');
            Rocket.finSpan = lineDataNumeric{1}(1);
        
        % Fin root chord [m]    
        case 'finRootChord'
            lineDataNumeric = textscan(lineData, '%f');
            Rocket.finRootChord = lineDataNumeric{1}(1);    
        
        % Fin tip chord [m]    
        case 'finTipChord'
            lineDataNumeric = textscan(lineData, '%f');
            Rocket.finTipChord = lineDataNumeric{1}(1);
        
        % Fin thickness [m]    
        case 'finThickness'
            lineDataNumeric = textscan(lineData, '%f');
            Rocket.finThickness = lineDataNumeric{1}(1);
            
        % Axial distance between fin leading edge root and tip [m]        
        case 'finSweepDistance'
            lineDataNumeric = textscan(lineData, '%f');
            Rocket.finSweepDistance = lineDataNumeric{1}(1);
        
        % Fin leading edge length [m]
        case 'finLeadingEdgeLength'
            lineDataNumeric = textscan(lineData, '%f');
            Rocket.finLeadingEdgeLength = lineDataNumeric{1}(1);    
        
        % Fin trailing edge length [m]
        case 'finTrailingEdgeLength'
            lineDataNumeric = textscan(lineData, '%f');
            Rocket.finTrailingEdgeLength = lineDataNumeric{1}(1); 
            
        % Number of launch lugs    
        case 'numLaunchLugs'
            lineDataNumeric = textscan(lineData, '%f');
            Rocket.numLaunchLugs = lineDataNumeric{1}(1);    
        
        % Exposed lug surface area [m²]    
        case 'lugSurfaceArea'
            lineDataNumeric = textscan(lineData, '%f');
            Rocket.lugSurfaceArea = lineDataNumeric{1}(1);
        
        % Rocket empty mass [kg]    
        case 'emptyMass'
            lineDataNumeric = textscan(lineData, '%f');
            Rocket.emptyMass = lineDataNumeric{1}(1);
        
        % Rocket empty inertia [kg·m²]    
        case 'emptyInertia'
            lineDataNumeric = textscan(lineData, '%f');
            Rocket.emptyInertia = lineDataNumeric{1}(1);
        
        % Rocket center of mass for empty configuration [m from tip]
        case 'emptyCenterOfMass'
            lineDataNumeric = textscan(lineData, '%f');
            Rocket.emptyCenterOfMass = lineDataNumeric{1}(1);
        
        % Airbrakes position from rocket tip [m]    
        case 'airbrakePosition'
            lineDataNumeric = textscan(lineData, '%f');
            Rocket.airbrakePosition = lineDataNumeric{1}(1);
        
        % Number of airbrake fins    
        case 'numAirbrakes'
            lineDataNumeric = textscan(lineData, '%f');
            Rocket.numAirbrakes = lineDataNumeric{1}(1);
        
        % Airbrake opening angle [deg]    
        case 'airbrakeAngle'
            lineDataNumeric = textscan(lineData, '%f');
            Rocket.airbrakeAngle = lineDataNumeric{1}(1);    
        
        % Motor configuration file name  
        case 'motorId'
            lineDataString = textscan(lineData,'%s');
            Rocket.motorId = lineDataString{1}{1};
            
        % Hybrid motor configuration
        case 'hybrid'
            lineDataString = textscan(lineData,'%s');
            Rocket.fuelTankId = lineDataString{1}{1};
            Rocket.interMotorDistance = str2double(lineDataString{1}{2});
            Rocket.isHybrid = true;    
        
        % Motor thrust multiplication factor    
        case 'motorThrustFactor'
            lineDataNumeric = textscan(lineData,'%f');
            Rocket.motorThrustFactor = lineDataNumeric{1}(1);    
        
        % Payload mass [kg]    
        case 'payloadMass'
            lineDataNumeric = textscan(lineData,'%f');
            Rocket.payloadMass = lineDataNumeric{1}(1);
        
        % Main parachute drag area (S*CD) [m²]     
        case 'mainParachuteDragArea'
            lineDataNumeric = textscan(lineData,'%f');
            Rocket.mainParachuteDragArea = lineDataNumeric{1}(1);
            
        % Drogue parachute drag area (S*CD) [m²]    
        case 'drogueParachuteDragArea'
            lineDataNumeric = textscan(lineData,'%f');
            Rocket.drogueParachuteDragArea = lineDataNumeric{1}(1);
           
        % Main parachute deployment altitude [m]    
        case 'mainParachuteDeploymentAltitude'
            lineDataNumeric = textscan(lineData,'%f');
            Rocket.mainParachuteDeploymentAltitude = lineDataNumeric{1}(1); 
        
        % Center of pressure position error factor     
        case 'centerOfPressureFactor'
            lineDataNumeric = textscan(lineData,'%f');
            Rocket.centerOfPressureFactor = lineDataNumeric{1}(1);
        
        % Normal force coefficient derivative error factor
        case 'normalForceCoefficientFactor'
            lineDataNumeric = textscan(lineData,'%f');
            Rocket.normalForceCoefficientFactor = lineDataNumeric{1}(1);
        
        % Drag coefficient error factor    
        case 'dragCoefficientFactor'
            lineDataNumeric = textscan(lineData,'%f');
            Rocket.dragCoefficientFactor = lineDataNumeric{1}(1);
        
        % Inertia matrix [kg·m²]
        case 'inertiaMatrix'
            lineDataNumeric = textscan(lineData, '%f');
            data = lineDataNumeric{1}';
            inertiaMatrix = [data(1:3); data(4:6); data(7:9)];
            Rocket.inertiaMatrix = inertiaMatrix;
        
        % Fuel tank length [m]
        case 'tankLength'
            lineDataNumeric = textscan(lineData,'%f');
            Rocket.tankLength = lineDataNumeric{1}(1);
        
        % Fuel tank radius [m]
        case 'tankRadius'
            lineDataNumeric = textscan(lineData,'%f');
            Rocket.tankRadius = lineDataNumeric{1}(1);
        
        % Fuel tank position from rocket tip [m]
        case 'tankPosition'
            lineDataNumeric = textscan(lineData,'%f');
            Rocket.tankPosition = lineDataNumeric{1}(1);
                        
        otherwise
            warning('In rocket definition, unknown line identifier: %s', lineIdentifier);
         
    end
end    

fclose(fileId);

% -------------------------------------------------------------------------
% 2. Read Motor Configuration
% -------------------------------------------------------------------------

Rocket = motor2RocketReader(Rocket.motorId, Rocket);

% -------------------------------------------------------------------------
% 3. Validation Checks
% -------------------------------------------------------------------------

if validateStages(Rocket)
    error('ERROR: Invalid rocket stage configuration.')
end

if ~(strcmp(Rocket.coneMode, 'on') || strcmp(Rocket.coneMode, 'off'))
    error('ERROR: Cone mode parameter "%s" unknown. Use "on" or "off".', Rocket.coneMode);
end

% -------------------------------------------------------------------------
% 4. Derived Parameters
% -------------------------------------------------------------------------

% 4.1 Maximum body diameter [m]
Rocket.maxDiameter = Rocket.stageDiameters(find(Rocket.stageDiameters == max(Rocket.stageDiameters), 1, 'first'));

% 4.2 Mean fin chord [m]
Rocket.meanFinChord = (Rocket.finRootChord + Rocket.finTipChord) / 2; 

% 4.3 Maximum cross-sectional body area [m²]
Rocket.maxCrossSectionArea = pi * Rocket.maxDiameter^2 / 4; 

% 4.4 Exposed planform fin area [m²]
Rocket.exposedFinArea = (Rocket.finRootChord + Rocket.finTipChord) / 2 * Rocket.finSpan; 

% 4.5 Body diameter at middle of fin station [m]
Rocket.finBodyDiameter = interp1(Rocket.stagePositions, Rocket.stageDiameters, ...
    Rocket.finRootPosition + Rocket.finRootChord / 2, 'linear', 'extrap'); 

% 4.6 Virtual fin planform area [m²]
Rocket.virtualFinArea = Rocket.exposedFinArea + 1/2 * Rocket.finBodyDiameter * Rocket.finRootChord; 

% 4.7 Rocket total length [m]
Rocket.totalLength = Rocket.stagePositions(end);

% 4.8 Rocket inertia including motor and tank
[~, ~, ~, ~, longitudinalInertia, ~, radialInertia, ~] = Mass_Properties(0, Rocket, 'Linear');
if ~isfield(Rocket, 'inertiaMatrix')
    Rocket.inertiaMatrix = [longitudinalInertia, 0, 0; 0, longitudinalInertia, 0; 0, 0, radialInertia];
end

end

% -------------------------------------------------------------------------
% Helper Functions
% -------------------------------------------------------------------------

function isInvalid = validateStages(Rocket)
    isInvalid = false;
    
    if ~(length(Rocket.stageDiameters) == Rocket.numStages && ...
         length(Rocket.stagePositions) == Rocket.numStages)
        isInvalid = true;
        display('ERROR: Rocket stageDiameters and/or stage positions length mismatch with declared stages.');
    elseif ~(Rocket.stageDiameters(1) == 0 && Rocket.stagePositions(1) == 0)
        isInvalid = true;
        display('ERROR: Rocket must start with a point (diameter = 0, position = 0).');
    end
end