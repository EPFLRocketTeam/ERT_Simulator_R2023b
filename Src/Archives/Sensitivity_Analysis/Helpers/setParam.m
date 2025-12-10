function simulatior3D = setParam(simulatior3D, Xid, X)
%SETPARAM sets the given parameters to the given values in the simulator object.
%   INPUTS
%       simulatior3D  Simulator object (only multilayerwindSimulator3D implemented)
%       Xid     Names of the parameters
%       X       Values of the parameters
%   OUTPUT
%       simulatior3D  Simulator object with the new values


% multilayerwindSimulator3D list of parameters

[rocketparamIDs, envparamIDs] = paramNames(simulatior3D);

% Base wind layer values
layerspeed = [simulatior3D.Environment.Vspeed(2) ...
           simulatior3D.Environment.Vspeed(11) ...
           simulatior3D.Environment.Vspeed(16) ...
           simulatior3D.Environment.Vspeed(51) ...
           simulatior3D.Environment.Vspeed(101) ...
           simulatior3D.Environment.Vspeed(201)];

layerAzi = [simulatior3D.Environment.Vazy(2) ...
           simulatior3D.Environment.Vazy(11) ...
           simulatior3D.Environment.Vazy(16) ...
           simulatior3D.Environment.Vazy(51) ...
           simulatior3D.Environment.Vazy(101) ...
           simulatior3D.Environment.Vazy(201)];

% Boolean for case by case treatment (efficiency)       
isChanged_Vi = false;
isChanged_ai = false;

% Nominal values
layerheight_NV = [10, 100, 200, 500, 1000, 2000];
interpheight = 0:10:4000;
Thrust_Time_NV = [0, 0.02/6.35, 6/6.35, 1];

% Checking that the geometrical information is complete
nz_param = sum(ismember(["z1", "z12", "z23"], Xid));
if  (nz_param ~= 0) && (nz_param ~= 3)
    error("Error: the information about the stages heights is incomplete");
end

nd_param = sum(ismember(["dmin", "dd"], Xid));
if (nd_param ~= 0) && (nd_param ~= 2)
    error("Error: the information about the stages diameters is incomplete");
end


% Changing the parameters in simulatior3D
k = length(Xid);
for i=1:k
    id = Xid(i);
    
    switch id
        case "dmin"
            dmin = X(i);
        case "dd"
            dd = X(i);
        case "z1"
            z1 = X(i);
        case "z12"
            z12 = X(i);
        case "z23"
            z13 = X(i);
        case "railTime"
            simulatior3D.Rocket.Thrust_Force(2) = X(i); 
        case "flightTime"
            simulatior3D.Rocket.Thrust_Force(3) = X(i); 
        case "Vi1"
            layerspeed(1) = X(i);
            isChanged_Vi = true;
        case "Vi2"
            layerspeed(2) = X(i);
            isChanged_Vi = true;
        case "Vi3"
            layerspeed(3) = X(i);
            isChanged_Vi = true;
        case "Vi4"
            layerspeed(4) = X(i);
            isChanged_Vi = true;
        case "Vi5"
            layerspeed(5) = X(i);
            isChanged_Vi = true;
        case "Vi6"
            layerspeed(6) = X(i);
            isChanged_Vi = true;
        case "ai1"
            layerAzi(1) = X(i);
            isChanged_ai = true;
        case "ai2"
            layerAzi(2) = X(i);
            isChanged_ai = true;
        case "ai3"
            layerAzi(3) = X(i);
            isChanged_ai = true;
        case "ai4"
            layerAzi(4) = X(i);
            isChanged_ai = true;
        case "ai5"
            layerAzi(5) = X(i);
            isChanged_ai = true;
        case "ai6"
            layerAzi(6) = X(i);
            isChanged_ai = true;
        case "Burn_Time"
            simulatior3D.Rocket.Burn_Time = X(i);
            simulatior3D.Rocket.Thrust_Time =  X(i) * Thrust_Time_NV;
        otherwise
            if ismember(id, rocketparamIDs)
                simulatior3D.Rocket.(id) = X(i);
            elseif ismember(id, envparamIDs)
                simulatior3D.Environment.(id) = X(i);
            else
                print("Error: unknown parameter (", id, ")");
            end
    end
end

if nz_param == 3
    simulatior3D.Rocket.stagePositions = [0, z1, z1 + z12, z1 + z12 + z13];
end

if nd_param == 2
    simulatior3D.Rocket.stageDiameters = [0, dmin + dd, dmin + dd, dmin];
end

if isChanged_Vi
    simulatior3D.Environment.Vspeed = interp1(layerheight_NV, layerspeed, interpheight, 'pchip', 'extrap');
end

if isChanged_ai
    simulatior3D.Environment.Vazy = interp1(layerheight_NV, layerAzi, interpheight, 'pchip', 'extrap');
    simulatior3D.Environment.Vdirx= cosd(simulatior3D.Environment.Vazy);
    simulatior3D.Environment.Vdiry= sind(simulatior3D.Environment.Vazy);
end

% Changing the dependent parameters (see rocketReader.m part 4)

% 4.1 Maximum body diameter
simulatior3D.Rocket.maxDiameter = simulatior3D.Rocket.stageDiameters(find(simulatior3D.Rocket.stageDiameters == max(simulatior3D.Rocket.stageDiameters), 1, 'first'));
% 4.2 Fin cord
simulatior3D.Rocket.meanFinChord = (simulatior3D.Rocket.finRootChord + simulatior3D.Rocket.finTipChord)/2; 
% 4.3 Maximum cross-sectional body area
simulatior3D.Rocket.maxCrossSectionArea = pi*simulatior3D.Rocket.maxDiameter^2/4; 
% 4.4 Exposed planform fin area
simulatior3D.Rocket.exposedFinArea = (simulatior3D.Rocket.finRootChord + simulatior3D.Rocket.finTipChord )/2*simulatior3D.Rocket.finSpan; 
% 4.5 Body diameter at middle of fin station (CAREFUL, assumption for the SA)
simulatior3D.Rocket.finBodyDiameter = simulatior3D.Rocket.maxDiameter; 
% 4.6 Virtual fin planform area
simulatior3D.Rocket.virtualFinArea = simulatior3D.Rocket.exposedFinArea + 1/2*simulatior3D.Rocket.finBodyDiameter*simulatior3D.Rocket.finRootChord; 
% 4.8 Rocket Length
simulatior3D.Rocket.totalLength = simulatior3D.Rocket.stagePositions(end);
% Saturation Vapor Ration
p_ws = exp(77.345+0.0057*simulatior3D.Environment.groundTemperature-7235/simulatior3D.Environment.groundTemperature)/simulatior3D.Environment.groundTemperature^8.2;
p_a = simulatior3D.Environment.groundPressure;
simulatior3D.Environment.Saturation_Vapor_Ratio = 0.62198*p_ws/(p_a-p_ws);
% Casing masses
simulatior3D.Rocket.casing_massP = simulatior3D.Rocket.motor_massP-simulatior3D.Rocket.propel_massP;
simulatior3D.Rocket.casing_massF = simulatior3D.Rocket.motor_massF-simulatior3D.Rocket.propel_massF;
%Global motor info
simulatior3D.Rocket.motor_dia = max(simulatior3D.Rocket.motor_diaP, simulatior3D.Rocket.motor_diaF);
simulatior3D.Rocket.motor_length = simulatior3D.Rocket.motor_lengthP + simulatior3D.Rocket.motor_lengthF + simulatior3D.Rocket.interMotorDistance ;
simulatior3D.Rocket.propel_mass = simulatior3D.Rocket.propel_massP + simulatior3D.Rocket.propel_massF ;
simulatior3D.Rocket.motor_mass = simulatior3D.Rocket.motor_massP + simulatior3D.Rocket.motor_massF; 
simulatior3D.Rocket.casing_mass = simulatior3D.Rocket.casing_massP + simulatior3D.Rocket.casing_massF ;
% Mass variation coefficient
A_T = trapz(simulatior3D.Rocket.Thrust_Time, simulatior3D.Rocket.Thrust_Force);
simulatior3D.Rocket.Thrust2dMass_Ratio = simulatior3D.Rocket.propel_mass/(A_T);


end
