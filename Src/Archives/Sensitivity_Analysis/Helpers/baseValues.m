 function XX = baseValues(simulatior3D, Xid, s)
%BASEVALUES extracts the parameters of the already existing simulatior3D and return the domain
%of definition of each value (i.e. the domain in which it can variate for the SA).
%   INPUTS: 
%       simulatior3D      Simulator object containing the base values of the parameters
%       Xid         IDs of the parameters that will change during the SA.
%       s           Relative size of the the domain of definition (i.g. if x0 is the base
%                   value of parameter x, then x will take value in [a, b] with
%                   a=x0(1-s) and b=x0(1+s). Hence, 0<=s<1.
%   OUTPUT:
%       XX          Matrix that has one row of the form [x0 a b] for each parameter in
%                   Xid.

% Error management
if (s <= 0) || (s > 1)
    print("Warning: sigma out of bound (must be in [0, 1[).")
end

%TODO Check that the base values are correct

[rocketparamIDs, envparamIDs] = paramNames(simulatior3D);

% Initialazing
k = length(Xid);
XX = NaN(k, 3);

for i=1:k
    id = Xid(i);
    
    switch id
        case "dmin"
            XX(i,1) = simulatior3D.Rocket.("stageDiameters")(end);
        case "dd"
            XX(i,1) = max(simulatior3D.Rocket.("stageDiameters")) - simulatior3D.Rocket.("stageDiameters")(end);
        case "z1"
            XX(i,1) = simulatior3D.Rocket.("stagePositions")(2);
        case "z12"
            XX(i,1) = simulatior3D.Rocket.("stagePositions")(3) - simulatior3D.Rocket.("stagePositions")(2);
        case "z23"
            XX(i,1) = simulatior3D.Rocket.("stagePositions")(4) - simulatior3D.Rocket.("stagePositions")(3); 
        case "railTime"
            XX(i,1) = simulatior3D.Rocket.("Thrust_Force")(2); 
        case "flightTime"
            XX(i,1) = simulatior3D.Rocket.("Thrust_Force")(3);
        case "Vi1"
            XX(i,1) = simulatior3D.Environment.("Vspeed")(2);
        case "Vi2"
            XX(i,1) = simulatior3D.Environment.("Vspeed")(11);
        case "Vi3"
            XX(i,1) = simulatior3D.Environment.("Vspeed")(16);
        case "Vi4"
            XX(i,1) = simulatior3D.Environment.("Vspeed")(51);
        case "Vi5"
            XX(i,1) = simulatior3D.Environment.("Vspeed")(101);
        case "Vi6"
            XX(i,1) = simulatior3D.Environment.("Vspeed")(201);
        case "ai1"
            XX(i,1) = simulatior3D.Environment.("Vazy")(2);
        case "ai2"
            XX(i,1) = simulatior3D.Environment.("Vazy")(11);
        case "ai3"
            XX(i,1) = simulatior3D.Environment.("Vazy")(16);
        case "ai4"
            XX(i,1) = simulatior3D.Environment.("Vazy")(51);
        case "ai5"
            XX(i,1) = simulatior3D.Environment.("Vazy")(101);
        case "ai6"
            XX(i,1) = simulatior3D.Environment.("Vazy")(201);
        otherwise
            if ismember(id, rocketparamIDs)
                XX(i,1) = simulatior3D.Rocket.(id);
            elseif ismember(id, envparamIDs)
                XX(i,1) = simulatior3D.Environment.(id);
            else
                print("Error: unknown parameter (", id, ")");
            end
    end
end

% Compute the intervals
XX(:,2) = (1-s)*XX(:,1);
XX(:,3) = (1+s)*XX(:,1);

% Checking the physical conditions
if simulatior3D.Rocket.stagePositions(3)*(1-s) < (simulatior3D.Rocket.finRootChord + simulatior3D.Rocket.finRootPosition)*(1+s)
    disp("Warning: the fins might be two far back compared to the length of the rocket (unrealistic configuration). Decrease sigma.");
if simulatior3D.Rocket.stagePositions(3)*(1-s) < simulatior3D.Rocket.airbrakePosition*(1+s)
    disp("Warning: the airbrakes might be two far back compared to the length of the rocket (unrealistic configuration). Decrease sigma."); 
end

end

