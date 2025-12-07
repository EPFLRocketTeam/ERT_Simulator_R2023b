function dCalpha = dbarrowmanLift_ddelta(Rocket, alpha, M, theta)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here


    % reference area

    Aref = pi*Rocket.stageDiameters(2)^2/4;
    
    % cone
    if alpha == 0
        dCNa_cone = 0;
    else
        dCNa_cone = 2*(cos(alpha)/alpha-1/alpha^2*sin(alpha));
    end
    
    % numStages
    dCNa_stage = zeros(1, Rocket.numStages-2);
    CP_stage = zeros(1, Rocket.numStages-2);
    for i = 1:(Rocket.numStages-2)
        if alpha == 0
            dCNa_stage(i) = (Rocket.stageDiameters(i+2)^2-Rocket.stageDiameters(i+1)^2)*pi/Aref/2;
        else
            dCNa_stage(i) = (Rocket.stageDiameters(i+2)^2-Rocket.stageDiameters(i+1)^2)*pi/Aref/2*(cos(alpha)/alpha-1/alpha^2*sin(alpha));
        end 
    end
    
    % Output
    dCalpha = [dCNa_cone, dCNa_stage,0]; 
end

