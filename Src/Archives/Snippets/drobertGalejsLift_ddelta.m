function dCalpha2 = drobertGalejsLift_ddelta(Rocket, alpha, K)

    % cone
    Ap_cone = 0.5*Rocket.stagePositions(2)*Rocket.stageDiameters(2);
    Xp_cone = 2/3*Rocket.stagePositions(2);

    % numStages
    Ap_stage = zeros(1, Rocket.numStages-2);
    Xp_stage = zeros(1, Rocket.numStages-2);
    for i = 1:(Rocket.numStages-2)
        Ap_stage(i) = (Rocket.stageDiameters(i+1)+Rocket.stageDiameters(i+2))/2*(Rocket.stagePositions(i+2)-Rocket.stagePositions(i+1));
        Xp_stage(i) = Rocket.stagePositions(i+1)+1/3*(Rocket.stagePositions(i+2)-Rocket.stagePositions(i+1))*(Rocket.stageDiameters(i+1)+2*Rocket.stageDiameters(i+2))/(Rocket.stageDiameters(i+1)+Rocket.stageDiameters(i+2));
    end
    
    % Output
    dCalpha2 = 4/pi/Rocket.stageDiameters(2)^2*K*[Ap_cone, Ap_stage];
end

