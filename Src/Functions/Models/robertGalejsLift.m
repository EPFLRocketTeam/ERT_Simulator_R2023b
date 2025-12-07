function [Calpha2, Xp] = robertGalejsLift(Rocket, alpha, K)
    
    % cone
    if strcmp(Rocket.coneMode, 'on')
        Ap_cone = 0.5*Rocket.stagePositions(2)*Rocket.stageDiameters(2);
        Xp_cone = 2/3*Rocket.stagePositions(2);
    end

    % numStages
    Ap_stage = zeros(1, Rocket.numStages-2);
    Xp_stage = zeros(1, Rocket.numStages-2);
    for i = 1:(Rocket.numStages-2)
        Ap_stage(i) = (Rocket.stageDiameters(i+1)+Rocket.stageDiameters(i+2))/2*(Rocket.stagePositions(i+2)-Rocket.stagePositions(i+1));
        Xp_stage(i) = Rocket.stagePositions(i+1)+1/3*(Rocket.stagePositions(i+2)-Rocket.stagePositions(i+1))*(Rocket.stageDiameters(i+1)+2*Rocket.stageDiameters(i+2))/(Rocket.stageDiameters(i+1)+Rocket.stageDiameters(i+2));
    end
    
    % Output
    Ap = Ap_stage;
    Xp = Xp_stage;
    if strcmp(Rocket.coneMode, 'on')
        Ap = [Ap_cone, Ap];
        Xp = [Xp_cone, Xp];
    end
    Calpha2 = 4/pi/Rocket.stageDiameters(2)^2*K*Ap*alpha;
end 