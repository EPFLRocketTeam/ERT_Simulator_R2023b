function [Calpha2, centerOfPressure] = robertGalejsLift(Rocket, angleOfAttack, galejsFactor)
    
    % cone
    if strcmp(Rocket.coneMode, 'on')
        Ap_cone = 0.5*Rocket.stagePositions(2)*Rocket.stageDiameters(2);
        centerOfPressureCone = 2/3*Rocket.stagePositions(2);
    end

    % numStages
    Ap_stage = zeros(1, Rocket.numStages-2);
    centerOfPressureStage = zeros(1, Rocket.numStages-2);
    for i = 1:(Rocket.numStages-2)
        Ap_stage(i) = (Rocket.stageDiameters(i+1)+Rocket.stageDiameters(i+2))/2*(Rocket.stagePositions(i+2)-Rocket.stagePositions(i+1));
        centerOfPressureStage(i) = Rocket.stagePositions(i+1)+1/3*(Rocket.stagePositions(i+2)-Rocket.stagePositions(i+1))*(Rocket.stageDiameters(i+1)+2*Rocket.stageDiameters(i+2))/(Rocket.stageDiameters(i+1)+Rocket.stageDiameters(i+2));
    end
    
    % Output
    Ap = Ap_stage;
    centerOfPressure = centerOfPressureStage;
    if strcmp(Rocket.coneMode, 'on')
        Ap = [Ap_cone, Ap];
        centerOfPressure = [centerOfPressureCone, centerOfPressure];
    end
    Calpha2 = 4/pi/Rocket.stageDiameters(2)^2*galejsFactor*Ap*angleOfAttack;
end 