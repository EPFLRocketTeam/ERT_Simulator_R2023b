function [Calpha, centerOfPressure] = barrowmanLift(Rocket, angleOfAttack, machNumber, rollAngle)

    % reference area

    referenceArea = pi*Rocket.stageDiameters(2)^2/4;
    
    % cone
    if strcmp(Rocket.coneMode, 'on')
        if angleOfAttack == 0
            normalLiftDerivativeCone = 2;
        else
            normalLiftDerivativeCone = 2*sin(angleOfAttack)/angleOfAttack;
        end    
    centerOfPressureCone = 2/3*Rocket.stagePositions(2);   
    end
    
    % body
    normalLiftDerivativeStage = zeros(1, Rocket.numStages-2);
    centerOfPressureStage = zeros(1, Rocket.numStages-2);
    for i = 1:(Rocket.numStages-2)
        if angleOfAttack == 0
            normalLiftDerivativeStage(i) = (Rocket.stageDiameters(i+2)^2-Rocket.stageDiameters(i+1)^2)*pi/referenceArea/2;
        else
            normalLiftDerivativeStage(i) = (Rocket.stageDiameters(i+2)^2-Rocket.stageDiameters(i+1)^2)*pi/referenceArea/2*sin(angleOfAttack)/angleOfAttack;
        end
        centerOfPressureStage(i) = Rocket.stagePositions(i+1)+1/3*(Rocket.stagePositions(i+2)-Rocket.stagePositions(i+1))*(1+(1-Rocket.stageDiameters(i+1)/Rocket.stageDiameters(i+2))/(1-(Rocket.stageDiameters(i+1)/Rocket.stageDiameters(i+2))^2));
    end
    
    % fins 
    if(machNumber<1)
        beta  = sqrt(1-machNumber^2);
    else
        %warning('Warining: In barrowman calculations Mach number is > 1.');
        beta = sqrt(machNumber^2-1);
    end
    
    gamma_c = atan(((Rocket.finSweepDistance+Rocket.finTipChord)/2-Rocket.finRootChord/2)/Rocket.finSpan);
    A = 0.5*(Rocket.finTipChord+Rocket.finRootChord)*Rocket.finSpan;
    R = Rocket.stageDiameters(find(Rocket.stagePositions<Rocket.finRootPosition, 1, 'last'))/2;
    KTB = 1 + R/(R+Rocket.finSpan);
    normalLiftDerivative1 = KTB*2*pi*Rocket.finSpan^2/referenceArea/(1+sqrt(1+(beta*Rocket.finSpan^2/A/cos(gamma_c))^2));
    normalLiftDerivativeFins = normalLiftDerivative1*sum(sin(rollAngle+2*pi/Rocket.numFins*(0:(Rocket.numFins-1))).^2);
    centerOfPressureFins = Rocket.finRootPosition + Rocket.finSweepDistance/3*(Rocket.finRootChord+2*Rocket.finTipChord)/(Rocket.finRootChord+Rocket.finTipChord) + 1/6*((Rocket.finRootChord+Rocket.finTipChord)-(Rocket.finRootChord*Rocket.finTipChord)/(Rocket.finRootChord+Rocket.finTipChord));
    
    % Output
    Calpha = [normalLiftDerivativeStage, normalLiftDerivativeFins]; 
    centerOfPressure = [centerOfPressureStage, centerOfPressureFins]; 
    if strcmp(Rocket.coneMode, 'on')
        Calpha = [normalLiftDerivativeCone, Calpha]; 
        centerOfPressure = [centerOfPressureCone, centerOfPressure]; 
    end    
    
    centerOfPressure(find(isnan(centerOfPressure))) = 0;
end