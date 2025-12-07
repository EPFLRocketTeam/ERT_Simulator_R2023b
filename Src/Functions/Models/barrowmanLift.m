function [Calpha, CP] = barrowmanLift(Rocket, alpha, M, theta)

    % reference area

    Aref = pi*Rocket.stageDiameters(2)^2/4;
    
    % cone
    if strcmp(Rocket.coneMode, 'on')
        if alpha == 0
            CNa_cone = 2;
        else
            CNa_cone = 2*sin(alpha)/alpha;
        end    
    CP_cone = 2/3*Rocket.stagePositions(2);   
    end
    
    % body
    CNa_stage = zeros(1, Rocket.numStages-2);
    CP_stage = zeros(1, Rocket.numStages-2);
    for i = 1:(Rocket.numStages-2)
        if alpha == 0
            CNa_stage(i) = (Rocket.stageDiameters(i+2)^2-Rocket.stageDiameters(i+1)^2)*pi/Aref/2;
        else
            CNa_stage(i) = (Rocket.stageDiameters(i+2)^2-Rocket.stageDiameters(i+1)^2)*pi/Aref/2*sin(alpha)/alpha;
        end
        CP_stage(i) = Rocket.stagePositions(i+1)+1/3*(Rocket.stagePositions(i+2)-Rocket.stagePositions(i+1))*(1+(1-Rocket.stageDiameters(i+1)/Rocket.stageDiameters(i+2))/(1-(Rocket.stageDiameters(i+1)/Rocket.stageDiameters(i+2))^2));
    end
    
    % fins 
    if(M<1)
        beta  = sqrt(1-M^2);
    else
        %warning('Warining: In barrowman calculations Mach number is > 1.');
        beta = sqrt(M^2-1);
    end
    
    gamma_c = atan(((Rocket.finSweepDistance+Rocket.finTipChord)/2-Rocket.finRootChord/2)/Rocket.finSpan);
    A = 0.5*(Rocket.finTipChord+Rocket.finRootChord)*Rocket.finSpan;
    R = Rocket.stageDiameters(find(Rocket.stagePositions<Rocket.finRootPosition, 1, 'last'))/2;
    KTB = 1 + R/(R+Rocket.finSpan);
    CNa1 = KTB*2*pi*Rocket.finSpan^2/Aref/(1+sqrt(1+(beta*Rocket.finSpan^2/A/cos(gamma_c))^2));
    CNa_fins = CNa1*sum(sin(theta+2*pi/Rocket.numFins*(0:(Rocket.numFins-1))).^2);
    CP_fins = Rocket.finRootPosition + Rocket.finSweepDistance/3*(Rocket.finRootChord+2*Rocket.finTipChord)/(Rocket.finRootChord+Rocket.finTipChord) + 1/6*((Rocket.finRootChord+Rocket.finTipChord)-(Rocket.finRootChord*Rocket.finTipChord)/(Rocket.finRootChord+Rocket.finTipChord));
    
    % Output
    Calpha = [CNa_stage, CNa_fins]; 
    CP = [CP_stage, CP_fins]; 
    if strcmp(Rocket.coneMode, 'on')
        Calpha = [CNa_cone, Calpha]; 
        CP = [CP_cone, CP]; 
    end    
    
    CP(find(isnan(CP))) = 0;
end