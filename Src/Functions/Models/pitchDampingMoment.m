function pitchDampingCoefficient = pitchDampingMoment(Rocket, rho, Calpha, centerOfPressure, massRate, centerOfMass, w, V)
% PITCHDAMPINGMOMENT computes the pitch damping moment coefficient of the
% rocket. It also applies to yaw damping, but not to roll!
% Damping is based on the rocket's geometry i.e the air resistance opposing
% its rotational movement and the mass change rate during the thrust phase.

    if V == 0
        pitchDampingCoefficient = 0;
    else
        % -------------------------------------------------------------------------
        % thrust damping
        % -------------------------------------------------------------------------

        thrustDampingCoefficient = massRate*(Rocket.totalLength-centerOfMass).^2*w*2/V^2/rho/Rocket.maxCrossSectionArea;

        % -------------------------------------------------------------------------
        % Aerdynamic damping
        % -------------------------------------------------------------------------

        %Method found in peak of flight news letter, indications towards an article
        %from Bryson, 1953, Stability Derivatives for a Slender Missile With
        %Application to a Wing- Body-Vertical-Tail Configuration,
        % or from Sacks, 1954, Aerodynamic forces, moments, and stability
        % derivatives for slender bodies of general cross section 
        %Aerodynamic damping coefficient
        normalLiftDerivativeTotal = sum(Calpha.*(centerOfPressure-centerOfMass).^2);
        % Total
        aerodynamicDampingCoefficient = normalLiftDerivativeTotal*w/V;

%         % OpenRocket method, see OpenRocket documentation 3.2.3
%         % Damping coefficient relative to body
%         CDM_body = 0.275*Rocket.stagePositions(end)^4/Rocket.maxCrossSectionArea*w^2/V^2;
%         % Damping coefficient relative to fins
%         CDM_fin = 0.6*Rocket.numFins*Rocket.exposedFinArea*(Rocket.finRootPosition-CM)^3/Rocket.maxCrossSectionArea/Rocket.maxDiameter*w^2/V^2;
%         % Total
%         CDM_aero = CDM_body + CDM_fin;

        % -------------------------------------------------------------------------
        % Total damping
        % -------------------------------------------------------------------------
        pitchDampingCoefficient = aerodynamicDampingCoefficient + thrustDampingCoefficient;
    end
end