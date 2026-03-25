function C1 = CorrectionMoment(t,Rocket,normalForceCoefficientSlope,Xp,Velocity,Environnement,Altitude)
% Give the corrective moment based on rocket geometry
%

%--------------------------------------------------------------------------
% 1 Intrasic parameter
%--------------------------------------------------------------------------
[T, a, p, density, Nu] = stdAtmos(Altitude,Environnement);
[M,dMdt,Cm,dCmdt,I_L,dI_Ldt,I_R,dI_Rdt] = massProperties(t,Rocket,'NonLinear');

%--------------------------------------------------------------------------
% 2 Total Damping Coefficient
%--------------------------------------------------------------------------
C1 = 1/2*density*Rocket.maxCrossSectionArea*Velocity^2*normalForceCoefficientSlope*(Xp-Cm);

end

