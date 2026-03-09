function [mass,massRate] = massNonLin(t,Rocket)
%	Return the tocket mass during burn time
%   INPUT:
%   - t         Time
%   - Rocket    Structure Containing all datas
%   OUTPUT:
%   - Mass      Rocket mass
%   - dMassdt   Rocket mass derivative over time

% OUTPUT:
if t>Rocket.burnTime
    mass = Rocket.emptyMass+Rocket.motorMass-Rocket.propelMass;
    massRate = 0;
else
    tt = linspace(0,t,500);
    Current_Impulse = trapz(tt,thrust(tt,Rocket));
    mass = Rocket.emptyMass + Rocket.motorMass - Rocket.Thrust2dMass_Ratio*Current_Impulse;
    massRate = Rocket.Thrust2dMass_Ratio*thrust(t,Rocket);
end
end

