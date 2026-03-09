function [mass,massRate] = massLin(time,Rocket)
%	Return the rocket mass during burn time
%   INPUT:
%   - t         Time
%   - Rocket    Structure Containing all datas
%   OUTPUT:
%   - Mass      Rocket mass
%   - dMassdt   Rocket mass derivative over time

% OUTPUT:
if time > Rocket.burnTime
    mass = Rocket.emptyMass + Rocket.casingMass;
    massRate = 0;
else
    massRate = Rocket.propelMass/Rocket.burnTime;
    mass = Rocket.emptyMass+Rocket.motorMass-time*massRate;
end
end

