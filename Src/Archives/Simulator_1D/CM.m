function [cm dcmdt] = CM(t,Rocket)
%	Return the rocekt center of mass
%   INPUT:
%   - t         Time
%   - Rocket    Structure Containing all datas
%   OUTPUT:
%   - cm        Center of mass

% Appels de fonctions necessaires
[M,dMdt] = Mass_Lin(t,Rocket);

% Centre de masse
cm = (Rocket.emptyCenterOfMass*Rocket.emptyMass + ... 
    (M-Rocket.emptyMass)*(Rocket.totalLength-Rocket.motor_length/2))/M;

% D?riv?e centre de masse
dcmdt = (dMdt*(Rocket.totalLength-Rocket.motor_length/2)-dMdt*cm)/M;
end

