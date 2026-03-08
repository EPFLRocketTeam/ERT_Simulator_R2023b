function [mass, massRate, centerOfMass, inertiaMoment, inertiaRate] = RocketInertia(t, Rocket, massModel)
%         M     dMdt      CM            I              dIdt
% ROCKETINERTIA Compute mass, mass derivative, center of mass position from
% cone tip, Inertia moment tensor and derivative of the inertia moment
% tensor. 
% INPUTS : 
% - t           :   time [s]
% - Rocket      :   Rocket structure
% - massModel   :   mass model selection [0 (linear), 1 (non-linear)]
% OUTPUTS
% - M           :   Mass [kg]
% - dMdt        :   Time derivative of mass [kg/s]
% - CM          :   Center of mass from cone tip [m]
% - I           :   Moment of inertia tensor in rocket coordinate system (3x3)[kgm^2]
% - dIdt        :   Time derivative of moment of inertia tensor in rocket coordinate system (3x3)[kgm^2/s]

% Compute mass values
if(massModel)
    [mass, massRate] = Mass_Lin(t, Rocket);
else
    [mass, massRate] = Mass_Non_Lin(t, Rocket);
end

% Compute center of mass
centerOfMass = (Rocket.emptyCenterOfMass*Rocket.emptyMass + ... 
    (mass-Rocket.emptyMass)*(Rocket.totalLength-Rocket.motorLength/2))/mass;

% Compute Inertia tensor
% longitudinal inertia
innerDiameterGrain = 0.005; % Diametre interieur grains (Tjr identique)
outerDiameterGrain = Rocket.motor_dia/2; % Diametre exterieur grains

inertiaCasing = Rocket.casingMass*(Rocket.motorLength^2/12 + outerDiameterGrain^2/2); 

grainMass = mass-Rocket.emptyMass-Rocket.casingMass; % Masse des grains
inertiaGrain = grainMass*(Rocket.motorLength^2/12 + (outerDiameterGrain^2+innerDiameterGrain^2)/4);

inertia = Rocket.emptyInertia + inertiaCasing + inertiaGrain + ...
    (grainMass+Rocket.casingMass)*...
    (Rocket.totalLength-centerOfMass-Rocket.motorLength/2); % I + ... + Steiner
% rotational inertia

inertiaMoment = [inertia, 1];
% TODO : rotational inertia and inertia moment derivative.
inertiaRate = 0;
end