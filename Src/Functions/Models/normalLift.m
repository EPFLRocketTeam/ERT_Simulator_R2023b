function [ ...
    normalLiftDerivative, ... CNa
    centerOfPressure, ... Xp
    normalLiftDerivativeBarrowman, ... CNa_barrowman
    centerOfPressureBarrowman ... Xp_barrowman
] = normalLift( ...
    Rocket, ...
    angleOfAttack, ... alpha
    galejsFactor, ... K
    machNumber, ... M
    rollAngle, ... theta
    Galejs...
)
% NORMALLIFT computes the normal force intensity applied to the center of
% pressure according to Barrowman's theory and corrections for extreme
% aspect ratio bodies proposed by robert Galejs.
% INPUTS:
% - Rocket      : Rocket object
% - alpha       : angle of attack [rad]
% - K           : Robert Galejs' correction factor
% - M           : Mach number
% - theta       : Roll angle [rad]
% - Galejs      : Flag indicating use of Galejs' correction or not [1 or 0]
% OUTPUTS:
% - CNa         : Normal lift derivative
%   versus delta coefficient derivative [1/rad]
% - Xp          : Center of pressure
% - CNa_barrowman: Normal lift coefficient derivatives of rocket components
%   according to barrowman theory [1/rad]
% - Xp_barrowman: Center of pressure of rocket components
%   according to barrowman theory [1/rad]

[normalLiftDerivativeBarrowman, centerOfPressureBarrowman] ...
    = barrowmanLift(Rocket, angleOfAttack, machNumber, rollAngle);
% Fac for montecarlo simulation 
centerOfPressureBarrowman ...
    = centerOfPressureBarrowman*Rocket.centerOfPressureFactor;
normalLiftDerivativeBarrowman ...
    = normalLiftDerivativeBarrowman*Rocket.normalForceCoefficientFactor;
if Galejs
    [normalLiftDerivativeGalejs, centerOfPressureGalejs] ...
        = robertGalejsLift(Rocket, angleOfAttack, galejsFactor);
    normalLiftDerivative ...
        = sum([normalLiftDerivativeBarrowman, normalLiftDerivativeGalejs]);
    centerOfPressure = sum([ ...
        normalLiftDerivativeBarrowman.*centerOfPressureBarrowman, ...
        normalLiftDerivativeGalejs.*centerOfPressureGalejs ...
    ])/normalLiftDerivative;
else
    normalLiftDerivative = sum(normalLiftDerivativeBarrowman);
    centerOfPressure ...
        = sum(normalLiftDerivativeBarrowman.*centerOfPressureBarrowman) / ...
        normalLiftDerivative;
end

end