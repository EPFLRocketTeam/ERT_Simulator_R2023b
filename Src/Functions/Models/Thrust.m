function Thrust = thrust(time,Rocket)
%	Return the motor thrust along its axis
%   INPUT:
%   - t         Time
%   - Rocket    Structure Containing all datas
%   OUTPUT:
%   - T         Motor Thrust

%   Linear Interpolation
if time > Rocket.burnTime 
    Thrust = 0;
elseif time < 0
    Thrust = 0;
else
    Thrust = interp1(Rocket.thrustTime,Rocket.thrustForce,time);
end
end

