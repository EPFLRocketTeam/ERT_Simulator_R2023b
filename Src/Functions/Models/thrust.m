function thrust = thrust(time,Rocket)
%	Return the motor thrust along its axis
%   INPUT:
%   - t         Time
%   - Rocket    Structure Containing all datas
%   OUTPUT:
%   - T         Motor thrust

%   Linear Interpolation
if time > Rocket.burnTime 
    thrust = 0;
elseif time < 0
    thrust = 0;
else
    thrust = interp1(Rocket.thrustTime,Rocket.thrustForce,time);
end
end

