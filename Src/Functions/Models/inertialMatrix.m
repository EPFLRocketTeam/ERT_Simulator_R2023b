

function I = inertialMatrix(rocket, Cm, t)
    if t < rocket.burnTime
        % Find position of the CM of the propelant 
        zPropel = rocket.tankZ + (1 - t / rocket.burnTime) * rocket.tankL;

        % Evaluate the mass of propelant
        propelantMass = rocket.propelMass * (1 - t / rocket.burnTime);

        % Find the distance between rocket CM and propelant CM
        deltaZ = zPropel - Cm;
    
        % Compute I of propelant
        I = rocket.rocketInertia + ...
            inertialFillCylinder(propelantMass, zPropel, rocket.tankR);% + ...
            %huygensSteinerMatrix(propelantMass, 0, 0, deltaZ);
    else
        I = rocket.rocketInertia;
    end
end

% Compute the inertia matrix of a fill cylinder
% in    m : mass
% in    h : height
% in    r : radius
% out   I : inertia matrix

% Inertia matrix
%       |Ixx Ixy Ixz|
%   I = |Iyx Iyy Iyz|
%       |Izx Izy Izz|

function [I] = inertialFillCylinder(m, h, r)
    I = [m*h^2 / 12 + m*r^2 / 4, 0, 0;
        0, m*h^2 / 12 + m*r^2 / 4, 0;
        0, 0, m*r^2 / 4];
end

% Compute Huygens-Steiner matrix for inertial matrix displacement
function I = huygensSteinerMatrix(m, x, y, z)
    I = [m*(y^2 + z^2), -m*x*y, -m*x*z;
        -m*x*y, m*(x^2 + z^2), -m*y*z;
        -m*x*z, -m*y*z, m*(x^2 + y^2)];
end