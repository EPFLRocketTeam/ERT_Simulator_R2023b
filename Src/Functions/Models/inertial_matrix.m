

function I = inertial_matrix(rocket, Cm, t)
    if t < rocket.Burn_Time
        % Find position of the CM of the propelant 
        z_propel = rocket.tank_z + (1 - t / rocket.Burn_Time) * rocket.tank_L;

        % Evaluate the mass of propelant
        propelant_mass = rocket.propel_mass * (1 - t / rocket.Burn_Time);

        % Find the distance between rocket CM and propelant CM
        delta_z = z_propel - Cm;
    
        % Compute I of propelant
        I = rocket.emptyInertianertia + ...
            inertial_fill_cylinder(propelant_mass, z_propel, rocket.tank_r);% + ...
            %huygens_steiner_matrix(propelant_mass, 0, 0, delta_z);
    else
        I = rocket.emptyInertianertia;
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

function [I] = inertial_fill_cylinder(m, h, r)
    I = [m*h^2 / 12 + m*r^2 / 4, 0, 0;
        0, m*h^2 / 12 + m*r^2 / 4, 0;
        0, 0, m*r^2 / 4];
end

% Compute Huygens-Steiner matrix for inertial matrix displacement
function I = huygens_steiner_matrix(m, x, y, z)
    I = [m*(y^2 + z^2), -m*x*y, -m*x*z;
        -m*x*y, m*(x^2 + z^2), -m*y*z;
        -m*x*z, -m*y*z, m*(x^2 + y^2)];
end