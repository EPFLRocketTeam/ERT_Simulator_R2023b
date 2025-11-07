% Compute the inertia matrix of a fill cylinder
% in    m : mass
% in    h : height
% in    r : radius
% out   I : inertia matrix

% Inertia matrix
%       |Ixx Ixy Ixz|
%   I = |Iyx Iyy Iyz|
%       |Izx Izy Izz|

function [I] = inertia_fill_cylinder(m, h, r)
    I = [m*h^2 / 12 + m*r^2 / 4, 0, 0;
        0, m*h^2 / 12 + m*r^2 / 4, 0;
        0, 0, m*r^2 / 4];
end