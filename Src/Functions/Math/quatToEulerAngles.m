% Transform quaternions to Euler angles
% phi: roll
% theta: pitch
% psi: yaw
% Source of formula used:
% https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles,
% section "Quaternion to Euler angles (in 3-2-1 sequence) conversion

% Only works with normalized quaternions

function [phi, theta, psi] = quatToEulerAngles(qx, qy, qz, qw)
    phi = atan2(2.*(qw.*qx + qy.*qz), 1 - 2.*(qx.^2 + qy.^2));
    theta = -pi/2 + 2.*atan2(sqrt(1 + 2.*(qw.*qy - qx.*qz)), ...
        sqrt(1 - 2.*(qw.*qy - qx.*qz)));
    psi = atan2(2.*(qw.*qz + qx.*qy), 1 - 2.*(qy.^2 + qz.^2));
end