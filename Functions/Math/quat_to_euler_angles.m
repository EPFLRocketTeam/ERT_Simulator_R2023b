% Transform quaternions to Euler angles

function [phi, theta, psi] = quat_to_euler_angles(qx, qy, qz, qw)
    phi = atan2(2.*(qw.*qx + qy.*qz), 1 - 2.*(qx.^2 + qy.^2));
    theta = -pi/2 + 2.*atan2(sqrt(1 + 2.*(qw.*qy - qx.*qz)), ...
        sqrt(1 - 2.*(qw.*qy - qx.*qz)));
    psi = atan2(2.*(qw.*qz + qx.*qy), 1 - 2.*(qy.^2 + qz.^2));
end