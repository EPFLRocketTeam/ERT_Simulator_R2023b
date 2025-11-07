%% === Inertial coupling ===

% Make a simulation before using this script. This script will use the
% result of the simulation to make inertial coupling simulation using the
% data of the first simulations.

% Index of the frame to begin with
id_initial = 100;

% Use previous simulation
% S_dot = [X_dot;V_dot;Q_dot;W_dot];
S_simu = S2;
t_simu = T2;
X_simu = S_simu(:,1:3);
V_simu = S_simu(:,4:6);
Q_simu = S_simu(:,7:10);
W_simu = S_simu(:,11:13);

T = 3736; % thrust in N (2600)
alpha = rad(0); % thrust deviation angle in rad (put a value in deg)
m = 75; % rocket mass in kg
x_g = 3.285; % position of the center of mass measured from the top in m
D = 0.185; % rocket diameter in m
R = 0.5*D; % radius in m
h = 4.115; % rocket length in m
Dbot = h - x_g; % distance between center of mass and bottom of the rocket in m
Dtop = x_g; % distance between center of mass and top of the rocket in m
Dtop = h-Dbot; % distance between center of mass and top of the rocket in m
S = pi*R^2; % Section of the rocket
Cd = 0.35; % drag coefficient
MS = 2.5; % static margin
Dgp = MS*D; % distance center of mass - center of pressure in m

rho_atm = 1.225; % density in kg/m^3
g = 9.81; % gravity in m/s^2

%Ig = m*R^2/4 + m*h^2/12; % moment of inertia for a HOMOGENEOUS cylinder around any perpendicular axis
Ig = (1/6) * m * (Dtop^2 + Dbot^2) + m*R^2/4; % moment of inertia for a cylinder divided in 2 distinct homogeneous parts
Iz = 0.5*m*R^2; % moment of inertia around axis of symmetry (for homogeneous cylinder)
I = diag([Ig Ig Iz]); % in axes of inertia reference frame
beta = rad(1); % angle in (x,z) plane between axis of inertia and geometrical axis (in rad)
P = mat3_ini(cos(beta), 0, sin(beta), 0, 1, 0, -sin(beta), 0, cos(beta)); % change of basis matrix
I = P*I*inv(P) % in geometrical reference frame;
C2 = 0; %0.1; %damping coefficient in roll

%% simulation variables
t0 = t_simu(id_initial);
dt = 0.01; % timestep in s
t_simu = 20; % time of the simulation in s
N = t_simu/dt; % number of steps
t = linspace(t0, t0 + t_simu, N);

%% Initial conditions

x0 = X_simu(id_initial,:)'; % initial position in m
v0 = V_simu(id_initial,:)'; % initial translation velocity in m/s
q0 = Q_simu(id_initial,:)';
omega0 = W_simu(id_initial, :)';

% Print initial values
disp("Initial values")
disp(t0)
disp(x0)
disp(v0)
disp(q0)
disp(omega0)

% Transformation matrix omega <-> q_dot
A0 = 0.5*mat4x3_ini(-q0(2), -q0(3), -q0(4), q0(1), -q0(4), q0(3), q0(4), q0(1), -q0(2), -q0(3), q0(2), q0(1));
q0_dot = A0*omega0;

% initial state vector
X0 = [x0(1), x0(2), x0(3), v0(1), v0(2), v0(3), q0(1), q0(2), q0(3), q0(4), q0_dot(1), q0_dot(2), q0_dot(3), q0_dot(4)]; 

%% Solving the system

[tout, Xout] = ode45(@DiffEq3D, t, X0);

%% Results

L_carre = zeros(size(tout)); % angular momentum quared
E_tot = zeros(size(tout)); % Total energy
norm_quat = zeros(size(tout));
Euler_angles = zeros(3, length(tout));

% Plot the trajectory
figure
h_cm = animatedline('LineWidth', 1, 'Color', 'black');
h_rocket = animatedline('Linewidth', 2, 'Color', 'red');
legend('center of mass', 'Rocket')
%set(gca, 'XLim', [-1.5 1.5], 'YLim', [-1.5 1.5], 'ZLim', [-1.5 1.5]);
axis equal
view(0,0); %view(60, 22);
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
set(gca,'FontSize',12)
grid on

for i=1:length(tout)
    
    v = Xout(i,4:6)'; % speed
    q = Xout(i,7:10)'; % orientation quaternion
    norm_quat(i) = QuatCheckNorm(q);
    q_dot = Xout(i,11:14)'; 
    omegas = omega_from_q(q, q_dot);% proper angular speed vector (B1)
    A = Quat2mat(q); % transformation matrix (base B0 to B1)
    
    Euler_angles(1,i) = atan2(2*(q(1)*q(4) + q(2)*q(3)), 1 - 2*(q(3)^2 + q(4)^2));
    Euler_angles(2,i) = asin(2*(q(1)*q(3) - q(4)*q(2)));
    Euler_angles(3,i) = atan2(2*(q(1)*q(2) + q(3)*q(4)), 1 - 2*(q(2)^2 + q(3)^2));
    
    sqrt(0.5*rho_atm*Cd*S*Dgp/Iz);
    
    % Expression of the basis vectors of the rocket (base B1) in the base B0
    x = A*[1 0 0]';
    y = A*[0 1 0]';
    z = A*[0 0 1]';
    
    L_carre(i) = (I(1,1)*omegas(1))^2 + (I(2,2)*omegas(2))^2 + (I(3,3)*omegas(3))^2; % squared angular moment in proper frame
    E_tot(i) = 0.5*m*(v(1)^2 + v(2)^2 + v(3)^2) + 0.5*(I(1,1)*omegas(1)^2 + I(2,2)*omegas(2)^2 + I(3,3)*omegas(3)^2); % total kinetic energy
    omegas = A*omegas; % angular speed in B0
    omegas_norm = sqrt(omegas(1)^2 + omegas(2)^2 + omegas(3)^2);
    
    
    addpoints(h_cm, Xout(i,1), Xout(i,2), Xout(i,3)); hold on;
    addpoints(h_rocket, [Xout(i,1) + Dtop*z(1), Xout(i,1) - Dbot*z(1)], [Xout(i,2) + Dtop*z(2), Xout(i,2) - Dbot*z(2)], [Xout(i,3) + Dtop*z(3), Xout(i,3) - Dbot*z(3)]);
    
    pause(0.01);
    if (i == N) 
        break
    end
    clearpoints(h_rocket);
   
end


% Plot vz as a function of time
figure
plot(tout, Xout(:,6), 'bx')
xlabel('time [s]')
ylabel('v_z [m/s]')
set(gca,'FontSize',12)
grid on


% Plot the Euler angles
figure
plot(tout, rad2deg(Euler_angles(1,:)), 'bx-'); hold on
plot(tout, rad2deg(Euler_angles(2,:)), 'gx-'); hold on
plot(tout, rad2deg(Euler_angles(3,:)), 'rx-')
xlabel('time [s]')
ylabel('Euler angles [deg]')
legend('\psi', '\theta', '\phi')
title('Euler angles')
set(gca,'FontSize',12)
grid on

figure
title('Euler angles')
subplot(2,1,1)
plot(tout, rad2deg(Euler_angles(1,:)), 'bx-')
xlabel('time [s]')
ylabel('Euler angle [deg]')
legend('\psi')
set(gca,'FontSize',12)
grid on
subplot(2,1,2)
plot(tout, rad2deg(Euler_angles(2,:)), 'gx-'); hold on
plot(tout, rad2deg(Euler_angles(3,:)), 'rx-')
xlabel('time [s]')
ylabel('Euler angles [deg]')
legend('\theta', '\phi')
set(gca,'FontSize',12)
grid on


% Plot angular momentum and energy
figure
subplot(2,1,1)
plot(tout, L_carre, 'bx-')
xlabel('time [s]')
ylabel('L^2 [kg^2m^4/s^2]')
set(gca,'FontSize',12)
grid on
subplot(2,1,2)
plot(tout, E_tot, 'bx-')
xlabel('time [s]')
ylabel('E [J]')
set(gca,'FontSize',12)
grid on


% Plot norm of the orientation quaternion
figure
plot(tout, norm_quat, 'bx-')
xlabel('time [s]')
ylabel('quaternion norm')
set(gca,'FontSize',12)
grid on

%% Functions
function out = rad(alpha)
out = pi*alpha/180;
end

function out = deg(alpha)
out = 180*alpha/pi;
end

function mat = Quat2mat(q)
a = q(1);
b = q(2);
c = q(3);
d = q(4);

% rotation matrix    
mat = mat3_ini(2*(a^2 + b^2) -1, 2*(b*c - a*d), 2*(a*c + b*d), 2*(a*d + b*c), 2*(a^2 + c^2) -1, 2*(c*d - a*b), 2*(b*d - a*c), 2*(a*b + c*d), 2*(a^2 + d^2) -1);
end

function out=mat3_ini(a1, a2, a3, b1, b2, b3, c1, c2, c3)
out = zeros(3,3);
out(1, 1) = a1;
out(1, 2) = a2;
out(1, 3) = a3;
out(2, 1) = b1;
out(2, 2) = b2;
out(2, 3) = b3;
out(3, 1) = c1;
out(3, 2) = c2;
out(3, 3) = c3;
end

function out = mat4x3_ini(a, b, c, d, e, f, g, h, i, j, k, l)
out = zeros(4,3);
out(1,1) = a;
out(1,2) = b;
out(1,3) = c;
out(2,1) = d;
out(2,2) = e;
out(2,3) = f;
out(3,1) = g;
out(3,2) = h;
out(3,3) = i;
out(4,1) = j;
out(4,2) = k;
out(4,3) = l;
end

function A = Surf_cylinder_proj(R, h, gamma)
A = pi*R^2*sin(gamma) + 2*R*h*cos(gamma);
end

%% 3D differential equation

function dXdt = DiffEq3D(t, X)
% I, T, alpha, m, g, Dbot, Ig, S, rho_atm, Cd, Dgp, R, h, C2

% state variables
[T, a, p, rho, nu] = atmosphere(X(3), Environment)
[M,dMdt,Cm,~,I_L,~,I_R,~] = Mass_Properties(t, obj.Rocket,'NonLinear');
[CNa, Xcp,CNa_bar,CP_bar] = normalLift(obj.Rocket, alpha_cm, 1.1,...
                Mach, angle(3), 1);

I = inertia_fill_cylinder(M, Rocket.motor_length, Rocket.motor_dia / 2);
T = Thrust(t, obj.Rocket);
alpha = ???
m = M;
g = 9.80665;
Dbot = 
Ig =
S = 
rho_atm = rho;
Cd = norm([X(4), X(5), X(6)]) / a;
Dgp = abs(Xcp - Cm);
R = 
h = 
C2 = 

x1 = X(1);
x2 = X(2);
x3 = X(3);
v1 = X(4);
v2 = X(5);
v3 = X(6);
q1 = X(7);
q2 = X(8);
q3 = X(9);
q4 = X(10);
q1_dot = X(11);
q2_dot = X(12);
q3_dot = X(13);
q4_dot = X(14);


% algebra
q = [q1, q2, q3, q4];
%QuatNorm = QuatCheckNorm(q);
A = Quat2mat(q); % transformation matrix (base B0 to B1)
% Expression of the basis vectors of the rocket (base B1) in the base B0
x = A*[1 0 0]';
x = x/norm(x);
y = A*[0 1 0]';
y = y/norm(y);
z = A*[0 0 1]';
z = z/norm(z);
% direction of the thrust (base B0)
dir_thrust = A*[sin(alpha) 0 cos(alpha)]';
dir_thrust = dir_thrust/norm(dir_thrust);
% norm of the speed
v = sqrt(v1^2 + v2^2 + v3^2);
% cross product between rocket axis z and v
z_cross_v = cross(z, [v1 v2 v3]');
% express it in base B1
z_cross_v = A\z_cross_v;


% Dynamical computation of the cross section
delta = acos(dot(z, [v1 v2 v3]')/v);
if(delta <= pi/2)
    gamma = delta;
else
    if(delta > pi/2)
     gamma = pi - delta;   
    end
end
gamma = pi/2 - gamma; % angle between axis of the rocket and perpendicular to its speed
S = Surf_cylinder_proj(R, h, gamma);


% Definition of the forces
F1 = T*dir_thrust(1) - 0.5*rho_atm*Cd*S*v*v1;
F2 = T*dir_thrust(2) - 0.5*rho_atm*Cd*S*v*v2;
F3 = -m*g + T*dir_thrust(3) - 0.5*rho_atm*Cd*S*v*v3;


% solving system for translation
dXdt(1) = v1;
dXdt(2) = v2;
dXdt(3) = v3;
dXdt(4) = F1/m;
dXdt(5) = F2/m;
dXdt(6) = F3/m;

% Solving system for rotations

% Baumgarte constants
alpha_const = 50;
beta_const = 50;
% alpha = beta -> critical damping

% solving trivial part of the quaternion system
dXdt(7) = q1_dot;
dXdt(8) = q2_dot;
dXdt(9) = q3_dot;
dXdt(10) = q4_dot;

% Computing omega from the q_dots
omega = omega_from_q([q1, q2, q3, q4]', [q1_dot, q2_dot, q3_dot, q4_dot]');
omega1 = omega(1);
omega2 = omega(2);
omega3 = omega(3);

% Definition of the torques
M1 = Dgp*0.5*rho_atm*Cd*S*v*z_cross_v(1); 
M2 = T*Dbot*sin(alpha) + Dgp*0.5*rho_atm*Cd*S*v*z_cross_v(2);
M3 = Dgp*0.5*rho_atm*Cd*S*v*z_cross_v(3) - C2*omega3; 
M = [M1, M2, M3]';

% Euler equations
omega_dot = I\(M - cross(omega, I*omega));
omega1_dot = omega_dot(1);
omega2_dot = omega_dot(2);
omega3_dot = omega_dot(3);

% Computing "acceleration" of the quaternion
q1_dot_dot = 0.5*(-q2_dot*omega1 - q3_dot*omega2 - q4_dot*omega3 - q2*omega1_dot - q3*omega2_dot - q4*omega3_dot);
q2_dot_dot = 0.5*(q1_dot*omega1 - q4_dot*omega2 + q3_dot*omega3 + q1*omega1_dot - q4*omega2_dot + q3*omega3_dot);
q3_dot_dot = 0.5*(q4_dot*omega1 + q1_dot*omega2 - q2_dot*omega3 + q4*omega1_dot + q1*omega2_dot - q2*omega3_dot);
q4_dot_dot = 0.5*(-q3_dot*omega1 + q2_dot*omega2 + q1_dot*omega3 - q3*omega1_dot + q2*omega2_dot + q1*omega3_dot);

% Lagrange multiplier
lambda = - (q1_dot^2 + q2_dot^2 + q3_dot^2 + q4_dot^2 + 2*alpha_const*(q1*q1_dot + q2*q2_dot + q3*q3_dot + q4*q4_dot) + 0.5*beta_const*(q1^2 + q2^2 + q3^2 + q4^2 -1) + q1*q1_dot_dot + q2*q2_dot_dot + q3*q3_dot_dot + q4*q4_dot_dot)/(q1^2 + q2^2 + q3^2 +q4^2);

% Solving the non trivial part of the system
dXdt(11) = 0.5*(-q2_dot*omega1 - q3_dot*omega2 - q4_dot*omega3 - q2*omega1_dot - q3*omega2_dot - q4*omega3_dot) + lambda*q1;
dXdt(12) = 0.5*(q1_dot*omega1 - q4_dot*omega2 + q3_dot*omega3 + q1*omega1_dot - q4*omega2_dot + q3*omega3_dot) + lambda*q2;
dXdt(13) = 0.5*(q4_dot*omega1 + q1_dot*omega2 - q2_dot*omega3 + q4*omega1_dot + q1*omega2_dot - q2*omega3_dot) + lambda*q3;
dXdt(14) = 0.5*(-q3_dot*omega1 + q2_dot*omega2 + q1_dot*omega3 - q3*omega1_dot + q2*omega2_dot + q1*omega3_dot) + lambda*q4;

% Transposition
dXdt = dXdt';
end

function out = QuatCheckNorm(q)
out = sqrt(q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2);
end

function out = norm(x)
out = sqrt(x(1)^2 + x(2)^2 + x(3)^2);
end