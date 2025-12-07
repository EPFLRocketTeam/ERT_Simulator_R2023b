%% === Inertial Coupling Simulation ===

% This script performs inertial coupling simulation using data from a previous simulation.
% Ensure you have run a simulation and have S2, T2 variables in the workspace.

% Index of the frame to begin with
initialFrameId = 100;

% Use previous simulation data
% State vector format: S_dot = [X_dot; V_dot; Q_dot; W_dot];
statePrevious = S2;
timePrevious = T2;
positionPrevious = statePrevious(:, 1:3);
velocityPrevious = statePrevious(:, 4:6);
quaternionPrevious = statePrevious(:, 7:10);
angularVelocityPrevious = statePrevious(:, 11:13);

%% Rocket Parameters
thrustMagnitude = 3736; % thrust in N (reference: 2600)
thrustDeviationAngle = degToRad(0); % thrust deviation angle in radians
rocketMass = 75; % rocket mass in kg
centerOfMassFromTop = 3.285; % position of center of mass measured from top in m
rocketDiameter = 0.185; % rocket diameter in m
rocketRadius = 0.5 * rocketDiameter; % radius in m
rocketLength = 4.115; % rocket length in m
distanceToBottom = rocketLength - centerOfMassFromTop; % distance from CoM to bottom
distanceToTop = centerOfMassFromTop; % distance from CoM to top
crossSectionalArea = pi * rocketRadius^2; % cross-sectional area of rocket
dragCoefficient = 0.35; % drag coefficient
staticMargin = 2.5; % static margin
distanceCoMtoCP = staticMargin * rocketDiameter; % distance between CoM and center of pressure

%% Environment Parameters
airDensity = 1.225; % air density in kg/m^3
gravity = 9.81; % gravity in m/s^2

%% Inertia Properties
% Moment of inertia for a cylinder with two distinct homogeneous parts
momentInertiaPerpendicular = (1/6) * rocketMass * (distanceToTop^2 + distanceToBottom^2) ...
                           + rocketMass * rocketRadius^2 / 4;
momentInertiaAxial = 0.5 * rocketMass * rocketRadius^2; % around axis of symmetry
inertiaTensorDiagonal = diag([momentInertiaPerpendicular, momentInertiaPerpendicular, momentInertiaAxial]);

% Apply rotation for misalignment between inertia and geometric axes
inertiaMisalignmentAngle = degToRad(1); % misalignment angle in radians
rotationMatrix = create3x3Matrix(cos(inertiaMisalignmentAngle), 0, sin(inertiaMisalignmentAngle), ...
                                 0, 1, 0, ...
                                 -sin(inertiaMisalignmentAngle), 0, cos(inertiaMisalignmentAngle));
inertiaTensor = rotationMatrix * inertiaTensorDiagonal * inv(rotationMatrix);
rollDampingCoefficient = 0; % roll damping coefficient (can be set to 0.1 for damping)

%% Simulation Parameters
initialTime = timePrevious(initialFrameId);
timeStep = 0.01; % time step in seconds
simulationDuration = 20; % total simulation time in seconds
numSteps = simulationDuration / timeStep;
timeVector = linspace(initialTime, initialTime + simulationDuration, numSteps);

%% Initial Conditions
initialPosition = positionPrevious(initialFrameId, :)';
initialVelocity = velocityPrevious(initialFrameId, :)';
initialQuaternion = quaternionPrevious(initialFrameId, :)';
initialAngularVelocity = angularVelocityPrevious(initialFrameId, :)';

% Display initial values
disp("=== Initial Conditions ===")
disp("Initial time: " + initialTime + " s")
disp("Initial position: " + mat2str(initialPosition') + " m")
disp("Initial velocity: " + mat2str(initialVelocity') + " m/s")
disp("Initial quaternion: " + mat2str(initialQuaternion'))
disp("Initial angular velocity: " + mat2str(initialAngularVelocity') + " rad/s")

% Calculate initial quaternion derivative from angular velocity
omegaToQuatDotMatrix = 0.5 * create4x3Matrix(-initialQuaternion(2), -initialQuaternion(3), -initialQuaternion(4), ...
                                              initialQuaternion(1), -initialQuaternion(4), initialQuaternion(3), ...
                                              initialQuaternion(4), initialQuaternion(1), -initialQuaternion(2), ...
                                              -initialQuaternion(3), initialQuaternion(2), initialQuaternion(1));
initialQuaternionDot = omegaToQuatDotMatrix * initialAngularVelocity;

% Initial state vector
initialStateVector = [initialPosition(1), initialPosition(2), initialPosition(3), ...
                     initialVelocity(1), initialVelocity(2), initialVelocity(3), ...
                     initialQuaternion(1), initialQuaternion(2), initialQuaternion(3), initialQuaternion(4), ...
                     initialQuaternionDot(1), initialQuaternionDot(2), initialQuaternionDot(3), initialQuaternionDot(4)];

%% Baumgarte Stabilization Parameters (for quaternion constraint)
baumgarteAlpha = 50;
baumgarteBeta = 50; % Critical damping when alpha = beta

%% Solve the System
[tout, stateOut] = ode45(@(t, x) rocketDynamics3D(t, x, ...
    inertiaTensor, thrustMagnitude, thrustDeviationAngle, rocketMass, gravity, ...
    distanceToBottom, crossSectionalArea, airDensity, dragCoefficient, ...
    distanceCoMtoCP, rocketRadius, rocketLength, rollDampingCoefficient, ...
    baumgarteAlpha, baumgarteBeta), timeVector, initialStateVector);

%% Post-Processing and Visualization
angularMomentumSquared = zeros(size(tout)); % squared angular momentum
totalEnergy = zeros(size(tout)); % total mechanical energy
quaternionNorm = zeros(size(tout)); % norm of quaternion (should be ~1)
eulerAngles = zeros(3, length(tout)); % Euler angles (psi, theta, phi)

%% Trajectory Animation
figure('Name', 'Rocket Trajectory', 'Position', [100, 100, 800, 600]);
centerOfMassLine = animatedline('LineWidth', 1.5, 'Color', 'blue', 'DisplayName', 'Center of Mass');
rocketBodyLine = animatedline('LineWidth', 2.5, 'Color', 'red', 'DisplayName', 'Rocket Body');
legend('Location', 'best');
axis equal;
view(45, 30); % 3D view
xlabel('x [m]', 'FontSize', 12);
ylabel('y [m]', 'FontSize', 12);
zlabel('z [m]', 'FontSize', 12);
title('Rocket Trajectory and Orientation', 'FontSize', 14);
grid on;
box on;

for i = 1:length(tout)
    % Extract current state
    currentVelocity = stateOut(i, 4:6)';
    currentQuaternion = stateOut(i, 7:10)';
    quaternionNorm(i) = computeQuaternionNorm(currentQuaternion);
    currentQuaternionDot = stateOut(i, 11:14)';
    
    % Compute angular velocity and rotation matrix
    currentAngularVelocity = angularVelocityFromQuaternion(currentQuaternion, currentQuaternionDot);
    rotationMatrixCurrent = quaternionToRotationMatrix(currentQuaternion);
    
    % Compute Euler angles
    eulerAngles(1, i) = atan2(2 * (currentQuaternion(1) * currentQuaternion(4) + currentQuaternion(2) * currentQuaternion(3)), ...
                              1 - 2 * (currentQuaternion(3)^2 + currentQuaternion(4)^2)); % psi (yaw)
    eulerAngles(2, i) = asin(2 * (currentQuaternion(1) * currentQuaternion(3) - currentQuaternion(4) * currentQuaternion(2))); % theta (pitch)
    eulerAngles(3, i) = atan2(2 * (currentQuaternion(1) * currentQuaternion(2) + currentQuaternion(3) * currentQuaternion(4)), ...
                              1 - 2 * (currentQuaternion(2)^2 + currentQuaternion(3)^2)); % phi (roll)
    
    % Compute conserved quantities
    angularMomentumSquared(i) = (inertiaTensor(1, 1) * currentAngularVelocity(1))^2 + ...
                                (inertiaTensor(2, 2) * currentAngularVelocity(2))^2 + ...
                                (inertiaTensor(3, 3) * currentAngularVelocity(3))^2;
    
    totalEnergy(i) = 0.5 * rocketMass * sum(currentVelocity.^2) + ...
                     0.5 * (inertiaTensor(1, 1) * currentAngularVelocity(1)^2 + ...
                            inertiaTensor(2, 2) * currentAngularVelocity(2)^2 + ...
                            inertiaTensor(3, 3) * currentAngularVelocity(3)^2);
    
    % Get rocket axis in global frame
    rocketAxis = rotationMatrixCurrent * [0; 0; 1];
    
    % Update animation
    addpoints(centerOfMassLine, stateOut(i, 1), stateOut(i, 2), stateOut(i, 3));
    addpoints(rocketBodyLine, [stateOut(i, 1) + distanceToTop * rocketAxis(1), stateOut(i, 1) - distanceToBottom * rocketAxis(1)], ...
                                [stateOut(i, 2) + distanceToTop * rocketAxis(2), stateOut(i, 2) - distanceToBottom * rocketAxis(2)], ...
                                [stateOut(i, 3) + distanceToTop * rocketAxis(3), stateOut(i, 3) - distanceToBottom * rocketAxis(3)]);
    
    pause(0.01);
    if i == numSteps
        break;
    end
    clearpoints(rocketBodyLine);
end

%% Visualization Functions
createVerticalVelocityPlot(tout, stateOut);
createEulerAnglesPlot(tout, eulerAngles);
createConservationPlots(tout, angularMomentumSquared, totalEnergy);
createQuaternionNormPlot(tout, quaternionNorm);

%% Helper Functions
function radians = degToRad(degrees)
    radians = pi * degrees / 180;
end

function degrees = radToDeg(radians)
    degrees = 180 * radians / pi;
end

function rotationMatrix = quaternionToRotationMatrix(quaternion)
    a = quaternion(1);
    b = quaternion(2);
    c = quaternion(3);
    d = quaternion(4);
    
    rotationMatrix = create3x3Matrix(2*(a^2 + b^2) - 1, 2*(b*c - a*d), 2*(a*c + b*d), ...
                                     2*(a*d + b*c), 2*(a^2 + c^2) - 1, 2*(c*d - a*b), ...
                                     2*(b*d - a*c), 2*(a*b + c*d), 2*(a^2 + d^2) - 1);
end

function matrix = create3x3Matrix(a11, a12, a13, a21, a22, a23, a31, a32, a33)
    matrix = [a11, a12, a13;
              a21, a22, a23;
              a31, a32, a33];
end

function matrix = create4x3Matrix(a11, a12, a13, a21, a22, a23, a31, a32, a33, a41, a42, a43)
    matrix = [a11, a12, a13;
              a21, a22, a23;
              a31, a32, a33;
              a41, a42, a43];
end

function projectedArea = cylinderProjectedArea(radius, height, angle)
    % Calculate projected area of cylinder at given angle
    projectedArea = pi * radius^2 * sin(angle) + 2 * radius * height * cos(angle);
end

function angularVelocity = angularVelocityFromQuaternion(quaternion, quaternionDot)
    % Compute angular velocity from quaternion and its derivative
    % Relationship: omega = 2 * Q(q)^T * q_dot
    % where Q(q) is the quaternion multiplication matrix
    
    Q = [quaternion(1), -quaternion(2), -quaternion(3), -quaternion(4);
         quaternion(2),  quaternion(1), -quaternion(4),  quaternion(3);
         quaternion(3),  quaternion(4),  quaternion(1), -quaternion(2);
         quaternion(4), -quaternion(3),  quaternion(2),  quaternion(1)];
    
    omega4 = 2 * Q' * quaternionDot;
    angularVelocity = omega4(2:4); % Extract spatial components
end

%% 3D Dynamics Function
function stateDerivative = rocketDynamics3D(time, state, ...
    inertiaTensor, thrustMagnitude, thrustAngle, mass, gravity, ...
    distanceToBottom, referenceArea, airDensity, dragCoefficient, ...
    distanceCoMtoCP, radius, length, rollDamping, baumgarteAlpha, baumgarteBeta)
    
    % Unpack state vector
    position = state(1:3);
    velocity = state(4:6);
    quaternion = state(7:10);
    quaternionDot = state(11:14);
    
    % Compute rotation matrix from current quaternion
    rotationMatrix = quaternionToRotationMatrix(quaternion);
    
    % Get rocket axis direction in global frame
    rocketAxis = rotationMatrix * [0; 0; 1];
    rocketAxis = rocketAxis / norm(rocketAxis);
    
    % Thrust direction in global frame
    thrustDirectionLocal = [sin(thrustAngle); 0; cos(thrustAngle)];
    thrustDirectionGlobal = rotationMatrix * thrustDirectionLocal;
    thrustDirectionGlobal = thrustDirectionGlobal / norm(thrustDirectionGlobal);
    
    % Current speed magnitude
    speed = norm(velocity);
    
    % Cross product between rocket axis and velocity (in global frame)
    axisVelocityCross = cross(rocketAxis, velocity);
    
    % Express cross product in body frame
    axisVelocityCrossBody = rotationMatrix' * axisVelocityCross;
    
    % Dynamic cross-sectional area based on angle of attack
    angleOfAttack = acos(dot(rocketAxis, velocity) / speed);
    if angleOfAttack > pi/2
        angleOfAttack = pi - angleOfAttack;
    end
    gamma = pi/2 - angleOfAttack; % angle between rocket axis and normal to velocity
    dynamicArea = cylinderProjectedArea(radius, length, gamma);
    
    % Aerodynamic and thrust forces
    dragForce = 0.5 * airDensity * dragCoefficient * dynamicArea * speed * velocity;
    thrustForce = thrustMagnitude * thrustDirectionGlobal;
    gravityForce = [0; 0; -mass * gravity];
    
    totalForce = thrustForce - dragForce + gravityForce;
    
    % Initialize derivative vector
    stateDerivative = zeros(14, 1);
    
    % Translational dynamics
    stateDerivative(1:3) = velocity;
    stateDerivative(4:6) = totalForce / mass;
    
    % Rotational dynamics (trivial part for quaternion)
    stateDerivative(7:10) = quaternionDot;
    
    % Compute angular velocity
    angularVelocity = angularVelocityFromQuaternion(quaternion, quaternionDot);
    omegaX = angularVelocity(1);
    omegaY = angularVelocity(2);
    omegaZ = angularVelocity(3);
    
    % Aerodynamic and thrust torques
    torqueAeroX = distanceCoMtoCP * 0.5 * airDensity * dragCoefficient * dynamicArea * speed * axisVelocityCrossBody(1);
    torqueAeroY = distanceCoMtoCP * 0.5 * airDensity * dragCoefficient * dynamicArea * speed * axisVelocityCrossBody(2);
    torqueAeroZ = distanceCoMtoCP * 0.5 * airDensity * dragCoefficient * dynamicArea * speed * axisVelocityCrossBody(3);
    
    torqueThrustY = thrustMagnitude * distanceToBottom * sin(thrustAngle);
    torqueDampingZ = -rollDamping * omegaZ;
    
    totalTorque = [torqueAeroX; torqueThrustY + torqueAeroY; torqueAeroZ + torqueDampingZ];
    
    % Euler's equations for rotational dynamics
    angularVelocityDot = inertiaTensor \ (totalTorque - cross(angularVelocity, inertiaTensor * angularVelocity));
    omegaDotX = angularVelocityDot(1);
    omegaDotY = angularVelocityDot(2);
    omegaDotZ = angularVelocityDot(3);
    
    % Quaternion second derivatives (unconstrained)
    q1 = quaternion(1); q2 = quaternion(2); q3 = quaternion(3); q4 = quaternion(4);
    q1Dot = quaternionDot(1); q2Dot = quaternionDot(2); q3Dot = quaternionDot(3); q4Dot = quaternionDot(4);
    
    q1DotDot = 0.5 * (-q2Dot*omegaX - q3Dot*omegaY - q4Dot*omegaZ - q2*omegaDotX - q3*omegaDotY - q4*omegaDotZ);
    q2DotDot = 0.5 * ( q1Dot*omegaX - q4Dot*omegaY + q3Dot*omegaZ + q1*omegaDotX - q4*omegaDotY + q3*omegaDotZ);
    q3DotDot = 0.5 * ( q4Dot*omegaX + q1Dot*omegaY - q2Dot*omegaZ + q4*omegaDotX + q1*omegaDotY - q2*omegaDotZ);
    q4DotDot = 0.5 * (-q3Dot*omegaX + q2Dot*omegaY + q1Dot*omegaZ - q3*omegaDotX + q2*omegaDotY + q1*omegaDotZ);
    
    % Lagrange multiplier for quaternion normalization constraint
    quaternionNormSquared = sum(quaternion.^2);
    constraintTerm = sum(quaternionDot.^2) + ...
                     2 * baumgarteAlpha * sum(quaternion .* quaternionDot) + ...
                     0.5 * baumgarteBeta * (quaternionNormSquared - 1) + ...
                     q1*q1DotDot + q2*q2DotDot + q3*q3DotDot + q4*q4DotDot;
    
    lambda = -constraintTerm / quaternionNormSquared;
    
    % Apply constraint to quaternion second derivatives
    stateDerivative(11) = q1DotDot + lambda * q1;
    stateDerivative(12) = q2DotDot + lambda * q2;
    stateDerivative(13) = q3DotDot + lambda * q3;
    stateDerivative(14) = q4DotDot + lambda * q4;
end

function normValue = computeQuaternionNorm(quaternion)
    normValue = sqrt(sum(quaternion.^2));
end

%% Visualization Functions
function createVerticalVelocityPlot(time, state)
    figure('Name', 'Vertical Velocity', 'Position', [100, 100, 600, 400]);
    plot(time, state(:, 6), 'b-', 'LineWidth', 2);
    xlabel('Time [s]', 'FontSize', 12);
    ylabel('Vertical Velocity v_z [m/s]', 'FontSize', 12);
    title('Vertical Velocity vs Time', 'FontSize', 14);
    grid on;
    box on;
end

function createEulerAnglesPlot(time, eulerAngles)
    figure('Name', 'Euler Angles', 'Position', [100, 100, 900, 600]);
    
    subplot(2, 1, 1);
    plot(time, rad2deg(eulerAngles(1, :)), 'b-', 'LineWidth', 2);
    xlabel('Time [s]', 'FontSize', 12);
    ylabel('Yaw (\psi) [deg]', 'FontSize', 12);
    title('Yaw Angle', 'FontSize', 14);
    grid on;
    box on;
    
    subplot(2, 1, 2);
    plot(time, rad2deg(eulerAngles(2, :)), 'r-', 'LineWidth', 2);
    hold on;
    plot(time, rad2deg(eulerAngles(3, :)), 'g-', 'LineWidth', 2);
    xlabel('Time [s]', 'FontSize', 12);
    ylabel('Angle [deg]', 'FontSize', 12);
    title('Pitch (\theta) and Roll (\phi)', 'FontSize', 14);
    legend('Pitch (\theta)', 'Roll (\phi)', 'Location', 'best');
    grid on;
    box on;
end

function createConservationPlots(time, angularMomentumSquared, totalEnergy)
    figure('Name', 'Conservation Quantities', 'Position', [100, 100, 900, 600]);
    
    subplot(2, 1, 1);
    plot(time, angularMomentumSquared, 'b-', 'LineWidth', 2);
    xlabel('Time [s]', 'FontSize', 12);
    ylabel('L^2 [kg^2 m^4 / s^2]', 'FontSize', 12);
    title('Squared Angular Momentum', 'FontSize', 14);
    grid on;
    box on;
    
    subplot(2, 1, 2);
    plot(time, totalEnergy, 'r-', 'LineWidth', 2);
    xlabel('Time [s]', 'FontSize', 12);
    ylabel('Total Energy [J]', 'FontSize', 12);
    title('Total Mechanical Energy', 'FontSize', 14);
    grid on;
    box on;
end

function createQuaternionNormPlot(time, quaternionNorm)
    figure('Name', 'Quaternion Norm', 'Position', [100, 100, 600, 400]);
    plot(time, quaternionNorm, 'b-', 'LineWidth', 2);
    hold on;
    plot([time(1), time(end)], [1, 1], 'r--', 'LineWidth', 1.5);
    xlabel('Time [s]', 'FontSize', 12);
    ylabel('Quaternion Norm', 'FontSize', 12);
    title('Quaternion Norm Constraint', 'FontSize', 14);
    legend('Actual Norm', 'Desired Norm (1.0)', 'Location', 'best');
    grid on;
    box on;
    ylim([0.95, 1.05]);
end