%% Instructions
% Defaults :
% density = 0.943
% C_N_alpha_F = 10.5
% V0 = railState(end)
% Defaults from 2019_SI_OR_0007_QR_OPEN_ROCKET_SIMULATION_R01 :
% emptyMass = 23.093
% emptyInertia = 29.977 (from 2*36.57-[~,~,~,I] = RocketInertia(0,Rocket,1))
% 36.57 is OR value. The above computes the empty rocket's inertia.
%
% Change to worst case scenario :
% Main_3D.m line 29 : change railState(end, 2) to 20
% WARNING : Remember to revert value for other cases !!
% Change V0 to 20 in this document
% Change density to 0.896 (=0.95*0.943)
% Change C_N_alpha_F to 9.45 (=0.9*10.5)
% Change emptyInertia to 31.4795 (=29.977*1.05)
% Change emptyCenterOfMass to 1.8244
% Value obtained by solving (23.093*x + 8.3420*2.679)/31 = 2.08 for x
% 23.093 is emptyMass
% 8.3420 is motor_mass
% 2.6790 is the distance of the motor's CM from the rocket's CP
% 31 is the rocket's mass off the rail
% 2.08 is the adjusted value (-5%) of the CP
%
% For max speed :
% Change
% Il = simulatior3D.simAuxResults.Il(1)
% W = simulatior3D.simAuxResults.CM(1)
% P = simulatior3D.simAuxResults.Xcp(1)
% to
% Il = simulatior3D.simAuxResults.Il(index)
% W = simulatior3D.simAuxResults.CM(index)
% P = simulatior3D.simAuxResults.Xcp(index)

addpath(genpath('../Simulator_3D'));
Main_3D;
close all

% For max speed :
[maxi,index] = max(flightState(:,6));

acc = diff(flightState(:,6))./diff(flightTime);
display(['Acceleration off rail : ' num2str(acc(1))]);
display(['Time off rail : ' num2str(railTime(end))]);
display(['Fuel burn off rail : ' num2str(Rocket.motor_mass+Rocket.emptyMass-simulatior3D.simAuxResults.Mass(1))]);
display(['Inertia off rail : ' num2str(simulatior3D.simAuxResults.Il(1))]);
display(['Max speed AGL : ' num2str(flightState(index,3))]);

X_F = 2.8;
% % Rocket length
L_r = simulatior3D.Rocket.stagePositions(end)
C_N_alpha = 12;
C_N_alpha_F = 10.5;
% % Propellant mass
m_p = 4.959; % CS_M1800
% % Burn time
t_b = 5.6;
% % Speed at end of rail
V0 = railState(end)
% Density
density = 0.943;


%% Calculations

Il = simulatior3D.simAuxResults.Il(1)
W = simulatior3D.simAuxResults.CM(1)
P = simulatior3D.simAuxResults.Xcp(1)
d = max(Rocket.stageDiameters)
A = pi/4*d^2
Margin = (P-W)/d

C_m_alpha = C_N_alpha*d/L_r*(P-W)

C_mq_fin = 2*C_N_alpha_F*(X_F-P)^2/L_r^2
C_mq_thrust = 4*m_p/t_b*(L_r-P)^2/(density*V0*L_r)
C_mq = C_mq_fin + C_mq_thrust
p = 0.5*density*V0^2
Damping = 0.25*L_r/V0*sqrt(p*A*L_r/Il)*C_mq/sqrt(C_m_alpha)

%% Display
display(['Margin : ' num2str(Margin)]);
display(['Damping ratio : ' num2str(Damping)]);