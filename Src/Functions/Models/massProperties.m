function [M,dMdt,Cm,dCmdt,I_L,dI_Ldt,I_R,dI_Rdt] = massProperties(t,Rocket,Opt)
%Return the global mass properties of the rocket
%   [M,dMdt,Cm,dCmdt,I_L,dI_Ldt,I_R,dI_Rdt] = Mass_Properties_Lin(t,Rocket)
%   INPUT:
%   - t         Time [s]
%   - Rocket    Matlab structure containing all data [.]
%   OUTPUT:
%   - M         Mass [kg]
%   - dMdt      Derivative of Mass [kg/s]
%   - Cm        Center of Mass [m]
%   - dCmdt     Derivative of Center of Mass [m/s]
%   - I_L       Longitudinal Moment of Inertia [kgm^2]
%   - dI_Ldt    Derivative of L. Moment of Inertia [kgm^2/s]
%   - I_R       Rotational Moment of Inertia [kgm^2]
%   - dI_Rdt    Derivative of R. Moment of Inertia [kgm^2/s]

%--------------------------------------------------------------------------
% Mass
%--------------------------------------------------------------------------
if (Rocket.isHybrid == 0)
    
if strcmp(Opt, 'Linear')
    if t == 0
        dMdt = Rocket.propelMass/Rocket.burnTime;
        M = Rocket.emptyMass;       
    elseif t > Rocket.burnTime
        M = Rocket.emptyMass + Rocket.casingMass;
        dMdt = 0;
    else
        dMdt = Rocket.propelMass/Rocket.burnTime;
        M = Rocket.emptyMass+Rocket.motorMass-t*dMdt;
    end
elseif strcmp(Opt, 'NonLinear')
    if t == 0
        dMdt = Rocket.Thrust2dMass_Ratio*thrust(t,Rocket);
        M = Rocket.emptyMass;
    elseif t>Rocket.burnTime
        M = Rocket.emptyMass+Rocket.motorMass-Rocket.propelMass;
        dMdt = 0;
    else
        tt = linspace(0,t,500);
        Current_Impulse = trapz(tt,thrust(tt,Rocket));
        M = Rocket.emptyMass + Rocket.motorMass - ... 
        Rocket.Thrust2dMass_Ratio*Current_Impulse;
        dMdt = Rocket.Thrust2dMass_Ratio*thrust(t,Rocket);
    end
else
    error('Opt parameter should be Linear or Nonlinear')
end
%--------------------------------------------------------------------------
% Center of Mass
%--------------------------------------------------------------------------
% Centre de masse
Cm = (Rocket.emptyCenterOfMass*Rocket.emptyMass + ... 
    (M-Rocket.emptyMass)*(Rocket.totalLength-Rocket.motorLength/2))/M;
 
% Derivee centre de masse
dCmdt = (dMdt*(Rocket.totalLength-Rocket.motorLength/2)-dMdt*Cm)/M;

%--------------------------------------------------------------------------
% Moment of Inertia
%--------------------------------------------------------------------------
% I_L:
R_i = 0.005; % Diametre interieur grains (Tjr identique)
R_e = Rocket.motorDiameter/2; % Diametre exterieur grains

I_L_Casing = Rocket.casingMass*(Rocket.motorLength^2/12 + R_e^2/2); 

Grain_Mass = M-Rocket.emptyMass-Rocket.casingMass; % Masse des grains
I_L_Grain = Grain_Mass*(Rocket.motorLength^2/12 + (R_e^2+R_i^2)/4);

I_L = Rocket.emptyInertia + I_L_Casing + I_L_Grain + ...
    (Grain_Mass+Rocket.casingMass)*...
    (Rocket.totalLength-Cm-Rocket.motorLength/2)^2; % I + ... + Steiner

% dI_L/dt:
dI_L_Grain = dMdt*(Rocket.motorLength^2/12 + (R_e^2+R_i^2)/4);

dI_Ldt = dI_L_Grain+dMdt*(Rocket.totalLength-Cm-Rocket.motorLength/2)^2+...
    2*(Grain_Mass+Rocket.casingMass)*(Rocket.totalLength-Cm-Rocket.motorLength/2)*...
    dCmdt;

% I_R:
I_R = 1e6;

% dI_R/dt:
dI_Rdt = 0;

else
    
    if strcmp(Opt, 'Linear')
    
    if t == 0
        dMdt = (Rocket.propelMass)/Rocket.burnTime;
        M = Rocket.emptyMass;       
    elseif t > Rocket.burnTime
        M = Rocket.emptyMass + Rocket.casingMass;
        dMdt = 0;
    else
        dMdt = Rocket.propelMass /Rocket.burnTime;   
        M = Rocket.emptyMass+Rocket.motorMass-t*dMdt;
    end
elseif strcmp(Opt, 'NonLinear')
    if t == 0
        dMdt = Rocket.Thrust2dMass_Ratio*thrust(t,Rocket);
        M = Rocket.emptyMass ; 
    elseif t>Rocket.burnTime
        M = Rocket.emptyMass+Rocket.casingMass;
        dMdt = 0;
    else
        tt = linspace(0,t,500);
        Current_Impulse = trapz(tt,thrust(tt,Rocket));
        M = Rocket.emptyMass + Rocket.motorMass - ... 
        Rocket.Thrust2dMass_Ratio*Current_Impulse;
        dMdt = Rocket.Thrust2dMass_Ratio*thrust(t,Rocket);
    end
else
    error('Opt parameter should be Linear or Nonlinear')
    end
%--------------------------------------------------------------------------
% Center of Mass
%--------------------------------------------------------------------------
% Centre de masse

motorCenterOfMass = (Rocket.totalLength - Rocket.motorLengthPropel/2 )* (Rocket.motorMassPropel - t*(Rocket.massPropel/Rocket.burnTime));
motorCenterOfMassFuel = (Rocket.totalLength - Rocket.motorLengthFuel/2 -Rocket.motorLengthPropel - Rocket.interMotorDistance )* (Rocket.motorMassFuel - t*(Rocket.massFuel/Rocket.burnTime));
% Centre de masse

Cm = (Rocket.emptyCenterOfMass*Rocket.emptyMass +  motorCenterOfMass + motorCenterOfMassFuel )/M;
% D?riv?e centre de masse
%dcmdtn = (dMdt*(Rocket.totalLength-Rocket.motorLength)-dMdt*Cm)/M;

%Cm =(Rocket.emptyCenterOfMass*Rocket.emptyMass +  (M-Rocket.emptyMass)*(Rocket.totalLength-Rocket.motorLength/2))/M;
 
% Derivee centre de masse
dCmdt = (dMdt*(Rocket.totalLength-(Rocket.motorLength+Rocket.motorLengthFuel + Rocket.interMotorDistance))/2-dMdt*Cm)/M;

%--------------------------------------------------------------------------
% Moment of Inertia
%--------------------------------------------------------------------------
% I_L:
R_i = 0.005; % Diametre interieur grains (Tjr identique)
R_e = Rocket.motorDiameter/2; % Diametre exterieur grains

I_L_Casing = Rocket.casingMass*(Rocket.motorLength^2/12 + R_e^2/2); 

Grain_Mass = M-Rocket.emptyMass-Rocket.casingMass; % Masse des grains
I_L_Grain = Grain_Mass*(Rocket.motorLength^2/12 + (R_e^2+R_i^2)/4);

I_L = Rocket.emptyInertia + I_L_Casing + I_L_Grain + ...
    (Grain_Mass+Rocket.casingMass)*...
    (Rocket.totalLength-Cm-Rocket.motorLength/2)^2; % I + ... + Steiner

% dI_L/dt:
dI_L_Grain = dMdt*(Rocket.motorLength^2/12 + (R_e^2+R_i^2)/4);

dI_Ldt = dI_L_Grain+dMdt*(Rocket.totalLength-Cm-Rocket.motorLength/2)^2+...
    2*(Grain_Mass+Rocket.casingMass)*(Rocket.totalLength-Cm-Rocket.motorLength/2)*...
    dCmdt;

% I_R:
I_R = 1e6;

% dI_R/dt:
dI_Rdt = 0;
end
end

