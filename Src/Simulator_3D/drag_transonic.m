function Value = drag_transonic(Rocket, alpha, Uinf, nu, a)
    % INPUTS:
    % - Rocket  : Rocket object
    % - alpha   : angle of attack [rad]
    % - Uinf    : Free stream velocity [m/s]
    % - nu      : Dynamic viscosity [m2/s]
    % - a       : Speed of sound [m/s]
    % OUTPUTS:
    % - CD      : Drag coefficient
    % REFERENCES:
    % - Drag Coefficient Prediction, Chapter 1 Preetam Sharma
    % Semester Project Report, Professor Flavio Noca, December 2021.

        % -------------------------------------------------------------------------
    % 0.1 Change of units from m to ft and inches
    % -------------------------------------------------------------------------
    
    % velocities in ft
    a = a * 3.28084;
    Uinf = Uinf * 3.28084;
    
    % nu in ft2/s
    mu = nu * 3.28084^2;

    % -------------------------------------------------------------------------
    % 0.2 Divergence 
    % -------------------------------------------------------------------------
    if Uinf < 0.1
        Uinf = 0.1;
    end 
    
    % -------------------------------------------------------------------------
    % 1. Parameters
    % -------------------------------------------------------------------------

    M = Uinf/a; % Mach number
    
    % Geometry of the rocket (inches, deg)
    L = Rocket.stagePositions(end) * 39.3701;                 % Lenght of the rocket
    Ln = Rocket.stagePositions(2) * 39.3701;     % Length of the rocket's nose
    d = Rocket.maxDiameter * 39.3701;           % Maximum rocket diameter
    r = d / 2;                                       % Maximum rocket radius
    db = Rocket.stageDiameters(end) * 39.3701;             % Base diameter of rocket at aft end
    Lo = L - Ln;                                     % Rocket length to body ratio, where the length is taken aft of the maximum body-diameter position: Assuming no boatrails TODO: calculate for boatrails presence
    Lb = L;                                          % Base position ATTENTION PEUT ETRE FAUX
    Le = L;                                          % Effective length of rocket ATTENTION PEUT ETRE FAUX
    Cr = Rocket.finRootChord * 39.3701;                % Root chord of fin
    Ct = Rocket.finTipChord * 39.3701;                % Tip chord of fin
    Nf = Rocket.numFins;                      % Fin number
    t = Rocket.finThickness* 39.3701;          % Maximum thickness of fin
    Xtc = Rocket.finLeadingEdgeLength;                                         % Distance from fin leading edge to maximum thickness
    
    % First useful parameters
    SB = 2 * pi * d / 2 * (L - Ln) + pi * d / 2 * Ln; % Total wetted surface area of body ATTENTION ICI PAS EXACTE
    Sf = Rocket.virtualFinArea;       % Total wetted surface area of fins % Other possible calculation: bs2*(L-Lf+Cr-bs2/np.tan(np.deg2rad(Thetaf)))
    Sr = SB + Sf;                                    % Total wetted surface area of rocket
    Xtcbar = Xtc / Cr;                               % Normalized distance from fin leading edge to maximum thickness
    
    %Material coefficients
    KB = 0.0004;
    KF = 0.0008;
    
    % May be of use
    Lf = Rocket.finRootPosition * 39.3701; %Position of LE of fin
    % Thetaf = atan((Rocket.get_finSweepDistance)/(Rocket.get_finSpanpan)) % Sweep angle (deg)
    % bs2 = Rocket.get_finSpanpan * 39.3701 % Lenght of fin

    % Limitation of model
    if Ln/Le > 0.6
        display("WARNING: In drag calculation, the transonic model gives a bad approximation.");
    else
        
        % ---------------------------------------------------------------------
        % 2. Body friction drag
        % ---------------------------------------------------------------------
        
        % equations p4 and p5
        
        % Compressible Reynolds Number
        Rnstar = a * M * L / (12 * nu)*(1 + 0.0283 * M - 0.043 * M^2 + 0.2107 * M^3 - 0.03829 * M^4 + 0.002709 * M^5);
        
        % Incompressible skin friction coefficient
        Cfstar = 0.037036 * Rnstar ^ (-0.155079);
        
        % Compressible skin friction coefficient
        Cf = Cfstar * (1 + 0.00798 * M - 0.1813 * M ^ (2) + 0.0632 * M ^ (3) - 0.00933 * M ^ (4) + 0.000549 * M ^ (5));
        
        % Incompressible skin friction coefficient with roughness
        Cftermstar = 1 / ((1.89 + 1.62 * log10 (L / KB)) ^ (2.5));
        
        % Compressible skin friction coefficient with roughness
        Cfterm = Cftermstar / ((1 + 0.2044 * M ^ (2)));
        
        % Final skin friction coefficient
        if Cf >= Cfterm
            Cffinal = Cf;
        else
            Cffinal = Cfterm;
        end
            
        % Total friction drag for body
        Cdfbody = Cffinal * (1 + 60 / ((L / d) ^ (3)) + 0.0025 * (L / d)) * (4 * SB) / (pi * d ^ (2));
        
        
        % ---------------------------------------------------------------------
        % 3.1 Fin friction drag (compressible)
        % ---------------------------------------------------------------------
        
        % Compressible Reynolds Number
        Rnstar = a * M * Cr / (12 * nu) * (1 + 0.0283 * M - 0.043 * M ^ 2 + 0.2107 * M ^ 3 - 0.03829 * M ^ 4 + 0.002709 * M ^ 5);
        
        % Incompressible skin friction coefficient
        Cfstar = 0.037036 * Rnstar ^ (-0.155079);
        
        % Compressible skin friction coefficient
        Cf = Cfstar * (1 + 0.00798 * M - 0.1813 * M ^ (2) + 0.0632 * M ^ (3) - 0.00933 * M ^ (4) + 0.000549 * M ^ (5));
        
        % Incompressible skin friction coefficient with roughness
        Cftermstar = 1 / ((1.89 + 1.62 * log10(Cr / KF)) ^ (2.5));
        
        % Compressible skin friction coefficient with roughness
        Cfterm = Cftermstar / ((1 + 0.2044 * M ^ (2)));
        
        % Final skin friction coefficient
        if Cf >= Cfterm
            Cffinal = Cf;
        else
            Cffinal = Cfterm;
        end
            
        % ---------------------------------------------------------------------
        % 3.2 Fin friction drag (incompressible)
        % ---------------------------------------------------------------------
            
        % Incompressible Reynolds Number
        Rn = a * M * Cr / (12 * nu);
        
        % Ratio of fin tip chord to root chord
        Lambda = Ct / Cr;
        
        % Average flat plate skin friction coefficient for each fin panel
        if Lambda == 0
            Cflam = Cffinal * (1 + (0.5646) / (log10(Rn)));
        elseif Lambda == 1
            Cflam = 0;
        else
            Cflam = Cffinal * ((log10(Rn)) ^ (2.6)) / ((Lambda ^ (2) - 1)) * ((Lambda ^ (2)) / ((log10(Rn * Lambda)) ^ (2.6)) - (1) / ((log10(Rn)) ^ (2.6)) + 0.5646 * ((Lambda ^ (2)) / ((log10(Rn * Lambda)) ^ (3.6)) - (1) / ((log10(Rn)) ^ (3.6))));
        end
        % Coefficient of friction drag for all fins
        Cdffins = Cflam * (1 + 60 * ((t) / (Cr)) ^ (4) + 0.8 * (1 + 5 * Xtcbar ^ (2)) * ((t) / (Cr))) * (4 * Nf * Sf) / (pi * d ^ (2));
        
        % ---------------------------------------------------------------------
        % 4. Protuberance friction drag
        % ---------------------------------------------------------------------
        
        Npro = 8; %Number of identical protuberances
        Lp = 0.194*Cr;
        tpro = t; %thick of prot
        Spro = 2*pi*(r+t)/4*Lp+2*pi/4*tpro*(2*r+tpro); %Wetted surface area of each protuberance
        Kp = 0.008;
        Lnosprot = Lf+Cr/2;
        Aprot = pi/4*tpro*(2*r+tpro);
        
        Rnstar = a*M*Lp/(12*mu)*(1 + 0.0283*M - 0.043*M^2 + 0.2107*M^3 - 0.03829*M^4 + 0.002709*M^5); %Compressible Reynolds Number
        Cfstar = 0.037036*Rnstar^(-0.155079); %Incompressible skin friction coefficient
        Cf = Cfstar*(1+0.00798*M-0.1813*M^(2)+0.0632*M^(3)-0.00933*M^(4)+0.000549*M^(5)); %Compressible skin friction coefficient
        Cftermstar = 1/((1.89+1.62*log10(Lp/Kp))^(2.5)); %Incompressible skin friction coefficient with roughness
        Cfterm = Cftermstar/((1+0.2044*M^(2))); %= Compressible skin friction coefficient with roughness
        if Cf >= Cfterm
            Cffinal = Cf; %Final skin friction coefficient
        else
            Cffinal = Cfterm;
        end
        Cfpro = 0.8151*Cffinal*(Lnosprot/Lp)^(-0.1243);
        Cdpro1 = Cfpro*(1+1.798*(sqrt(Aprot)/Lp)^(3/2))*4*Npro*Spro/(pi*d^2);
        
        %%% PROTUBERANCE FRICTION DRAG NUMBER 2 %%%
        
        Npro = 1; %Number of identical protuberances
        Lp = 0.4094488; %1.04cm
        tpro = 0.15748; %thick of prot 0.4cm
        Spro = 2*pi*(r+t)*Lp+2*pi*tpro*(2*r+tpro); %Wetted surface area of each protuberance
        Kp = 0.008;
        Lnosprot = L-Lp;
        Aprot = pi*tpro*(2*r+tpro);
        
        Rnstar = a*M*Lp/(12*mu)*(1 + 0.0283*M - 0.043*M^2 + 0.2107*M^3 - 0.03829*M^4 + 0.002709*M^5); %Compressible Reynolds Number
        Cfstar = 0.037036*Rnstar^(-0.155079); %Incompressible skin friction coefficient
        Cf = Cfstar*(1+0.00798*M-0.1813*M^(2)+0.0632*M^(3)-0.00933*M^(4)+0.000549*M^(5)); %Compressible skin friction coefficient
        Cftermstar = 1/((1.89+1.62*log10(Lp/Kp))^(2.5)); %Incompressible skin friction coefficient with roughness
        Cfterm = Cftermstar/((1+0.2044*M^(2))); %= Compressible skin friction coefficient with roughness
        if Cf >= Cfterm
            Cffinal = Cf; %Final skin friction coefficient
        else
            Cffinal = Cfterm;
        end
        Cfpro = 0.8151*Cffinal*(Lnosprot/Lp)^(-0.1243);
        Cdpro2 = Cfpro*(1+1.798*(sqrt(Aprot)/Lp)^(3/2))*4*Npro*Spro/(pi*d^2);
        
        % ---------------------------------------------------------------------
        % 5. Drag due to excresciencies
        % ---------------------------------------------------------------------
        
        % Coefficient for excrescencies drag increment
        if M < 0.78
            Ke = 0.00038;
        elseif M <= 1.04
            Ke = -0.4501*M^(4)+1.5954*M^(3)-2.1062*M^(2)+1.2288*M-0.26717;
        else
            Ke = 0.0002*M^(2)-0.0012*M+0.0018;
        end
        % Change is drag coefficient due to excrescencies 
        Cde = Ke*(4*Sr)/(pi*d^2);
        
        % ---------------------------------------------------------------------
        % 6. Total friction and interfernce drag coefficient
        % ---------------------------------------------------------------------
        
        % Mutual interference factor of fins and launch lug with body
        Kf = 1.04;
        Cdf = Cdfbody + Kf*Cdffins + Cde + Kf*Cdpro1 + Kf*Cdpro2;
        
        % ---------------------------------------------------------------------
        % 7. Base drag
        % ---------------------------------------------------------------------
        
        % Constants for subsonic flows
        Kb = 0.0274*atan((Lo/d)+0.0116);
        n = 3.6542*((Lo)/(d))^(-0.2733);
        
        % Base drag coefficient for subsonic flows (M < 0.6)
        CdbMinf06 = Kb*(((db)/(d))^(n))/(sqrt(Cdf));
        
        % Constants for other flows
        if M > 0.6 && M < 1.0
            fb = 1.0 + 215.8*(M-0.6)^(6.0);
        elseif M < 2.0
            fb = 2.0881*(M-1)^(3)-3.7938*(M-1)^(2)+1.4618*(M-1)+1.883917;
        else
            fb = 0.297*(M-2)^(3)-0.7937*(M-2)^(2)-0.1115*(M-2)+1.64006;
        end
            
        % Base drag coefficient for other flows (M > 0.6)
        CdbMsup06 = CdbMinf06*fb;
        
        % ---------------------------------------------------------------------
        % 8. Transonic wave drag
        % ---------------------------------------------------------------------
        
        % Transonic drag divergence Mach Number
        Md = -0.0156*(Ln/d)^2 + 0.136*(Ln/d) + 0.6817;
        
        % Constants for the wave drag
        if Ln/Le < 0.2
            a = 2.4;
            b = -1.05;
        else
            a = -321.94*((Ln)/(Le))^(2)+264.07*((Ln)/(Le))-36.348;
            b = 19.634*((Ln)/(Le))^(2)-18.369*((Ln)/(Le))+1.7434;
        end
        
        % Final Mach Number of Transonic Region
        Mf = a*((Le)/(d))^(b)+1.0275;
        
        % Other constants for the wave drag
        c = 50.676*((Ln)/(Lb))^(2)-51.734*((Ln)/(Lb))+15.642;
        g = -2.2538*((Ln)/(Lb))^(2)+1.3108*((Ln)/(Lb))-1.7344;
        
        % Maximum drag rise over transonic region
        if Le/d >= 6
            DeltaCdmax = c*(Le/d)^g;
        else
            DeltaCdmax = c*(6)^g;
        end
        
        % (p14)
        x = (M-Md)/(Mf-Md);
        F = -8.3474*x^(5)+24.543*x^(4)-24.946*x^(3)+8.6321*x^(2)+1.1195*x;
        
        % Transonic drag rise for given Mach Number (Eq1.8, p14)
        if M >= Md && M <= Mf
            DeltaCdt = DeltaCdmax*F;
        else
            DeltaCdt = 0;
        end
        
        % ---------------------------------------------------------------------
        % 9. Supersonic wave drag
        % ---------------------------------------------------------------------

        % Supersonic drag rise for given Mach Number (Eq1.9, p14)
        if M >= Mf
            DeltaCds = DeltaCdmax;
        else
            DeltaCds = 0;
        end
        
        % ---------------------------------------------------------------------
        % 10. Total drag coefficient
        % ---------------------------------------------------------------------
        
        Value = Cdfbody + Kf*Cdffins + Cde + CdbMsup06 + DeltaCdt + DeltaCds;
    end
end

