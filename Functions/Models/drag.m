function CD = drag(Rocket, alpha, Uinf, nu, a)
%L : Page du PDF issu du livre Mandell
%C : Code original
%J : Ligne du code d'ORK barrowmancalculator.java
%T : Page du PDF de thèse ORK
if Uinf < 0.1 %C
    Uinf = 0.1;
end
c = Rocket.fin_c; % fin cord
SF = Rocket.fin_SF; % Virtual fin planform area
Sm = Rocket.Sm; % maximum cross-sectional body area
df = Rocket.fin_df; % body diameter at middle of fin station
SE = Rocket.fin_SE; % Exposed planform fin area

% 2.1 Body L.464 Fig. 51
Rl = Rocket.stage_z(end)*Uinf/nu;
Rl_crit = 5e5; 
% 2.2 Fins
Rc = c*Uinf/nu; 
Rc_crit = 5.14e6;

% Skin friction coefficient T.54
Rs=60e-6; %Roughness height of regular paint
Rcrit = 51*(Rs/Rocket.L)^(-1.039); %Critical Reynold Number of roughness
if Rl<10^4
    Cf=1.48e-2;
elseif Rl<Rcrit
    Cf=(1.50*log(Rl)-5.6)^-2;
else
    Cf=0.032*(Rs/Rocket.L)^0.2;
end

M = Uinf / a; %C

% Compressibility corrections T.55 et J.623 
if M<1.1 
    c1=(1-0.1*M^2);
end

if M>0.9
    c2=1/(1+0.15*M^2)^0.58;
end

if M<0.9
    Cf = Cf* c1;
elseif M<1.1
    Cf = Cf * (c2 * (M-0.9)/0.2 + c1* (1.1-M)/0.2);
else 
    Cf = Cf* c2;
end

% Roughness correction J.649
if M<0.9
    roughnesscorrection = 1-0.1*M^2;
elseif M> 1.1
    roughnesscorrection = 1/(1+0.18*M^2);
else
    c1 = 1-0.1*0.9^2;
    c2 = 1/(1+0.18*1.1^2);
    roughnesscorrection= c2 * (M - 0.9) / 0.2 + c1 * (1.1 - M) / 0.2;
end

% Roughness limted :  
roughnesslimited= (0.032*(Rs/Rocket.L)^0.2)*roughnesscorrection; %J.470

if (Rl>1.0e6 && roughnesslimited>Cf) %J.484
    Cfc=roughnesslimited;
else
    Cfc=Cf;
end


dm = Rocket.dm; %C maximum rocket diameter

%Body pressure drag

%Shoulder pressure drag

%Boattail pressure drag

%Fin pressure drag

%AxialCD J.871

%Base drag J.838 & T.60
if M<=1
    base=0.12+0.13*M^2;
else
    base = 0.25/M;
end

%Total Drag
if Rocket.stage_z(2)/dm < 1.5 %C
    display('WARNING: In drag coefficient calculation, ogive cone ratio is out of bounds. Drag estimation cannot be trusted.');
    SsSm_noseogive=0;
else
    SsSm_noseogive= 2.67*Rocket.stage_z(2)/dm; %M.439
end

SsSm_nose=SsSm_noseogive; %Ogive car celui de ORK de Nordend mais possible de changer
SsSm_cyl=4*(Rocket.stage_z(end-1)-Rocket.stage_z(2))/dm; %M.438
db = Rocket.diameters(end-1); %base diameter
lt = Rocket.stage_z(end)-Rocket.stage_z(end-1); %Length of boattail
SsSm_boat=2/dm^2*(dm-db)*lt*sqrt(1+(dm-db)/(2*lt)^2); %M.441
SsSm = SsSm_nose +SsSm_cyl +SsSm_boat;
if strcmp(Rocket.cone_mode, 'on')
    SsSm = SsSm; % + 2.67*Rocket.stage_z(2)/dm; %Pour Nordend: Un peu mieux (20m plus proche) de pas le mettre
end

if Rocket.stage_z(2)/dm < 1.5
    display('WARNING: In drag coefficient calculation, ogive cone ratio is out of bounds. Drag estimation cannot be trusted.');
end

CDf_B=Cfc*(1+60/(Rocket.stage_z(end)/dm)^3+0.0025*(Rocket.stage_z(end)/dm))*SsSm; %M.431

CDb=base*(db/dm)^3/sqrt(CDf_B);%Mix entre J.804 et M.431 fonctionne aussi très bien mais pas bonne courbe de CD base*(1-De^2/Dref^2); %base*(db/dm)^3/sqrt(CDf_B);
%1:Nordend Centré : base*(Rocket.diameters(end-1)^2-(98*10^-3)^2)/dm^2; ==>
%4500 / WH : 10100
%2:Nordend décalé : base*(db/dm)^3/sqrt(CDf_B); ==> 3900m / WH 7600
%3:Nordend coherent mais U : 0.029*(db/dm)^3/sqrt(CDf_B); ==> 4740 / WH :
%9795
CD0_B=CDf_B+CDb;
CD0_F = Cfc*2*(1+2*Rocket.fin_t/c)*SF/(Sm);
CD0_FB = CD0_B + CD0_F;
CD=CD0_FB ;




% % 5. Drag at AoA %Source assez peu fiable + donne pas de résultat
% très différent
% % -------------------------------------------------------------------------
% 
% % 5.1 Body drag at AoA
% % 5.1.1 factor tables as seen in Fig. 35 and 36 on p. 405
% alpha = abs(alpha);
% etatab=[4 6 8 10 12 14 16 18 20 22 24;0.6 0.63 0.66 0.68 0.71 0.725 0.74 0.75 0.758 0.77 0.775];
% deltaktab=[4 6 8 10 12 14 16 18 20;0.78 0.86 0.92 0.94 0.96 0.97 0.975 0.98 0.982];
% etak = interp1(etatab(1,:),etatab(2,:),Rocket.stage_z(end)/dm,'linear','extrap');
% deltak=interp1(deltaktab(1,:),deltaktab(2,:),Rocket.stage_z(end)/dm,'linear','extrap');
% % 5.1.2 Compute body drag at angle of attack alpha
% % 5.1.2.1 x1 as defined by explanations of (eq 140, p 404)
% x1 = Rocket.stage_z(find(diff(Rocket.diameters)==0, 1, 'first'));
% % 5.1.2.2 x0 as in (eq 140, p404)
% x0 = 0.55*x1+0.36*Rocket.stage_z(end);
% % 5.1.2.3 Section Area at station x0
% S0 = pi*interp1(Rocket.stage_z, Rocket.diameters, x0, 'linear')^2/4;
% % 5.1.2.4 Body drag at low AoA (eq 139, p. 404)
% CDB_alpha = 2*deltak*S0/Sm*alpha*sin(alpha); %pourquoi *sin alpha ?
% tmp_stages = [x0, Rocket.stage_z(Rocket.stage_z>x0)];
% tmp_diameters = [interp1(Rocket.stage_z, Rocket.diameters, x0, 'linear'), Rocket.diameters(Rocket.stage_z>x0)];
% % 5.1.2.4 Body drag at high AoA (eq 142, p. 406) %Pas la même
% CDB_alpha = CDB_alpha + 2*alpha^2*sin(alpha)/Sm*etak*1.2*sum((tmp_diameters(1, end-1)+tmp_diameters(2:end))/2.*(tmp_stages(2:end)-tmp_stages(1:end-1)));
% 
% % 5.2 Fin drag at AoA
% % 5.2.1 Fin Exposed Surface Coefficient
% % TODO: Consider rocket roll for lateral exposed fin surface
% FESC = 2;
% % 5.2.2 induced fin drag, similar to (eq 145, p 413) DIFFERENT
% CDi = 1.2*alpha^2*SF/Sm*FESC;
% % 5.2.3 Interference coefficients as estimated by Hassan (eq 34 and 35, p
% % 12) based on Mandell Fig. 40 p 416.
% Rs = df/(2*Rocket.fin_s+df); % Total fin span ratio
% KFB = 0.8065*Rs^2+1.1553*Rs; % Interference of body on fin lift
% KBF = 0.1935*Rs^2+0.8174*Rs+1; % Interference of fins on body lift
% % 5.2.4 Interference Drag Coefficient (eq 146, p 415)
% DCDi = (KFB + KBF - 1)*3.12*SE/Sm*alpha^2*FESC; % Interference drag %different
% CDF_alpha = CDi + DCDi;
% 
% % 5.3 Total drag at AoA (eq 148, p 417)
% CD_alpha = CDB_alpha + CDF_alpha;
% % -------------------------------------------------------------------------
% % 7. Drag of tumbeling body (c.f. OpenRocket Documentation section 3.5) %NP
% % -------------------------------------------------------------------------
% fin_efficiency = [0.5, 1, 1.5, 1.41, 1.81, 1.73, 1.9, 1.85];
% CD_t_fin = 1.42*fin_efficiency(Rocket.fin_n);
% CD_t_body = 0.56;
% 
% CD_t = (SE*CD_t_fin+CD_t_body*dm*(Rocket.stage_z(end)-Rocket.stage_z(2)))/Sm;
% % -------------------------------------------------------------------------
% % 6. Subsonic drag coefficient
% % -------------------------------------------------------------------------
% CD = CD0_FB + CD_alpha;
% if CD > CD_t
%     CD = CD_t;
% end

CD=0.73*CD;

    % -------------------------------------------------------------------------
    % Transsonic and Supersonic drag coefficient
    % -------------------------------------------------------------------------
    % from here on the reference file is the semester report by Xavier Palle
    % and Jan Schulz:
    % "Calculation of the drag coefficient of a rocket at transonic and 
    % supersonic speed" (January 2022)

    Ln = Rocket.stage_z(2) * 39.3701;     % Length of the rocket's nose
    d = Rocket.dm * 39.3701;           % Maximum rocket diameter
    Le = Rocket.stage_z(end)* 39.3701;                % Effective lenght of the rocket

    % Transonic drag divergence Mach Number
    Md = -0.0156*(Ln/d)^2 + 0.136*(Ln/d) + 0.6817;

    % Constants for the wave drag
    if Ln/Le < 0.2
        a1 = 2.4;
        b = -1.05;
    else
        a1 = -321.94*((Ln)/(Le))^(2)+264.07*((Ln)/(Le))-36.348;
        b = 19.634*((Ln)/(Le))^(2)-18.369*((Ln)/(Le))+1.7434;
    end

    % Final Mach Number of Transonic Region
    Mf =1.5; %a1*((Le)/(d))^(b)+1.0275;

    % Mach number
    M = Uinf / a;

    % if Md < M && M < Mf
    % % -------------------------------------------------------------------------
    % % 7. Transsonic drag coefficient
    % % -------------------------------------------------------------------------   
    %     CD = drag_transonic(Rocket, alpha, Uinf, nu, a);

    if M >= Mf

    % -------------------------------------------------------------------------
    % 8. Supersonic drag coefficient
    % -------------------------------------------------------------------------

        % length of the nose
        Ln = Rocket.stage_z(2);
        % radius at the base of the nose
        rn = Rocket.diameters(2) / 2;
        % length of the body
        Lb = Rocket.stage_z(end) - Ln;

        % heat capacity ratio
        gamma=1.4;

        % (defined in eq 3.3, p 19)
        cp_sin_arr = load("data_Cp_conical_nose_sin.csv") ;

        % ---------------------------------------------------------------------
        % 3.1  Nose drag
        % ---------------------------------------------------------------------

        % 3.1.1  Pressure drag (eq 3.2, p 19)
        num_phi = 100;
        Phi = linspace(0.5*pi/num_phi,(num_phi-0.5)*pi/num_phi, num_phi);
        delta = atan(rn/Ln);
        delta_c = asin(cos(alpha)*sin(delta)*ones(1,num_phi) - sin(alpha)*cos(Phi)*cos(delta));
        CPn = sum(cp_sin(delta_c, M, cp_sin_arr))* Ln /num_phi /rn /cos(delta);

        % 3.1.2  Friction drag of nose + body (eq 3.4, p 19)
        if Rl < Rl_crit
            CFnb = CDf_B / (1+0.045*M^2)^(0.25);  % laminar flow
        else
            CFnb = CDf_B / (1+0.15*M^2)^(0.58);   % turbulent flow
        end


        % ---------------------------------------------------------------------
        % 3.2  Fin drag
        % ---------------------------------------------------------------------

        % 3.2.1  Pressure drag (eq 3.5, p 20 and eq 3.7, p 23)
        c1 = 2 / sqrt(M^2 - 1);
        c2 = ((M^2-2)^2 + gamma*M^4) / (2*(M^2-1)^2);
        j1 = Rocket.fin_t^2 /4 * (1/Rocket.fin_L1 + 1/Rocket.fin_L2);
        j2 = Rocket.fin_t^3 /8 * (1/Rocket.fin_L1^2 - 1/Rocket.fin_L2^2);
        if Rocket.fin_cr == Rocket.fin_ct
            CPf = (2*c1*j1 + 2*c2*j2) / Rocket.fin_cr;
        else
            CPf = (2*c1*j1 + 2*c2*j2) *log(Rocket.fin_ct/Rocket.fin_cr) / (Rocket.fin_ct - Rocket.fin_cr);
        end
        Phi = 2*pi/Rocket.fin_n *linspace(0, Rocket.fin_n-1, Rocket.fin_n);
        CPf = CPf + 2*c1 * sum((asin(sin(alpha) * sin(Phi))).^2);

        % 3.2.2  Friction drag (eq 3.4, p 19)
        if Rc < Rc_crit
            CFf = CD0_F / (1+0.045*M^2)^(0.25);  % laminar flow
        else
            CFf = CD0_F / (1+0.15*M^2)^(0.58);   % turbulent flow
        end


        % ---------------------------------------------------------------------
        % 3.3  Base drag
        % ---------------------------------------------------------------------

        % (eq 3.8, p 23)
        CB = 1 / (4*M);


        % ---------------------------------------------------------------------
        % 3.4  Body drag
        % ---------------------------------------------------------------------

        % 3.4.1  Pressure drag (eq 3.9, p 24)
        cp0 = 2 / (gamma*M^2) * (((gamma+1)*M^2/2)^(gamma/(gamma-1)) * ...
                                  ((gamma+1)/(2*gamma*M^2 - (gamma-1)))^(1/(gamma-1)) - 1);
        CPb = cp0 * 4*Lb / (3*pi*rn) * sin(alpha)^3;


        % ---------------------------------------------------------------------
        % 3.5  Summing up the coefficients
        % ---------------------------------------------------------------------

        % (eq 3.10, p 25)
        AB = pi * (Rocket.diameters(end) / 2)^2;
%         if Rocket.motor_mode == "on"
%             AB = AB - pi * (Rocket.motor_d / 2)^2;
%         end
        if M < 3
            CD_trans = drag_transonic(Rocket, alpha, Uinf, nu, a);
            CD_sup =  CPn + CFnb + CPb + (CPf+CFf) * c*Rocket.fin_s / Sm + CB * AB / Sm;
            CD = ((3-M)*CD_trans + (M-1.5)*CD_sup) / 1.5;
        else
            CD =  CPn + CFnb + CPb + (CPf+CFf) * c*Rocket.fin_s / Sm + CB * AB / Sm;
        end

    end


%CD=CD-0.2;
end