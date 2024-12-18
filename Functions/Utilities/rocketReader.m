function Rocket = rocketReader(rocketFilePath)

% -------------------------------------------------------------------------
% 1. Read Rocket
% -------------------------------------------------------------------------

rfid = fopen(rocketFilePath);

if rfid < 0
   error('ERROR: Rocket file name unfound.') 
end
Rocket.isHybrid = 0;

% Rocket Motor on/off parameter
Rocket.motor_state = 'off';

while ~feof(rfid)

    line_content = fgetl(rfid);
    [line_id, line_data] = strtok(line_content);
    switch line_id
        
        % Integer number indicating how many stages (diameter changes) the
        % rocket has. Typically, a straight rocket will have only 3 stages:
        % tip, cone base and tail. A rocket with a boattail has one
        % additional stage.
        case 'stages'
            line_data_num = textscan(line_data, '%f');
            Rocket.stages = line_data_num{1}(1);
            
        % List containing as many numbers as defined by the 'stages' 
        % parameter. Each number indicates the diameter at that stage     
        case 'diameters'
            line_data_num = textscan(line_data, '%f');
            Rocket.diameters = line_data_num{1}';
            
        % List containing as many numbers as defined by the 'stages' 
        % parameter. Each number indicates the position from the rocket's
        % tip of the diameter change
        case 'stage_z'
            line_data_num = textscan(line_data, '%f');
            Rocket.stage_z = line_data_num{1}';
        
        % Indicates if the aerodynamics are computed with or without the 
        % cone. 'cone_mode' = 'on' indicates the cone is on the rocket, 
        % 'cone_mode = off' indicates the cone is removed from the rocket
        case 'cone_mode'
            line_data_string = textscan(line_data,'%s');
            Rocket.cone_mode = line_data_string{1}{1};    
            
        % Integer referring to the number of fins    
        case 'fin_n'
            line_data_num = textscan(line_data, '%f');
            Rocket.fin_n = line_data_num{1}(1);
        
        % distance of the fin's leading edge root from the rocket's tip    
        case 'fin_xt'
            line_data_num = textscan(line_data, '%f');
            Rocket.fin_xt = line_data_num{1}(1);
        
        % fin span    
        case 'fin_s'
            line_data_num = textscan(line_data, '%f');
            Rocket.fin_s = line_data_num{1}(1);
        
        % fin root chord    
        case 'fin_cr'
            line_data_num = textscan(line_data, '%f');
            Rocket.fin_cr = line_data_num{1}(1);    
        
        % fin tip chord    
        case 'fin_ct'
            line_data_num = textscan(line_data, '%f');
            Rocket.fin_ct = line_data_num{1}(1);
        
        % fin thickness    
        case 'fin_t'
            line_data_num = textscan(line_data, '%f');
            Rocket.fin_t = line_data_num{1}(1);
            
        % axial distance between the fin's leading edge root and tip        
        case 'fin_xs'
            line_data_num = textscan(line_data, '%f');
            Rocket.fin_xs = line_data_num{1}(1);
        
        % Fins Leading edge 
        case 'fin_L1'
            line_data_num = textscan(line_data, '%f');
            Rocket.fin_L1 = line_data_num{1}(1);    
        % Fins Trailing edge 
        case 'fin_L2'
            line_data_num = textscan(line_data, '%f');
            Rocket.fin_L2 = line_data_num{1}(1); 
            
        % number of lugs    
        case 'lug_n'
            line_data_num = textscan(line_data, '%f');
            Rocket.lug_n = line_data_num{1}(1);    
        
        % exposed lug surface    
        case 'lug_S'
            line_data_num = textscan(line_data, '%f');
            Rocket.lug_S = line_data_num{1}(1);
        
        % rocket empty mass    
        case 'rocket_m'
            line_data_num = textscan(line_data, '%f');
            Rocket.rocket_m = line_data_num{1}(1);
        
        % rocket empty inertia    
        case 'rocket_I'
            line_data_num = textscan(line_data, '%f');
            Rocket.rocket_I = line_data_num{1}(1);
        
        % rocket center of mass for empty rocket (PL + rocket without
        % motor)
        case 'rocket_cm'
            line_data_num = textscan(line_data, '%f');
            Rocket.rocket_cm = line_data_num{1}(1);
        
        % position of airbrakes from rocket's tip    
        case 'ab_x'
            line_data_num = textscan(line_data, '%f');
            Rocket.ab_x = line_data_num{1}(1);
        
        % number of airbrake fins    
        case 'ab_n'
            line_data_num = textscan(line_data, '%f');
            Rocket.ab_n = line_data_num{1}(1);
        
        % airbrake openeing angle    
        case 'ab_phi'
            line_data_num = textscan(line_data, '%f');
            Rocket.ab_phi = line_data_num{1}(1);    
        
        % motor file name (with extension)  
        %in case of a hybrid motor, is the propergol bloc
	    % so the closest to the end of the rocket
        case 'motor'
            line_data_string = textscan(line_data,'%s');
            Rocket.motor_ID = line_data_string{1}{1};
            
            
        % is the tank fuel part of a hybrid motor, so
	    %the furthest to the end of the rocket         
	    % Second parameter is 
	    %the distance of the valve between the 
	    %propergol bloc and the tank fuel
	    case 'hybr'
	        line_data_string = textscan(line_data,'%s');
	        Rocket.fuel_ID = line_data_string{1}{1};
	        Rocket.intermotor_d = str2double(line_data_string{1}{2});
	        Rocket.isHybrid = 1;    
        
            
            
        % motor thrust multiplication factor    
        case 'motor_fac'
            line_data_num = textscan(line_data,'%f');
            Rocket.motor_fac = line_data_num{1}(1);    
        
        % payload mass    
        case 'pl_mass'
            line_data_num = textscan(line_data,'%f');
            Rocket.pl_mass = line_data_num{1}(1);
        
        % main parachute S*CD (area times drag coefficient)     
        case 'para_main_SCD'
            line_data_num = textscan(line_data,'%f');
            Rocket.para_main_SCD = line_data_num{1}(1);
            
        % drogue parachute S*CD (area times drag coefficient)    
        case 'para_drogue_SCD'
            line_data_num = textscan(line_data,'%f');
            Rocket.para_drogue_SCD = line_data_num{1}(1);
           
        % main parachute deployment event altitude    
        case 'para_main_event'
            line_data_num = textscan(line_data,'%f');
            Rocket.para_main_event = line_data_num{1}(1); 
        
        % error factor on center of pressure position     
        case 'cp_fac'
            line_data_num = textscan(line_data,'%f');
            Rocket.cp_fac = line_data_num{1}(1);
        
        % error factor on normal lift coefficient derivative
        case 'CNa_fac'
            line_data_num = textscan(line_data,'%f');
            Rocket.CNa_fac = line_data_num{1}(1);
        
        % error factor on drag coefficient    
        case 'CD_fac'
            line_data_num = textscan(line_data,'%f');
            Rocket.CD_fac = line_data_num{1}(1);
        
        % Read inertia matrix
        case 'rocket_inertia'
            line_data_num = textscan(line_data, '%f');
            data = line_data_num{1}';
            inertia = [data(1:3); data(4:6); data(7:9)];
            Rocket.rocket_inertia = inertia;
        
        % Read tank length
        case 'tank_L'
            line_data_num = textscan(line_data,'%f');
            Rocket.tank_L = line_data_num{1}(1);
        
        % Read tank radius
        case 'tank_r'
            line_data_num = textscan(line_data,'%f');
            Rocket.tank_r = line_data_num{1}(1);
        
        % Read tank position (from the top of the rocket)
        case 'tank_z'
            line_data_num = textscan(line_data,'%f');
            Rocket.tank_z = line_data_num{1}(1);
                        
        otherwise
            display(['ERROR: In rocket definition, unknown line identifier: ' line_id]);
         
    end

end    
   
% -------------------------------------------------------------------------
% 2. Read Motor
% -------------------------------------------------------------------------

Rocket = motor2RocketReader(Rocket.motor_ID, Rocket);

% -------------------------------------------------------------------------
% 3. Checks
% -------------------------------------------------------------------------

if checkStages(Rocket)
    error('ERROR: Reading rocket definition file.')
end

if ~(strcmp(Rocket.cone_mode, 'on') || strcmp(Rocket.cone_mode, 'off'))
    error(['ERROR: Cone mode parameter ' Rocket.cone_mode ' unknown.']);
end

% -------------------------------------------------------------------------
% 4. Intrinsic parameters
% -------------------------------------------------------------------------

% 4.1 Maximum body diameter
Rocket.dm = Rocket.diameters(find(Rocket.diameters == max(Rocket.diameters), 1, 'first'));
% 4.2 Fin cord
Rocket.fin_c = (Rocket.fin_cr + Rocket.fin_ct)/2; 
% 4.3 Maximum cross-sectional body area
Rocket.Sm = pi*Rocket.dm^2/4; 
% 4.4 Exposed planform fin area
Rocket.fin_SE = (Rocket.fin_cr + Rocket.fin_ct )/2*Rocket.fin_s; 
% 4.5 Body diameter at middle of fin station
Rocket.fin_df = interp1(Rocket.stage_z, Rocket.diameters, Rocket.fin_xt+Rocket.fin_cr/2, 'linear'); 
% 4.6 Virtual fin planform area
Rocket.fin_SF = Rocket.fin_SE + 1/2*Rocket.fin_df*Rocket.fin_cr; 
% 4.8 Rocket Length
Rocket.L = Rocket.stage_z(end);

% 4.9 Rocket inertia + motor and tank inertia
[~, ~, ~, ~, I_L, ~, I_R, ~] = Mass_Properties(0, Rocket, 'Linear');
if ~isfield(Rocket, 'rocket_inertia')
    Rocket.rocket_inertia = [I_L, 0, 0; 0, I_L, 0; 0, 0, I_R];
end

% -------------------------------------------------------------------------
% 5. Sub-routines
% -------------------------------------------------------------------------

function flag = checkStages(Rocket)
    flag = 0;
    if ~(length(Rocket.diameters) == Rocket.stages && length(Rocket.stage_z) == Rocket.stages)
        flag = 1;
        display('ERROR: In rocket defintion, rocket diameters and/or stage_z are not equal in length to the announced stages.')
    elseif ~(Rocket.diameters(1) == 0 && Rocket.stage_z(1) == 0)
        flag = 1;
        display('ERROR: In rocket defintion, rocket must start with a point (diameters(1) = 0, stage_z(1) = 0)');
    end
end

end

