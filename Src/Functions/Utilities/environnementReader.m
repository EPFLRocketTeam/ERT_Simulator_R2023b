function Environnement = environnementReader(environnementFilePath,varargin)

% -------------------------------------------------------------------------
% 1. Read Environnement
% -------------------------------------------------------------------------

rfid = fopen(environnementFilePath);

if rfid < 0
   error('environnementReader:FileNotFound','ERROR: Environnement file name unfound.') 
end

while ~feof(rfid)

    line_content = fgetl(rfid);
    [line_id, line_data] = strtok(line_content);
    switch line_id
        
        case 'Temperature_Ground'
            line_data_num = textscan(line_data, '%f');
            Environnement.Temperature_Ground = line_data_num{1}(1);
            
        case 'Pressure_Ground'
            line_data_num = textscan(line_data, '%f');
            Environnement.Pressure_Ground = line_data_num{1}(1);
            
        case 'Humidity_Ground'
            line_data_num = textscan(line_data, '%f');
            Environnement.Humidity_Ground = line_data_num{1}(1);
        
        case 'V_inf'
            line_data_num = textscan(line_data, '%f');
            Environnement.V_inf = line_data_num{1}(1);
           
        case 'V_Azimuth'
            line_data_num = textscan(line_data, '%f');
            Environnement.V_Azimuth = line_data_num{1};  
            
        case 'Turb_I'
            line_data_num = textscan(line_data, '%f');
            Environnement.Turb_I = line_data_num{1}(1);
            
        case 'Turb_model'
            line_data_string = textscan(line_data,'%s');
            Environnement.Turb_model = line_data_string{1}{1};
            
        case 'Rail_Length'
            line_data_num = textscan(line_data, '%f');
            Environnement.Rail_Length = line_data_num{1}(1);
            
        case 'Rail_Angle'
            line_data_num = textscan(line_data, '%f');
            Environnement.Rail_Angle = line_data_num{1}(1)/180*pi;
            
        case 'Rail_Azimuth'
            line_data_num = textscan(line_data, '%f');
            Environnement.Rail_Azimuth = line_data_num{1}(1)/180*pi;
            
        case 'Start_Altitude'
            line_data_num = textscan(line_data, '%f');
            Environnement.Start_Altitude = line_data_num{1}(1);
        
        case 'Start_Latitude'
            line_data_num = textscan(line_data, '%f');
            Environnement.Start_Latitude = line_data_num{1}(1);
            
        case 'Start_Longitude'
            line_data_num = textscan(line_data, '%f');
            Environnement.Start_Longitude = line_data_num{1}(1);
            
        case 'dTdh'
            line_data_num = textscan(line_data, '%f');
            Environnement.dTdh = line_data_num{1}(1);  
          
            %multilayerwind, number of layer , windlayer1, ..., windlayer n
            % windlayer: mesured_height, V_inf, V_Azimuth, Turb_I 
        case 'multilayerwind'
            line_data_string = textscan(line_data,'%s');
            Environnement.numberLayer = str2double(line_data_string{1}{1});
            i = 1: Environnement.numberLayer;
            layerHeight = i;
            layerSpeed = i;
            layerAzi = i;
            layerTurb = i;
            for i = 1: Environnement.numberLayer
                layerHeight(i)= str2double(line_data_string{1}{2+4*(i-1)});
                layerSpeed(i)= str2double(line_data_string{1}{3+4*(i-1)});
                layerAzi(i)= str2double(line_data_string{1}{4*i});
                layerTurb(i)= str2double(line_data_string{1}{1+4*i});
            end
            if nargin < 2
                % standard deviation for azimuth of wind speed
                % chosen to be 2 degrees for some reason
                azi_std = 2;
                for i = 1: Environnement.numberLayer
                    turb_std = layerSpeed(i) * layerTurb(i);
                    layerAzi(i) = normrnd(layerAzi(i), azi_std);
                    layerSpeed(i) = normrnd(layerSpeed(i), turb_std);
                end
            end
            axis = 0:10: 4000;
            Environnement.Vspeed = interp1(layerHeight,layerSpeed,axis, 'pchip', 'extrap');
            Environnement.Vazy = interp1(layerHeight,layerAzi,axis, 'pchip', 'extrap');
            Environnement.Vturb = interp1(layerHeight,layerTurb,axis, 'pchip', 'extrap');
            Environnement.Vdirx = cosd(Environnement.Vazy);
            Environnement.Vdiry = sind(Environnement.Vazy);
            Environnement.Vdirz = 0*cosd(Environnement.Vazy);
            
            
            
         %   hold on
          %  plot(axis,Environnement.Vspeed,'r')
           % plot(axis,Environnement.Vazy,'b')
            %plot(axis,Environnement.Vturb,'k')
            %xlim([0 4000])
            %ylim([-1 100])
            %hold off;
            Environnement.isWindLayered = 1;
            
        case 'map'
            line_data_string = textscan(line_data,'%s');
            map_name = line_data_string{1}{1};
            [Environnement.map_x, Environnement.map_y, Environnement.map_z]=xyz2grid(map_name);
            Environnement.map_x = Environnement.map_x-2648540;  % what are
            Environnement.map_y = Environnement.map_y-1195050;  % these constants
            Environnement.map_z = Environnement.map_z-Environnement.Start_Altitude;
            
        otherwise
            display(['ERROR: In environnement definition, unknown line identifier: ' line_id]);
         
    end
end

% -------------------------------------------------------------------------
% 2. Check undefined fields
% -------------------------------------------------------------------------

% The default values are taken from Environnement_Definition_EuRoC.txt

if ~isfield(Environnement,'Temperature_Ground')
    default_Temperature_Ground = 289.15;
    Environnement.Temperature_Ground = default_Temperature_Ground;
    warning('Missing field "Temperature_Ground"; defaulted to 289.15');
end
if ~isfield(Environnement,'Pressure_Ground')
    default_Pressure_Ground = 102400;
    Environnement.Pressure_Ground = default_Pressure_Ground;
    warning('Missing field "Pressure_Ground"; defaulted to 102400');
end
if ~isfield(Environnement,'Humidity_Ground')
    default_Humidity_Ground = 0.7;
    Environnement.Humidity_Ground = default_Humidity_Ground;
    warning('Missing field "Humidity_Ground"; defaulted to 0.7');
end
if ~isfield(Environnement,'Start_Altitude')
    default_Start_Altitude = 154;
    Environnement.Start_Altitude = default_Start_Altitude;
    warning('Missing field "Start_Altitude"; defaulted to 154');
end
if ~isfield(Environnement,'Start_Latitude')
    default_Start_Latitude = 39.393564;
    Environnement.Start_Latitude = default_Start_Latitude;
    warning('Missing field "Start_Latitude"; defaulted to 39.393564');
end
if ~isfield(Environnement,'Start_Longitude')
    default_Start_Longitude = -8.287676;
    Environnement.Start_Longitude = default_Start_Longitude;
    warning('Missing field "Start_Longitude"; defaulted to -8.287676');
end
if ~isfield(Environnement,'dTdh')
    default_dTdh = -9.5;
    Environnement.dTdh = default_dTdh;
    warning('Missing field "dTdh"; defaulted to -9.5');
end
if ~isfield(Environnement,'V_inf')
    default_V_inf = 2;
    Environnement.V_inf = default_V_inf;
    warning('Missing field "V_inf"; defaulted to 2');
end
if ~isfield(Environnement,'V_Azimuth')
    default_V_Azimuth = 250;
    Environnement.V_Azimuth = default_V_Azimuth;
    warning('Missing field "V_Azimuth"; defaulted to 250');
end
if ~isfield(Environnement,'Turb_I')
    default_Turb_I = 0;
    Environnement.Turb_I = default_Turb_I;
    warning('Missing field "Turb_I"; defaulted to 0');
end
if ~isfield(Environnement,'Turb_model')
    default_Turb_model = 'None';
    Environnement.Turb_model = default_Turb_model;
    warning('Missing field "Turb_model"; defaulted to "None"');
end
if ~isfield(Environnement,'Rail_Length')
    default_Rail_Length = 12;
    Environnement.Rail_Length = default_Rail_Length;
    warning('Missing field "Rail_Length"; defaulted to 12');
end
if ~isfield(Environnement,'Rail_Angle')
    default_Rail_Angle = 5;
    Environnement.Rail_Angle = default_Rail_Angle;
    warning('Missing field "Rail_Angle"; defaulted to 5');
end
if ~isfield(Environnement,'Rail_Azimuth')
    default_Rail_Azimuth = 156;
    Environnement.Rail_Azimuth = default_Rail_Azimuth;
    warning('Missing field "Rail_Azimuth"; defaulted to 156');
end
    
% -------------------------------------------------------------------------
% 3. Intrinsic parameters
% -------------------------------------------------------------------------
% 3.1 Environnement Viscosity
Tmp = xlsread('Snippets/Viscosity.xlsx');
Environnement.T_Nu = Tmp(:,1);
Environnement.Viscosity = Tmp(:,2);

% 3.2 Humidity Changes
p_ws = exp(77.345+0.0057*Environnement.Temperature_Ground-7235/Environnement.Temperature_Ground)/Environnement.Temperature_Ground^8.2;
p_a = Environnement.Pressure_Ground;
Environnement.Saturation_Vapor_Ratio = 0.62198*p_ws/(p_a-p_ws);

% 3.3 Wind direction
Environnement.V_dir = [cosd(Environnement.V_Azimuth);sind(Environnement.V_Azimuth); 0];
end

