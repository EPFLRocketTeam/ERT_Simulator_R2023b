function Environnement = environnementReader(environnementFilePath,varargin)

% -------------------------------------------------------------------------
% 1. Read Environnement
% -------------------------------------------------------------------------

rfid = fopen(environnementFilePath);

if rfid < 0
   error('environnementReader:FileNotFound','ERROR: Environnement file name unfound.');
end

% Declare Environnement as an empty struct
Environnement = struct();

while ~feof(rfid)

    lineContent = fgetl(rfid);
    [lineId, lineData] = strtok(lineContent);
    switch lineId
        
        case 'Temperature_Ground'
            lineDataCells = textscan(lineData, '%s');
            lineDataNum = str2double(lineDataCells{1}(1));
            if isnan(lineDataNum)
                error('environnementReader:NaN', 'Environment definition : Temperature_Ground is not a number.');
            end
            Environnement.Temperature_Ground = lineDataNum;
            
        case 'Pressure_Ground'
            lineDataCells = textscan(lineData, '%s');
            lineDataNum = str2double(lineDataCells{1}(1));
            if isnan(lineDataNum)
                error('environnementReader:NaN', 'Environment definition : Pressure_Ground is not a number.');
            end
            Environnement.Pressure_Ground = lineDataNum;
            
        case 'Humidity_Ground'
            lineDataCells = textscan(lineData, '%s');
            lineDataNum = str2double(lineDataCells{1}(1));
            if isnan(lineDataNum)
                error('environnementReader:NaN', 'Environment definition : Humidity_Ground is not a number.');
            end
            Environnement.Humidity_Ground = lineDataNum;
        
        case 'V_inf'
            lineDataCells = textscan(lineData, '%s');
            lineDataNum = str2double(lineDataCells{1}(1));
            if isnan(lineDataNum)
                error('environnementReader:NaN', 'Environment definition : V_inf is not a number.');
            end
            Environnement.V_inf = lineDataNum;
           
        case 'V_Azimuth'
            lineDataCells = textscan(lineData, '%s');
            lineDataNum = str2double(lineDataCells{1}(1));
            if isnan(lineDataNum)
                error('environnementReader:NaN', 'Environment definition : V_Azimuth is not a number.');
            end
            Environnement.V_Azimuth = lineDataNum;  
            
        case 'Turb_I'
            lineDataCells = textscan(lineData, '%s');
            lineDataNum = str2double(lineDataCells{1}(1));
            if isnan(lineDataNum)
                error('environnementReader:NaN', 'Environment definition : Turb_I is not a number.');
            end
            Environnement.Turb_I = lineDataNum;
            
        case 'Turb_model'
            lineDataCells = textscan(lineData,'%s');
            Environnement.Turb_model = lineDataCells{1}{1};
            
        case 'Rail_Length'
            lineDataCells = textscan(lineData, '%s');
            lineDataNum = str2double(lineDataCells{1}(1));
            if isnan(lineDataNum)
                error('environnementReader:NaN', 'Environment definition : Rail_Length is not a number.');
            end
            Environnement.Rail_Length = lineDataNum;
            
        case 'Rail_Angle'
            lineDataCells = textscan(lineData, '%s');
            lineDataNum = str2double(lineDataCells{1}(1));
            if isnan(lineDataNum)
                error('environnementReader:NaN', 'Environment definition : Rail_Angle is not a number.');
            end
            Environnement.Rail_Angle = lineDataNum/180*pi;
            
        case 'Rail_Azimuth'
            lineDataCells = textscan(lineData, '%s');
            lineDataNum = str2double(lineDataCells{1}(1));
            if isnan(lineDataNum)
                error('environnementReader:NaN', 'Environment definition : Rail_Azimuth is not a number.');
            end
            Environnement.Rail_Azimuth = lineDataNum/180*pi;
            
        case 'Start_Altitude'
            lineDataCells = textscan(lineData, '%s');
            lineDataNum = str2double(lineDataCells{1}(1));
            if isnan(lineDataNum)
                error('environnementReader:NaN', 'Environment definition : Start_Altitude is not a number.');
            end
            Environnement.Start_Altitude = lineDataNum;
        
        case 'Start_Latitude'
            lineDataCells = textscan(lineData, '%s');
            lineDataNum = str2double(lineDataCells{1}(1));
            if isnan(lineDataNum)
                error('environnementReader:NaN', 'Environment definition : Start_Latitude is not a number.');
            end
            Environnement.Start_Latitude = lineDataNum;
            
        case 'Start_Longitude'
            lineDataCells = textscan(lineData, '%s');
            lineDataNum = str2double(lineDataCells{1}(1));
            if isnan(lineDataNum)
                error('environnementReader:NaN', 'Environment definition : Start_Longitude is not a number.');
            end
            Environnement.Start_Longitude = lineDataNum;
            
        case 'dTdh'
            lineDataCells = textscan(lineData, '%s');
            lineDataNum = str2double(lineDataCells{1}(1));
            if isnan(lineDataNum)
                error('environnementReader:NaN', 'Environment definition : dTdh is not a number.');
            end
            Environnement.dTdh = lineDataNum;  
          
            %multilayerwind, number of layer , windlayer1, ..., windlayer n
            % windlayer: mesured_height, V_inf, V_Azimuth, Turb_I 
        case 'multilayerwind'
            lineDataCells = textscan(lineData,'%s');
            Environnement.numberLayer = str2double(lineDataCells{1}{1});
            if isnan(Environnement.numberLayer)
                error('environnementReader:NaN', 'Environment definition : multilayerwind number of layers is not a number.');
            end
            i = 1: Environnement.numberLayer;
            layerHeight = i;
            layerSpeed = i;
            layerAzi = i;
            layerTurb = i;
            for i = 1: Environnement.numberLayer
                layerHeight(i)= str2double(lineDataCells{1}{2+4*(i-1)});
                if isnan(layerHeight(i))
                    error('environnementReader:NaN', 'Environment definition : multilayerwind layer height is not a number.');
                end

                layerSpeed(i)= str2double(lineDataCells{1}{3+4*(i-1)});
                if isnan(layerSpeed(i))
                    error('environnementReader:NaN', 'Environment definition : multilayerwind wind speed is not a number.');
                end

                layerAzi(i)= str2double(lineDataCells{1}{4*i});
                if isnan(layerAzi(i))
                    error('environnementReader:NaN', 'Environment definition : multilayerwind azimuth is not a number.');
                end

                layerTurb(i)= str2double(lineDataCells{1}{1+4*i});
                if isnan(layerTurb(i))
                    error('environnementReader:NaN', 'Environment definition : multilayerwind standard deviation is not a number.');
                end
            end
            if nargin < 2
                % standard deviation for azimuth of wind speed
                % chosen to be 2 degrees for some reason
                aziStd = 2;
                for i = 1: Environnement.numberLayer
                    turbStd = layerSpeed(i) * layerTurb(i);
                    layerAzi(i) = normrnd(layerAzi(i), aziStd);
                    layerSpeed(i) = normrnd(layerSpeed(i), turbStd);
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
            lineDataCells = textscan(lineData,'%s');
            map_name = lineDataCells{1}{1};
            [Environnement.map_x, Environnement.map_y, Environnement.map_z]=xyz2grid(map_name);
            Environnement.map_x = Environnement.map_x-2648540;  % what are
            Environnement.map_y = Environnement.map_y-1195050;  % these constants
            Environnement.map_z = Environnement.map_z-Environnement.Start_Altitude;
            
        otherwise
            display(['ERROR: In environnement definition, unknown line identifier: ' lineId]);
         
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
    default_Rail_Angle = 5/180*pi;
    Environnement.Rail_Angle = default_Rail_Angle;
    warning('Missing field "Rail_Angle"; defaulted to 5/180*pi');
end
if ~isfield(Environnement,'Rail_Azimuth')
    default_Rail_Azimuth = 156/180*pi;
    Environnement.Rail_Azimuth = default_Rail_Azimuth;
    warning('Missing field "Rail_Azimuth"; defaulted to 156/180*pi');
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

