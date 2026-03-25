function SimOutput = SimOutputReader(simOutputFilePath)

% -------------------------------------------------------------------------
% 1. Read Environnement
% -------------------------------------------------------------------------

rfid = fopen(simOutputFilePath);

if rfid < 0
   error('ERROR: SimOutput file name unfound.') 
end

while ~feof(rfid)

    line_content = fgetl(rfid);
    [line_id, line_data] = strtok(line_content);
    switch line_id
        
        case 'stabilityMargin'
            line_data_num = textscan(line_data, '%f');
            SimOutput.stabilityMargin = line_data_num{1}(1);
            
        case 'angleOfAttack'
            line_data_num = textscan(line_data, '%f');
            SimOutput.angleOfAttack = line_data_num{1}(1);
            
        case 'normalForceCoefficientSlope'
            line_data_num = textscan(line_data, '%f');
            SimOutput.normalForceCoefficientSlope = line_data_num{1}(1);
            
        case 'centerOfPressure'
            line_data_num = textscan(line_data, '%f');
            SimOutput.centerOfPressure = line_data_num{1}(1);    
        
        case 'dragCoefficient'
            line_data_num = textscan(line_data, '%f');
            SimOutput.dragCoefficient = line_data_num{1}(1);
           
        case 'mass'
            line_data_num = textscan(line_data, '%f');
            SimOutput.mass = line_data_num{1};  
            
        case 'centerOfMass'
            line_data_num = textscan(line_data, '%f');
            SimOutput.centerOfMass = line_data_num{1}(1);
            
        case 'inertiaLong'
            line_data_num = textscan(line_data,'%f');
            SimOutput.inertiaLong = line_data_num{1}(1);
            
        case 'inertiaRot'
            line_data_num = textscan(line_data, '%f');
            SimOutput.inertiaRot = line_data_num{1}(1);
            
        case 'flightPathAngle'
            line_data_num = textscan(line_data, '%f');
            SimOutput.flightPathAngle = line_data_num{1}(1);
            
        case 'noseAngleOfAttack'
            line_data_num = textscan(line_data, '%f');
            SimOutput.noseAngleOfAttack = line_data_num{1}(1);
            
        case 'noseFlightPathAngle'
            line_data_num = textscan(line_data, '%f');
            SimOutput.noseFlightPathAngle = line_data_num{1}(1);
            
        otherwise
            display(['ERROR: In simOutput definition, unknown line identifier: ' line_id]);
         
    end
end  