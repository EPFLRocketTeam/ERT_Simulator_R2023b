function [time,Thrust,Info] = motorReader(motorFilePath)
% MOTORREADER extracts the raw motor data from RASP formated text file 
% named 'motorFilePath'.

% -------------------------------------------------------------------------
% 1. Read Motor
% -------------------------------------------------------------------------

rfid = fopen(motorFilePath);

% 1.1 Read Informations
lineContent = fgetl(rfid); % Read one line
Info = textscan(lineContent,'%s %f32 %f32 %s %f32 %f32 %s');

% 1.2 Read Thrust Informations
time = []; Thrust = []; % Initialization

while ~feof(rfid)   % Test end of file
    
    lineContent = fgetl(rfid); % Read one line
    Tmp = textscan(lineContent,'%f %f');
    time = [time Tmp{1}];
    Thrust = [Thrust Tmp{2}];
end
end

