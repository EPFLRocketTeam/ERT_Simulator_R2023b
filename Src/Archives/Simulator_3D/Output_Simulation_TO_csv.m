% Initialize
close all; 
%clear all;
addpath(genpath('../Declarations'),...
        genpath('../Functions'),...
        genpath('../Snippets'),...
        genpath('../Simulator_1D'));

% Rocket Definition

cHeader = {'time(ms)' 'X(x)' 'X(y)' 'X(z)'  'A(x)' 'A(y)' 'A(z)', 'P(Pa)', 'T(K)'}; %dummy header
commaHeader = [cHeader;repmat({','},1,numel(cHeader))]; %insert commaas
commaHeader = commaHeader(:)';
textHeader = cell2mat(commaHeader); %cHeader in text with commas
%write header to file
fid = fopen('test.csv','w'); 
fprintf(fid,'%s\n',textHeader);
fclose(fid);

cHeader = {'time(ms)' 'X(x)' 'X(y)' 'X(z)'  'A(x)' 'A(y)' 'A(z)', 'P(Pa)', 'T(K)'}; %dummy header
commaHeader = [cHeader;repmat({','},1,numel(cHeader))]; %insert commaas
commaHeader = commaHeader(:)';
textHeader = cell2mat(commaHeader); %cHeader in text with commas
%write header to file
fid = fopen('log.csv','w'); 
fprintf(fid,'%s\n',textHeader);
fclose(fid);



Rocket_0 = rocketReader('WASSERFALLEN_FRANKENSTEIN.txt');
simulationOutputs = SimOutputReader('Simulation/Simulation_outputs.txt');
name_of_environnment = 'Environment/Environnement_Definition_Wasserfallen.txt';

Environment = environnementReader(name_of_environnment);
simulatior3D = Simulator3D_CAN_COM(Rocket_0, Environment, simulationOutputs);
[railTime, railState] = simulatior3D.RailSim();
[burnTime, burnState, burnTimeEvents, burnStateEvents, burnEventIndices] = simulatior3D.FlightSim([railTime(end) simulatior3D.Rocket.Burn_Time(end)], railState(end, 2));
[coastTime, coastState, coastTimeEvents, coastStateEvents, coastEventIndices] = simulatior3D.FlightSim([burnTime(end) 40], burnState(end, 1:3)', burnState(end, 4:6)', burnState(end, 7:10)', burnState(end, 11:13)');
flightTime = [burnTime; coastTime(2:end)];
flightState = [burnState; coastState(2:end, :)];
combinedRailFlightTime = [railTime;flightTime];
combinedRailFlightState = [railState;flightState(:,3) flightState(:,6)];
[drogueTime, drogueState, drogueTimeEvents, drogueStateEvents, drogueEventIndices] = simulatior3D.DrogueParaSim(flightTime(end), flightState(end,1:3)', flightState(end, 4:6)');
[mainChuteTime, mainChuteState, mainChuteTimeEvents, S4E, mainChuteEventsIndices] = simulatior3D.MainParaSim(drogueTime(end), drogueState(end,1:3)', drogueState(end, 4:6)');

T = readtable('test.csv', 'HeaderLines',1);
max = T{end,1};
bound = round(100*max)/100;
[~, index] = unique(T{:,1}); 
X = interp1(T{:,1}(index), T{:,2}(index) , 0.0: 0.01 : bound, 'pchip', 'extrap');  
Y = interp1(T{:,1}(index), T{:,3}(index) , 0.0: 0.01 : bound, 'pchip', 'extrap');  
Z = interp1(T{:,1}(index), T{:,4}(index) , 0.0: 0.01 : bound, 'pchip', 'extrap');  
AX = interp1(T{:,1}(index), T{:,5}(index) , 0.0: 0.01 : bound, 'pchip', 'extrap');  
AY = interp1(T{:,1}(index), T{:,6}(index) , 0.0: 0.01 : bound, 'pchip', 'extrap');  
AZ = interp1(T{:,1}(index), T{:,7}(index) , 0.0: 0.01 : bound, 'pchip', 'extrap');  
P = interp1(T{:,1}(index), T{:,8}(index) , 0.0: 0.01 : bound, 'pchip', 'extrap');  
T = interp1(T{:,1}(index), T{:,9}(index) , 0.0: 0.01 : bound, 'pchip', 'extrap');  

%cHeader = {'time(ms)' 'A(mg selon x)' 'A(mg y)' 'A(mg z)' 'P(Pa)'}; %dummy header
cHeader = {'time(ms)' 'X(m)' 'Y(m)' 'Z(m)'  'Ax(mg)' 'Ay(mg)' 'Az(mg)', 'P(Pa)', 'T(K)'}; %dummy header
commaHeader = [cHeader;repmat({','},1,numel(cHeader))]; %insert commaas
commaHeader = commaHeader(:)';
textHeader = cell2mat(commaHeader); %cHeader in text with commas
%write header to file
fid = fopen('result.csv','w'); 
fprintf(fid,'%s\n',textHeader);
fclose(fid);

[t,~,p,~, ~] = stdAtmos(Environment.startAltitude, Environment);


to_output = transpose([ 0.0: 0.01 : 19.99 ; 0*ones(1,2000) ; 0*ones(1,2000); 000*ones(1,2000);
     0*ones(1,2000) ; 0*ones(1,2000); -1000*ones(1,2000); p*ones(1,2000); t*ones(1,2000) ]);

dlmwrite('result.csv',to_output,'-append');


to_output = transpose([ 20.0: 0.01 : bound+20 ; X;Y;Z;AX ; AY ; AZ ; P ;T]);
            
dlmwrite('result.csv',to_output,'-append');




