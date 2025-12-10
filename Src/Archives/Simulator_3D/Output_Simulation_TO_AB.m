% Initialize
close all; 
%clear all;
addpath(genpath('../Declarations'),...
        genpath('../Functions'),...
        genpath('../Snippets'),...
        genpath('../Simulator_1D'));


Rocket_0 = rocketReader('BL_H4.txt');
simulationOutputs = SimOutputReader('Simulation/Simulation_outputs.txt');
name_of_environnment = 'Environment/Environnement_Definition_Wasserfallen.txt';

Environment = environnementReader(name_of_environnment);
simulatior3D = Simulator3D_AB_COM(Rocket_0, Environment, simulationOutputs);
    
    
cHeader = {'time(ms)' 'A(mg)' 'A(mg)' 'A(mg)' 'P(Pa)'}; %dummy header
commaHeader = [cHeader;repmat({','},1,numel(cHeader))]; %insert commaas
commaHeader = commaHeader(:)';
textHeader = cell2mat(commaHeader); %cHeader in text with commas
%write header to file
fid = fopen('log.csv','w'); 
fprintf(fid,'%s\n',textHeader);
fclose(fid);

[~,~,p,~, ~] = stdAtmos(Environment.startAltitude, Environment);


to_output = transpose([ 0.0: 0.01 : 19.99 ; 0*ones(1,2000) ; 0*ones(1,2000); -1000*ones(1,2000); p*ones(1,2000)]);

dlmwrite('log.csv',to_output,'-append');


[railTime, railState] = simulatior3D.RailSim();
[burnTime, burnState, burnTimeEvents, burnStateEvents, burnEventIndices] = simulatior3D.FlightSim([railTime(end) simulatior3D.Rocket.Burn_Time(end)], railState(end, 2));
[coastTime, coastState, coastTimeEvents, coastStateEvents, coastEventIndices] = simulatior3D.FlightSim([burnTime(end) 40], burnState(end, 1:3)', burnState(end, 4:6)', burnState(end, 7:10)', burnState(end, 11:13)');
flightTime = [burnTime; coastTime(2:end)];
flightState = [burnState; coastState(2:end, :)];
combinedRailFlightTime = [railTime;flightTime];
combinedRailFlightState = [railState;flightState(:,3) flightState(:,6)];
[drogueTime, drogueState, drogueTimeEvents, drogueStateEvents, drogueEventIndices] = simulatior3D.DrogueParaSim(flightTime(end), flightState(end,1:3)', flightState(end, 4:6)');
[mainChuteTime, mainChuteState, mainChuteTimeEvents, S4E, mainChuteEventsIndices] = simulatior3D.MainParaSim(drogueTime(end), drogueState(end,1:3)', drogueState(end, 4:6)');






