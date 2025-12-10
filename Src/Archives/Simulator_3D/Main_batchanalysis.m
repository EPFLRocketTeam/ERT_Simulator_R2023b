%% Rocket Simulator 3D-Effect of wind and Rail angle on apogee

% Initialize
close all; clear all;
addpath(genpath('../Declarations'),...
        genpath('../Functions'),...
        genpath('../Snippets'),...
        genpath('../Simulator_1D'));

% Rocket Definition
Rocket = rocketReader('Eiger_Kaltbrunn.txt');
Environment = environnementReader('Environment/Environnement_Definition_Kaltbrunn.txt');
simulationOutputs = SimOutputReader('Simulation/Simulation_outputs.txt');

simulatior3D = Simulator3D(Rocket, Environment, simulationOutputs);
%%
windSpace = [-5;-4;-3;-2;-1;0;1;2;3;4;5];
rail_AngleSpace = [0; 0.03;0.06;0.09];
%%
batchResults =[];
fh = figure; hold on;
for i=1:length(windSpace)
    for j=1:length(rail_AngleSpace)
        Environment = setfield(Environment, 'V_inf',windSpace(i) );
        Environment = setfield(Environment, 'railAngle', rail_AngleSpace(j));
        
        
         simulatior3D = Simulator3D(Rocket, Environment, simulationOutputs);
    
        %% ------------------------------------------------------------------------
        % 6DOF Rail Simulation
        %--------------------------------------------------------------------------

        [railTime, railState] = simulatior3D.RailSim();

        %% ------------------------------------------------------------------------
        % 6DOF Flight Simulation
        %--------------------------------------------------------------------------

         [flightTime, flightState, T21, S21, burnEventIndices] = simulatior3D.FlightSim([railTime(end) simulatior3D.Rocket.Burn_Time(end)], railState(end, 2));

        apogee_rec(i,j) = flightState(end, 3);

        plot(flightTime, flightState(:,3));

        drawnow;
        
        batchResults = [batchResults; flightTime(end), flightState(end, 3), windSpace(i), rail_AngleSpace(j)];
        
        
    end
end
%% Save Results

save('WindRailStudy','batchResults' )
%%
figure
surf(rail_AngleSpace,windSpace,apogee_rec)