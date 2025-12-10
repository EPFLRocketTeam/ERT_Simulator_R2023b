function [Y, T] = SimAPI_NS(simulatior3D, Xid, X)
%SIMAPI_NS evaluates the simulator using the given parameters and monitor the time of execution.
%   INPUTS: 
%       simulatior3D      Simulator object containing the base values of the parameters
%       Xid         IDs of the parameters that will change during the SA.
%       X           Values of the parameter corresponding to Xid (one sample per column)
%   OUTPUT:
%       Y           Outputs corresponding to Yid (one output set per column)
% .     T           Time of execution of each phase.


% Running the code
N = size(X,2);
Y = NaN(2, 3, N);
T = NaN(5, N);

disp("Simulator: " + class(simulatior3D));
for idx_sim = 1:N

    % Set parameters
    simulatior3D = setParam(simulatior3D, Xid, X(:, idx_sim));
    
    % Rail simulation
    % Time 
    t_R0 = tic;
    [railTime, railState] = simulatior3D.RailSim();
    t_R = toc(t_R0);
    
    % Thrust phase
    t_A10 = tic;
    [burnTime, burnState, ~, ~, ~] = simulatior3D.FlightSim([railTime(end) simulatior3D.Rocket.Burn_Time(end)], railState(end, 2));
    t_A1 = toc(t_A10);
    
    % Ballistic phase
    t_A20 = tic;
    [coastTime, coastState, ~, ~, ~] = simulatior3D.FlightSim([burnTime(end) 40], burnState(end, 1:3)', burnState(end, 4:6)', burnState(end, 7:10)', burnState(end, 11:13)');
    t_A2 = toc(t_A20);
    
    flightTime = [burnTime; coastTime(2:end)];
    flightState = [burnState; coastState(2:end, :)];
    combinedRailFlightTime = [railTime;flightTime];
    combinedRailFlightState = [railState;flightState(:,3) flightState(:,6)];

    % Drogue parachut descent 
    t_D0 = tic;
    [T3, S3, ~, ~, ~] = simulatior3D.DrogueParaSim(flightTime(end), flightState(end,1:3)', flightState(end, 4:6)');
    t_D = toc(t_D0);
    
    % Main parachute descent
    t_M0 = tic;
    [~, mainChuteState, ~, ~, ~] = simulatior3D.MainParaSim(T3(end), S3(end,1:3)', S3(end, 4:6)');
    t_M = toc(t_M0);
    
    
    T(:, idx_sim) = [t_R, t_A1, t_A2, t_D, t_M];
    Y(1, :, idx_sim) = flightState(end,1:3)'; %position apogee
    Y(2, :, idx_sim) = mainChuteState(end,1:3)'; %position landing
    
    disp("Simulation " + num2str(idx_sim) + "/" + num2str(N) + " done. (" ...
         + num2str(t_R + t_A1 + t_A2 + t_D + t_M) + " s)");
    
end

