function Y = SimAPI(simulatior3D, Xid, Yid, X)
%SIMAPI evaluates the simulator using the given parameters.
%   INPUTS: 
%       simulatior3D      Simulator object containing the base values of the parameters
%       Xid         IDs of the parameters that will change during the SA.
%       Yid         IDs of the desired outputs
%       X           Values of the parameter corresponding to Xid (one sample per column)
%   OUTPUT:
%       Y           Outputs corresponding to Yid (one output set per column)

% Implemented outputs
Yim_rail = ["Veor"];
Yimp_apogee = ["apogee" "t@apogee" "Vmax" "Vmax@t" "Cdmax" "a_max" "margin_min" "CNa_min" "MarCNa_min" "MarCNa_av"];
Yimp_landing = ["landing_azi" "landing_drift"];

if ~(any(ismember(Yid, Yimp_landing)) || any(ismember(Yid, Yimp_apogee)) || any(ismember(Yid, Yimp_landing)))
    print("Error: no given output id is implemented");
end

% Running the code
o = length(Yid);
N = size(X,2);
Y = NaN(o, N);


parfor idx_sim = 1:N
    % Time 
    t_s0 = tic;

    % Set parameters
    simulationObj_i = setParam(simulatior3D, Xid, X(:, idx_sim));
    
    % Rail simulation
    [railTime, railState] = simulationObj_i.RailSim();
    
    if any(ismember(Yid, Yimp_apogee)) || any(ismember(Yid, Yimp_landing))
        % Thrust phase
        [burnTime, burnState, ~, ~, ~] = simulationObj_i.FlightSim([railTime(end) simulationObj_i.Rocket.Burn_Time(end)], railState(end, 2));
        
        % Ballistic phase
        [coastTime, coastState, ~, ~, ~] = simulationObj_i.FlightSim([burnTime(end) 40], burnState(end, 1:3)', burnState(end, 4:6)', burnState(end, 7:10)', burnState(end, 11:13)');
        
        flightTime = [burnTime; coastTime(2:end)];
        flightState = [burnState; coastState(2:end, :)];
        combinedRailFlightTime = [railTime;flightTime];
        combinedRailFlightState = [railState;flightState(:,3) flightState(:,6)];
    end
    
    if any(ismember(Yid, Yimp_landing))
        % Drogue parachut descent 
        [T3, S3, ~, ~, ~] = simulationObj_i.DrogueParaSim(flightTime(end), flightState(end,1:3)', flightState(end, 4:6)');
        % Main parachute descent
        [~, mainChuteState, ~, ~, ~] = simulationObj_i.MainParaSim(T3(end), S3(end,1:3)', S3(end, 4:6)');
    end
    
    % Extracting the outputs into the output matrix
    for i=1:o
        id = Yid(i);
        switch id
            case "Veor"
                Y(i,idx_sim) = railState(end,2);
            case "apogee" 
                Y(i,idx_sim) = flightState(end,3);
            case "t@apogee" 
                Y(i,idx_sim) = flightTime(end);
            case "Vmax" 
                Y(i,idx_sim) = max(flightState(:,6));
            case "Vmax@t" 
                [~, tvmax] = max(flightState(:,6));
                Y(i,idx_sim) = tvmax;
            case "Cdmax" 
                Y(i,idx_sim) = max(simulationObj_i.simAuxResults.Cd);
            case "a_max" 
                Y(i,idx_sim) = max(diff(combinedRailFlightState(:,2))./diff(combinedRailFlightTime));
            case "margin_min" 
                Y(i,idx_sim) = min(simulationObj_i.simAuxResults.Margin);
            case "CNa_min"
                Y(i,idx_sim) = min(simulationObj_i.simAuxResults.Cn_alpha);
            case "MarCNa_min"
                Y(i,idx_sim) = min(simulationObj_i.simAuxResults.Margin.*simulationObj_i.simAuxResults.Cn_alpha);
            case "MarCNa_av"
                Y(i,idx_sim) = mean(simulationObj_i.simAuxResults.Margin.*simulationObj_i.simAuxResults.Cn_alpha);
            case "landing_drift"
                Y(i,idx_sim) = sqrt(mainChuteState(end,1).^2 + mainChuteState(end,2).^2);
            case "landing_azi"
                if (mainChuteState(:,1) == 0)
                    Y(i,idx_sim) = mod(sign(mainChuteState(end,2)) * 90, 360);
                else
                    Y(i,idx_sim) = mod(atand(mainChuteState(end,2)/mainChuteState(end,1)) - 90*(sign(mainChuteState(end,1)) - 1) + 180, 360);
                end
            otherwise
                error("Error: the output " + id + " is not implemented");
        end
    end
    %Time
    if mod(idx_sim, 1) == 0
         disp("     Simulation " +  num2str(idx_sim) + "/" + num2str(N) + " done. [" + num2str(toc(t_s0)) + " s]");
    end
end

