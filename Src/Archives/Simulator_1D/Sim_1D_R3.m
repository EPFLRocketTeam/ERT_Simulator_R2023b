function [T, X, railTime, X1, flightTime, X2] = Sim_1D_R3(Rocket, Env, tspan, x0, AB_drag, theta, thrust_err, ab_err, EventType, EventVal, EventDir)

Option = odeset('RelTol', 1e-3, 'AbsTol', 1e-6);

switch(EventType)
    case 'Altitude'
        Option = odeset(Option, 'events', @AltEvent);
    case 'Velocity'
        Option = odeset(Option, 'events', @SpeedEvent);
    case 'None'
        Option = [];
    otherwise
        error(["Error: Invalid event argument. "...
        "In Sim_1D Event arguments are either "...
        "'Alitude', 'Velocity' or 'None'."]);
end

[T, X] = ode45(@(t,x) Rocket_Kinematic_R3(t, x, Rocket, Env, AB_drag, theta, 1+thrust_err, 1+ab_err),...
               tspan, x0, Option);
[railTime, X1] = ode45(@(t,x) Rocket_Kinematic_R3(t, x, Rocket, Env, AB_drag, theta, 1, 1+ab_err),...
               tspan, x0, Option);
[flightTime, X2] = ode45(@(t,x) Rocket_Kinematic_R3(t, x, Rocket, Env, AB_drag, theta, 1-thrust_err, 1+ab_err),...
               tspan, x0, Option);

    function [value, isterminal, direction] = AltEvent(railTime,X1)
        %   Stop simulation at apogee
        value = (X1(1)< EventVal)-0.5;   % altitude reaches beginning of burn phase
        isterminal = 1; % Stop the integration
        direction = EventDir;      
    end

    function [value, isterminal, direction] = SpeedEvent(railTime,X1)
        %   Stop simulation at apogee
        value = X1(2)-EventVal;   % altitude reaches beginning of burn phase
        isterminal = 1; % Stop the integration
        direction = EventDir;      
    end

end