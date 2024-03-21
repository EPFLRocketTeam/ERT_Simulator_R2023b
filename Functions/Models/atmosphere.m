%% Standard atmosphere - Limit 100 km
% This script compute the atmospheric coefficient using the International 
% Standard Atmosphere model (ISA). The value of temperature are obtained
% using a linear interpolation between the different layer of atmosphere.
% Each layer has it's own slope and initial temperature. Those values can
% be found at [TODO]. 
% We made the assumption that the atmosphere is hydrostatic.

% in    alt : The altitude at which the coefficients will be evaluate
% in    env : The environement of the simulation

% out   T : The temperature at this altitude
% out   a : speed of sound
% out   p : pressure
% out   rho : density
% out   nu : viscosity

% Warning : Check that the non differenciability of the temperature doesn't
% affect the quality of the simulation.

% > Viscosité à revoir (très haute atm)
% > Humidité
% > Comparaison
%  - Avec l'ancien modèle
%  - Avec les autres simulateurs

function [T, a, p, rho, nu] = atmosphere(alt, env)
    % Check that altitude isn't greater than 100 km
    if alt > 1e5
        alt = 1e5;
    end

    % Constant
    r_earth = 6378137;              % [m] Radius of earth
    R_star = 287.04;                % [J / (kg K)] real gas constant of air (R0 / M_air)
    gamma = 1.4;                    % [-] specific heat coefficient of air

    % Initial
    p0 = 101325;                    % [Pa] Pressure at sea level
    T0 = atmosphere_temperature(0, env); % [K] temperature at sea level
    g0 = 9.80665;                   % [m/sec^2] gravity at sea level

    % Evaluate the gravity at the given altitude
    g = g0*(r_earth/(r_earth+alt))^2;
    
    % Evaluate temperature using ISA and the Temperature Lapse Rate [K/m]
    T = atmosphere_temperature(alt, env);

    % Evaluate speed of sound
    % Source : https://en.wikipedia.org/wiki/Speed_of_sound
    a = sqrt(gamma * R_star * T);

    % Pressure
    p = p0 * exp(- g * alt / (T0 * R_star));

    % Density (ideal gas law)
    % Source : https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6764509/
    x = env.Saturation_Vapor_Ratio*env.Humidity_Ground;
    rho = p / (T * R_star) * (1 + x) / (1 + 1.609 * x);
    
    % Viscosity
    % Source https://www.grc.nasa.gov/www/k-12/airplane/viscosity.html
    mu = 1.458 * 1e-6 * T^1.5 / (T + 110.4);    % Dynamic viscosity
    nu = mu / rho;                              % Kinematic viscosity

    %[T, a, p, rho, nu] = old_atm(alt, env);
end



% This function return the temperature at a specific altitude using
% interpolation between layers of atmosphere. Those layers are given by the
% international standard model.
% in    alt : Altitude at which the temperature is evaluated
% out   T : The temperature at the given altitude

% INFO : If more pressision is needed, you can add atmospheric layer by
% adding altitude and temperature of the new layer in table_isa_alt and
% table_isa_tem respectively. Note that the altitude table should remain in
% ascending order.
function T = atmosphere_temperature(alt, env)
    % Table of the International Atmospheric Model
    % Source : https://en.wikipedia.org/wiki/International_Standard_Atmosphere
    table_isa_alt = [0,      11000,  20000,  32000,  47000,  51000,  71000,  86000];
    table_isa_tem = [env.Temperature_Ground, 216.65, ...
        216.65, 228.65, 270.65, 270.65, 214.15, 186.95];

    % Check if the altitude is in range
    if alt < table_isa_alt(1) || alt >= table_isa_alt(end)
        T = table_isa_tem(end);
    else
        % Find the interval index
        for i = 1:length(table_isa_alt)-1
            if alt >= table_isa_alt(i) && alt < table_isa_alt(i+1)
                index = i;
                break;
            end
        end

        % Interpolate the temperature
        alt1 =  table_isa_alt(index);
        alt2 =  table_isa_alt(index+1);
        T1 =    table_isa_tem(index);
        T2 =    table_isa_tem(index+1);
        
        T = T1 + (alt - alt1) * (T2 - T1)/(alt2 - alt1);
    end
end