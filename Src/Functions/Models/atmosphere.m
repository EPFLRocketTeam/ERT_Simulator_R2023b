%% Standard atmosphere - Limit 100 km
% This script compute the atmospheric coefficient using the International 
% Standard Atmosphere model (ISA). The value of temperature are obtained
% using a linear interpolation between the different layer of atmosphere.
% Each layer has it's own slope and initial temperature. 
% All the calculation and the physical formulas can be found on the wiki of
% the ERT :
% LINK : https://rocket-team.epfl.ch/en/competition/firehorn/flight-dynamics/ertsim/Atmosphere_Model

% in    alt : The altitude at which the coefficients will be evaluate
% in    env : The environement of the simulation

% out   T : The temperature at this altitude
% out   a : speed of sound
% out   p : pressure
% out   density : density
% out   nu : viscosity

function [T, a, p, density, nu] = atmosphere(alt, env)
    % Constant
    R_star = 287.04;                % [J / (kg K)] Real gas constant of air (R0 / M_air)
    gamma = 1.4;                    % [-] Specific heat coefficient of air

    % Initial
    p0 = 101325;                    % [Pa] Pressure at sea level
    T0 = env.groundTemperature;    % [K] Temperature at sea level
    g = 9.80665;                    % [m/sec^2] Gravity at sea level
    
    % Evaluate temperature using ISA and the Temperature Lapse Rate [K/m]
    % Also evaluate the integral dh/T(h) for the pressure.
    [T, I] = atmosphere_temperature_integral(alt, env);

    % Evaluate speed of sound
    a = sqrt(gamma * R_star * T);

    % Pressure
    p = p0 * exp(-g / R_star * I);
    % Pressure 0 over the max value of the ISA
    if alt > 86000
        p = 0;
    end

    % Density (ideal gas law)
    x = env.Saturation_Vapor_Ratio*env.groundHumidity;
    density = p / (T * R_star) * (1 + x) / (1 + 1.609 * x);
    
    % Viscosity
    mu = 1.715e-5 * (T/T0)^1.5 * (T0 + 110.4) / (T + 110.4);    % Dynamic viscosity
    nu = mu / density;                                              % Kinematic viscosity
end



% This function return the temperature at a specific altitude using
% interpolation between layers of atmosphere. Those layers are given by the
% international standard model.
% It also compute the integral for the pressure, as the temperature vary
% with altitude.
% in    alt : Altitude at which the temperature is evaluated
% in    env : The environement of the simulation
% out   T : The temperature at the given altitude

% INFO : If more pressision is needed, you can add atmospheric layer by
% adding altitude and temperature of the new layer in table_isa_alt and
% table_isa_tem respectively. Note that the altitude table should remain in
% ascending order.
function [T, I] = atmosphere_temperature_integral(alt, env)
    % Table of the International Atmospheric Model
    % Source : https://en.wikipedia.org/wiki/International_Standard_Atmosphere
    table_isa_alt = [0,      11000,  20000,  32000,  47000,  51000,  71000,  86000];
    table_isa_tem = [floor(env.groundTemperature), 216.65, ...
        216.65, 228.65, 270.65, 270.65, 214.15, 186.95];

    % Initialize the integral
    I = 0;

    % Check if the altitude is in range
    if alt < table_isa_alt(1) || alt >= table_isa_alt(end)
        T = table_isa_tem(end);
    else
        % Find the interval index and compute the integral at each layer
        for i = 1:length(table_isa_alt)-1
            if alt >= table_isa_alt(i) && alt < table_isa_alt(i+1)
                index = i;
                break;
            else
                % Integrate over all the layer
                I = I + integral_dh_T(...
                    table_isa_alt(i), table_isa_alt(i+1), ...
                    table_isa_tem(i), table_isa_tem(i+1));
            end
        end
        %disp(["Altitude : " num2str(alt)])
        % Interpolate the temperature
        alt1 =  table_isa_alt(index);
        alt2 =  table_isa_alt(index+1);
        railTime =    table_isa_tem(index);
        flightTime =    table_isa_tem(index+1);
        
        T = railTime + (alt - alt1) * (flightTime - railTime)/(alt2 - alt1);

        % Integrate until the altitude of the rocket (add the last layer)
        I = I + integral_dh_T(alt1, alt, railTime, T);
    end
end

% Function that compute the integral dh/T(h) (see the wiki page for more 
% information).
% in    hi : Initial height
% in    hf : Final height
% in    Ti : Initial temperature
% in    Tf : Final temperature
% out   Ik : The integral
function Ik = integral_dh_T(hi, hf, Ti, Tf)
    % Compute the integral
    if Ti == Tf
        % Case temperature slope = 0 : T = const
        Ik = (hf - hi) / Ti;
    else
        % Case temperature slope != 0 : T = ah + b
        a = (Tf - Ti) / (hf - hi);
        b = Ti - (a * hi);
        Ik = log( (a*hf+b) / (a*hi+b) )/a;
    end
end