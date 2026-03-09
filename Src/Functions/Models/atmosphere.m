%% Standard atmosphere - Limit 100 km
% This script compute the atmospheric coefficient using the International 
% Standard Atmosphere model (ISA). The value of temperature are obtained
% using a linear interpolation between the different layer of atmosphere.
% Each layer has it's own slope and initial temperature. 
% All the calculation and the physical formulas can be found on the wiki of
% the ERT :
% LINK : https://rocket-team.epfl.ch/en/competition/firehorn/flight-dynamics/ertsim/ERT-Sim-2025a/Atmosphere_Model

% in    alt : The altitude at which the coefficients will be evaluate
% in    env : The environement of the simulation

% out   T : The temperature at this altitude
% out   a : speed of sound
% out   p : pressure
% out   rho : density
% out   nu : viscosity

function [temperature, speedOfSound, pressure, density, kinematicViscosity] = atmosphere(alt, env)
    % Constant
    specificGasConstant = 287.04;   % [J / (kg K)] Real gas constant of air (R0 / M_air)
    specificHeatCoeff = 1.4;        % [-] Specific heat coefficient of air

    % Initial
    seaPressure = 101325;                       % [Pa] Pressure at sea level
    seaTemperature = env.Temperature_Ground;    % [K] Temperature at sea level
    gravity = 9.80665;                          % [m/sec^2] Gravity at sea level
    
    % Evaluate temperature using ISA and the Temperature Lapse Rate [K/m]
    % Also evaluate the integral dh/T(h) for the pressure.
    [temperature, I] = atmosphereTemperatureIntegral(alt, env);

    % Evaluate speed of sound
    speedOfSound = sqrt(specificHeatCoeff * specificGasConstant * temperature);

    % Pressure
    pressure = seaPressure * exp(-gravity / specificGasConstant * I);
    % Pressure 0 over the max value of the ISA
    if alt > 86000
        pressure = 0;
    end

    % Density (ideal gas law)
    x = env.Saturation_Vapor_Ratio*env.Humidity_Ground;
    density = pressure / (temperature * specificGasConstant) * (1 + x) / (1 + 1.609 * x);
    
    % Viscosity
    mu = 1.715e-5 * (temperature/seaTemperature)^1.5 * (seaTemperature + 110.4) / (temperature + 110.4);    % Dynamic viscosity
    kinematicViscosity = mu / density;                                              % Kinematic viscosity
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
% adding altitude and temperature of the new layer in tableIsaAltitude and
% tableIsaTemperature respectively. Note that the altitude table should remain in
% ascending order.
function [temperature, I] = atmosphereTemperatureIntegral(alt, env)
    % Table of the International Atmospheric Model
    % Source : https://en.wikipedia.org/wiki/International_Standard_Atmosphere
    tableIsaAltitude = [0,      11000,  20000,  32000,  47000,  51000,  71000,  86000];
    tableIsaTemperature = [floor(env.Temperature_Ground), 216.65, ...
        216.65, 228.65, 270.65, 270.65, 214.15, 186.95];

    % Initialize the integral
    I = 0;

    % Check if the altitude is in range
    if alt < tableIsaAltitude(1) || alt >= tableIsaAltitude(end)
        temperature = tableIsaTemperature(end);
    else
        % Find the interval index and compute the integral at each layer
        for i = 1:length(tableIsaAltitude)-1
            if alt >= tableIsaAltitude(i) && alt < tableIsaAltitude(i+1)
                index = i;
                break;
            else
                % Integrate over all the layer
                I = I + integral_dh_T(...
                    tableIsaAltitude(i), tableIsaAltitude(i+1), ...
                    tableIsaTemperature(i), tableIsaTemperature(i+1));
            end
        end
        %disp(["Altitude : " num2str(alt)])
        % Interpolate the temperature
        alt1 =  tableIsaAltitude(index);
        alt2 =  tableIsaAltitude(index+1);
        T1 =    tableIsaTemperature(index);
        T2 =    tableIsaTemperature(index+1);
        
        temperature = T1 + (alt - alt1) * (T2 - T1)/(alt2 - alt1);

        % Integrate until the altitude of the rocket (add the last layer)
        I = I + integral_dh_T(alt1, alt, T1, temperature);
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