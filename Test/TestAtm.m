% Initialize
%close all; clear all; clc;
addpath(genpath('./../Declarations'),...
        genpath('./../Functions'),...
        genpath('./../Snippets'),...
        genpath('./../Archives/Models'),...
        genpath('./../Simulator_3D'));

set(groot, 'defaultAxesFontSize', 22);
set(groot, 'defaultLegendInterpreter', 'latex')
set(groot, 'defaultTextInterpreter', 'latex')
set(groot, 'defaultAxesTickLabelInterpreter', 'latex')

% Objective compare the atmospheric model

% Load environement 
env = environnementReader('Environment/Environnement_Definition_EuRoC.txt');

% Altitude and Results array
len = 1e5;
alt = linspace(10, 1e4, len);

T = (1:len);
a = (1:len);
p = (1:len);
rho = (1:len);
nu = (1:len);

len_ = 1e5;
alt_ = linspace(10, 1e5, len);

T_ = (1:len_);
a_ = (1:len_);
p_ = (1:len_);
rho_ = (1:len_);
nu_ = (1:len_);

% Compute values with stdAtmos
for k = 1:length(alt)
    [T(k), a(k), p(k), rho(k), nu(k)] = stdAtmos(alt(k), env);
end

for k = 1:length(alt_)
    [T_(k), a_(k), p_(k), rho_(k), nu_(k)] = atmosphere(alt_(k), env);
    if imag(p_) ~= 0
        disp(["Index ", num2str(k), ", p = ", num2str(p_)])
    end
end

% Show results
Plot(T, a, p, rho, nu, alt, 1)
Plot(T_, a_, p_, rho_, nu_, alt_, 1)


% Compare with value in literature
% Source : https://www.engineeringtoolbox.com/international-standard-atmosphere-d_985.html
alt_compare = linspace(1000, 30000, 11);
[~, ~, ~, rho0, ~] = atmosphere(0, env);
for k = 1:length(alt_compare)
    [T, a, p, rho, nu] = atmosphere(alt_compare(k), env);
    disp(["z:" alt_compare(k) "T:" num2str(T) ", p:" num2str(p) ", rho:" num2str(rho/rho0), ", c:" num2str(a)])
end

function Plot(T, a, p, rho, nu, alt, fig_nbr)
    alt = alt/1e3;

    figure(fig_nbr)
    subplot(2,3,1)
    hold on
    title("Temperature")
    plot(T, alt, LineWidth=1.5)
    grid on
    box on
    xlabel("$T$ [K]")
    ylabel("$h$ [km]")

    subplot(2,3,2)
    hold on
    title("Speed of sound")
    plot(a, alt, LineWidth=1.5)
    grid on
    box on
    xlabel("$a$ [m/s]")
    ylabel("$h$ [km]")

    subplot(2,3,3)
    hold on
    title("Pressure")
    plot(p, alt, LineWidth=1.5)
    grid on
    box on
    xlabel("$p$ [N/m$^2$]")
    ylabel("$h$ [km]")

    subplot(2,3,4)
    hold on
    title("Density")
    plot(rho, alt, LineWidth=1.5)
    grid on
    box on
    xlabel("$\rho$ [kg/m$^3$]")
    ylabel("$h$ [km]")

    subplot(2,3,5)
    hold on
    title("Viscosity")
    plot(nu, alt, LineWidth=1.5)
    grid on
    box on
    set(gca, 'XScale', 'log')
    xlabel("$\nu$ [m$^2$/s]")
    ylabel("$h$ [km]")

end