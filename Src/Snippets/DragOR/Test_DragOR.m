close all; clear all; clc;
addpath(genpath('./Declarations'),...
        genpath('./Functions'),...
        genpath('./Snippets'),...
        genpath('./Simulator_3D'));
% Rocket Definition
Rocket = rocketReader('Nordend_N1332.txt');

Environment = environnementReader('Environment/Environnement_Definition_EuRoC.txt');

Drag = 'Drag/dragOK_N1332.csv';

interp_type = 'time';

simulationOutputs = SimOutputReader('Simulation/Simulation_outputs.txt');

simulatior3D = Simulator3D(Rocket, Environment, Drag, interp_type, simulationOutputs);

%%

dragOR = readtable(Drag,'Format','%s%s%s%s');

Index_start = find(contains(dragOR{:,1},'# Event LIFTOFF'));
Index_end = find(contains(dragOR{:,1},'# Event APOGEE'));

dragOR = dragOR{Index_start:Index_end,:};

removeIndex = find(contains(dragOR(:,1),"Event"));
dragOR(removeIndex,:) = [];
dragOR = str2double(dragOR);


%% 

t = 28;
x = 3000;
v = 20;

display(simulatior3D.Drag)

CD = drag(simulatior3D.Drag, simulatior3D.interp_type, t, x, v);