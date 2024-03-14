# ERT Flight Simulator :)
MATLAB flight simulator of the EPFL Rocket Team. 

## Repository Structure
This repository contains several sub-folders implementing various aspects of the simulation. Here we present a basic overview of the main components.

### Hierarchy
Here is a simplified hierarchy of the main subfolders.
```
ERT_simulator 
├── Declarations 
|   ├── Environment
|   ├── Motor
|   ├── Rocket
│   └── Simulation
|
├── Functions
|   ├── Maps
|   ├── Math
|   ├── Models
|   └── Utilities
|
├── Simulator_1/2/3D
|
├── Litterature
|
└── README.md
```

### Subfolders
Here is a brief description of these subfolders.

1. Declarations: input files for the interacting systems of the simulation (the **Rocket**, the **Motor** and the **Environment**.)
2. Functions: **Mathematical** functions and **Models** used in the simulation, as well as **Utilities** to manipulate the data and draw the rocket. Also contains the relevant code for 3D **Maps**
3. Simulator_1/2/3D: **Mains** used to run the code for 1/2/3D simulation.
4. Litterature: some relevant litterature for reference.

## User guide
We present here a brief user guide for the utilization of the ERT-Sim.

### Litterature
First and foremost, it is recommended to read the [project report](Litterature/Copie%20de%20Rapport%20Eric%20Simulation.pdf) for this Simulator, written by Eric Brunner & Maximilien Mingard, as well as the ERT [wiki](https://rocket-team.epfl.ch/). The report explains all the theory behind the flight simulation of a rocket and takes the reader through the steps in an understandable way, starting with the easier 1D model and ramping up gently to the full 3D simulation.

### Standard procedure
In order to run a 3D simulation, follow these steps:
1. Make sure your input files are in the corresponding subfolders in `Declarations` and follow the format of the already present files. Make sure your **Motor** file is properly referenced in your **Rocket** file.
2. Open the desired Main program, for instance `Simulator_3D\Main_3D.m` and update the **Rocket** and **Environment** input files in the corresponding lines.
3. Execute the program. The simulation will start to run and output the first figures. A prompt will appear in the command box asking you whether to plot the remaining figures. Type "yes" or "no" according to your preference.

Several Mains have been implemented over the years by successive people to test different functionalities. 