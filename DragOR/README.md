# Introduction
This alternative version of the ERTSim was developed in order to use the 
drag coefficients computed on the rocket simulator OpenRocket for a given 
rocket, motor and environment. The drag coefficients are interpolated with 
a polynomial of degree six either from the time, the altitude or the the 
speed of the rocket.

## Implementation
In order to run this version, the following folder and files have to be 
created/modified (all the mentioned files/folders can be found in the 
"DragOR" folder): 

   1. Export data from OpenRocket
After running a simulation on OpenRocket for a given rocket, motor and 
environment, the data must be exported on a .csv table containing the time, 
altitude total speed and drag coefficient

   2. Drag folder
The drag folder must be added to the folder "Declarations". This folder 
will contain the .csv file from the corresponding OpenRocket simulation.

   3. Main_DragOR.m
The file "Main.m" has to be replaced by the the file "Main\_DragOR.m".

   4. Simulator3D_DragOR.m
The file "Simulator3D.m" has to be replaced by the the file 
"Simulator3D\DragOR.m".

   5. drag_OR.m
The file "drag\OR.m" has to be placed in the folder "Functions\Models".

### Organization
After all the modification, the repository should look something like this:
```
ERT_simulator_R2023b
    README.md
    Main.m
    **Main_dragOR.m**
    stability_analysis_R02.m
    .gitignore
├── Test
|
├── Snippets
|
├── Declarations 
│   └── **Drag**
|
├── Functions
|   ├── Maps
|   ├── Math
|   ├── Models
|       └──**drag_OR.m**
|   └── Utilities
|
├── Simulator_3D
|   └── **Simulator3D_OR.m**
|
├── DragOR
|
└── Archives

```

#### User guide
On "Main_DragOR.m"

Select the file that was previously exported from OpenRocket that 
corresponds to the conditions of the simulation of the ERTSim.
example:
line 14 : Drag = 'Drag/dragOK_N1332.csv';

Select which type of interpolation needs to be used.
example:
exline 16 : interp_type = 'time';
