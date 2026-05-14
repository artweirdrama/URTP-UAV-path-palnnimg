Source Code for AHS-GGWO Algorithm
========================================================================

1. OVERVIEW
------------------------------------------------------------------------
This package contains the MATLAB source code for the paper titled:
"HCRS: Hierarchical Cooperative Recombination Strategy for Many-Objective Multi-UAV Path Planning"

It provides a minimal runnable demo to validate the algorithm's performance and reproducibility, specifically demonstrating the path planning task in the complex 3D environments described in the manuscript.

2. SYSTEM REQUIREMENTS
------------------------------------------------------------------------
- Software: MATLAB R2024b.
- Toolboxes: No special toolboxes are strictly required, but the 'Image Processing Toolbox' is recommended for loading the terrain file (.tif).
- Hardware: Standard PC (Intel i5/i7 recommended for optimal speed).

3. FILE DESCRIPTION
------------------------------------------------------------------------
** Main Execution **
- AMAIN.m                : The main entry script. RUN THIS file to start the simulation.
- AHS_GGWO.m             : The core implementation of the proposed AHS-GGWO algorithm.

** Environment & Modeling **
- CreateModel1~3.m       : Scripts to generate the maps, obstacles, and threats for Scenarios 1, 2, and 3.
- ChrismasTerrain2.tif   : The digital elevation model (DEM) file used for the terrain.
- PlotModel.m            : Helper function to visualize the 3D environment.

** Cost Function **
- SingleDroneCost.m      : The objective function evaluating path length, safety (threats), altitude, and smoothness.
- AMyCost.m              : Wrapper function for handling multi-UAV cost aggregation.

** Utilities (Multi-Objective Optimization) **
- NDSort.m               : Non-dominated Sorting (for Pareto front ranking).
- CrowdingDistance.m     : Calculation of crowding distance to maintain diversity.
- PGDominates.m          : Checks Pareto dominance relationship between solutions.
- PGDetermineDomination.m: Vectorized dominance check.
- PGDistP2S.m            : Calculus function for Point-to-Segment distance (used in threat avoidance).
- PGSphericalToCart.m    : Coordinate conversion utility.
- PGCreateRandomSolution.m: Initialization of random populations.

** Visualization **
- APlotSolution.m        : Plots the final 3D trajectories of the UAV swarm.

4. HOW TO RUN
------------------------------------------------------------------------
Step 1: Unzip this package to a local directory.
Step 2: Open MATLAB and set the "Current Folder" to this directory.
Step 3: Open the file "AMAIN.m".
Step 4: Click the "Run" button (or press F5).

RESULTS:
- The script will execute the optimization process (default 200 iterations).
- The computation time will be displayed in the Command Window.
- A 3D figure will pop up showing the converged trajectories of the UAVs in the selected scenario.

5. NOTES ON REPRODUCIBILITY
------------------------------------------------------------------------
- The parameters in this demo are consistent with those listed in Table 2 and Table 3 of the revised manuscript.
- To switch scenarios, you can modify the line "model = CreateModel3();" in 'AMAIN.m' to 'CreateModel1()' or 'CreateModel2()'.

========================================================================
