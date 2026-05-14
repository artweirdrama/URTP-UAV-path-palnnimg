Source Code for AHS-GGWO Algorithm
========================================================================

1. OVERVIEW
------------------------------------------------------------------------
This package contains the MATLAB source code for the paper titled:
"HCRS: Hierarchical Cooperative Recombination Strategy for Many-Objective Multi-UAV Path Planning"

It provides a minimal runnable demo to validate the algorithm's performance and reproducibility, specifically demonstrating the path planning task in the complex 3D environments described in the manuscript.

2. SYSTEM REQUIREMENTS & DEPENDENCIES
------------------------------------------------------------------------
- MATLAB Version: MATLAB R2024b is strictly recommended to ensure full compatibility.
- Toolboxes Required: 
  * No special third-party toolboxes are required.
  * The standard MATLAB 'Image Processing Toolbox' is recommended for loading the digital elevation model (DEM) terrain file (`ChrismasTerrain2.tif`).
- Hardware: Standard PC (Intel i5/i7 or equivalent recommended for optimal speed).

3. QUICK START & EXAMPLE COMMANDS
------------------------------------------------------------------------
Step 1: Unzip this package to a local directory.
Step 2: Open MATLAB and set the "Current Folder" to this directory.
Step 3: Run the main script by typing the following command in the MATLAB Command Window (or open `AMAIN.m` and press F5):

>> AMAIN

** Expected Outputs: **
1. Console Log: The command window will print the initialization progress and the real-time iteration outcome, including the computation time.
2. Visualizations: Upon completion (default 200 iterations), a 3D figure will automatically pop up, displaying the converged, collision-free trajectories of the UAV swarm in the selected terrain scenario.


4. CORE FUNCTIONAL ANNOTATIONS 
------------------------------------------------------------------------
** [1] Adaptive Operator Switching Logic (in `AMAIN.m`) **
- The algorithm dynamically monitors the population's stagnation status based on a predefined improvement threshold. 
- If no significant improvement is observed for K consecutive iterations, the algorithm automatically toggles the `useGWO` flag. This switches the search mode between the Grey Wolf Optimizer (GWO) operators (for local exploitation and convergence) and the Genetic Algorithm (GA) operators (for global exploration to escape local optima).

** [2] Evolutionary External Archive Update Rules (in `AMAIN.m` / `UpdateRepWithGA`) **
- The script manages a dynamic external archive (`rep`) to store global elite non-dominated solutions.
- Update Rules: It merges newly generated non-dominated solutions with the existing archive and truncates it based on crowding distance to maintain Pareto diversity.
- Evolutionary Mechanism: It actively applies Simulated Binary Crossover (SBX) and polynomial mutation exclusively to the archive members.
- Feedback Guidance: The top-ranked collision-free solutions from the archive are selected as the alpha, beta, and delta leaders to deeply guide the underlying GWO's search direction.

5. FILE DESCRIPTION
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

6. NOTES ON REPRODUCIBILITY
------------------------------------------------------------------------
- The parameters in this demo are consistent with those listed in Table 2 and Table 3 of the revised manuscript.
- To switch scenarios, you can modify the line `model = CreateModel3();` in `AMAIN.m` to `CreateModel1()` or `CreateModel2()`.
========================================================================