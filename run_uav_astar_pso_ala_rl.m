function run_uav_astar_pso_ala_rl()
% UAV 3D path planning with A* + PSO + ALA + RL adaptive controller
% Single-file demo, no extra toolbox required.

clc; clearvars; close all;
rng(42);

% Ensure local module functions are on path
curDir = fileparts(mfilename('fullpath'));
if ~isempty(curDir) && isfolder(curDir)
    addpath(curDir);
end

cfg = defaultConfig();

% Graphical scene input has highest priority.
if cfg.useGraphicalSceneInput
    [cfg.obstacles, cfg.threatRegions, cfg.start, cfg.goal] = inputSceneFromFigure(cfg);
elseif cfg.useInteractiveObstacleInput
    % Manual obstacle input in command window.
    cfg.obstacles = inputObstaclesFromConsole(cfg.mapSize);
end

[cfg.occupancy, cfg.threatMap, cfg.terrainMap, cfg.windField] = buildEnvironmentMaps(cfg);

fprintf('--- Stage 1: PSO + ALA + RL adaptive optimization ---\n');
optResult = optimizeWeightsPSO_ALA_RL(cfg);

fprintf('\nBest scalar fitness: %.6f\n', optResult.globalBestFitness);
disp('Best weight vector [w_h, w_move, w_threat, w_terrain, safety_margin, w_smooth, w_multi, w_climb, w_speed, w_wind]:');
disp(optResult.globalBestPos);

fprintf('\n--- Stage 2: Build Pareto-optimal path set ---\n');
pareto = buildParetoSet(cfg, optResult.archive, optResult.globalBestPos);

if isempty(pareto)
    fprintf('No feasible path found under current map/constraint settings.\n');
    return;
end

fprintf('Pareto path count: %d\n', numel(pareto));
printParetoSummary(pareto);

plotSceneAndParetoPaths(cfg, pareto);
plotCostFigures(pareto);
plotDetailedReport(cfg, pareto);

end
