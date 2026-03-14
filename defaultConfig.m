function cfg = defaultConfig()
% Map and UAV setup
cfg.mapSize = [45, 45, 20];
cfg.start = [3, 3, 3];
cfg.goal = [40, 39, 12];
cfg.flyRange = [2, 18]; % [zMin, zMax]

cfg.paramLower = [0.8, 0.4, 0.0, 0.0, 1.0, 0.0, 0.2, 0.1, 0.0, 0.0];
cfg.paramUpper = [3.5, 4.5, 6.0, 4.0, 6.0, 2.0, 3.0, 3.5, 2.0, 3.0];

cfg.seedWeights = [1.4, 1.0, 1.8, 0.8, 2.8, 0.6, 1.0, 1.2, 0.4, 1.2];

cfg.obstacles = [
    10, 12, 14;
    12, 13, 13;
    14, 18, 15;
    20, 25, 17;
    23, 25, 16;
    25, 30, 14;
    28, 14, 12;
    30, 18, 18;
    32, 32, 16;
    35, 24, 15;
    37, 36, 18
];

cfg.threatRegions = [
    16, 16, 8, 6, 1.2;
    30, 10, 9, 7, 1.0;
    26, 31, 10, 8, 1.4
];

cfg.terrainAmp = 1.5;
cfg.terrainFreq = [0.18, 0.13];

cfg.baseWind = [0.8, -0.2, 0.1];
cfg.windSwirl = 0.9;

cfg.useInteractiveObstacleInput = false;
cfg.useGraphicalSceneInput = true;

cfg.maxExpandedNodes = 250000;
cfg.neighborMode = '26';
cfg.goalTolerance = 0;

cfg.popSize = 24;
cfg.maxIter = 40;
cfg.archiveMax = 80;

cfg.rl.alpha = 0.20;
cfg.rl.gamma = 0.92;
cfg.rl.epsilon = 0.22;
cfg.rl.epsilonDecay = 0.97;
cfg.rl.epsilonMin = 0.05;

cfg.ref.length = 120;
cfg.ref.risk = 120;
cfg.ref.energy = 200;
cfg.ref.smooth = 100;
cfg.ref.time = 160;

cfg.plotMaxPaths = 10;
end