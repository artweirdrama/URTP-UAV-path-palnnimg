function result = optimizeWeightsPSO_ALA_RL(cfg)
D = 10;
N = cfg.popSize;
T = cfg.maxIter;

lb = cfg.paramLower;
ub = cfg.paramUpper;

X = repmat(lb, N, 1) + rand(N, D) .* repmat((ub - lb), N, 1);
X(1, :) = clipVec(cfg.seedWeights, lb, ub);
V = zeros(N, D);

pBest = X;
pBestFit = inf(N, 1);

archive = struct('x', {}, 'metrics', {}, 'fitness', {}, 'path', {});

gBest = X(1, :);
gBestFit = inf;
gBestMetrics = [];
gBestPath = [];

nStates = 4;
nActions = 3;
Q = zeros(nStates, nActions);
prevGlobal = inf;

base.w  = 0.72;
base.c1 = 1.55;
base.c2 = 1.55;
ala.baseProb = 0.30;
ala.baseJump = 0.24;

historyBest = nan(T, 1);

for t = 1:T
    fits = inf(N, 1);
    metricsCell = cell(N, 1);
    pathCell = cell(N, 1);

    for i = 1:N
        [fits(i), metricsCell{i}, pathCell{i}] = evaluateCandidate(X(i, :), cfg);

        if fits(i) < pBestFit(i)
            pBestFit(i) = fits(i);
            pBest(i, :) = X(i, :);
        end

        if fits(i) < gBestFit
            gBestFit = fits(i);
            gBest = X(i, :);
            gBestMetrics = metricsCell{i};
            gBestPath = pathCell{i};
        end

        if isfinite(fits(i)) && ~isempty(pathCell{i})
            archive = updateArchive(archive, X(i, :), metricsCell{i}, fits(i), pathCell{i}, cfg.archiveMax);
        end
    end

    historyBest(t) = gBestFit;
    improvement = max(0, prevGlobal - gBestFit);
    prevGlobal = gBestFit;

    diversity = mean(std(X, 0, 1) ./ max((ub - lb), 1e-6));
    st = packState(improvement, diversity);

    epsGreedy = max(cfg.rl.epsilonMin, cfg.rl.epsilon * cfg.rl.epsilonDecay^(t - 1));
    if rand < epsGreedy
        action = randi(nActions);
    else
        [~, action] = max(Q(st, :));
    end

    [w, c1, c2, alaProb, alaJump] = actionToParams(action, base, ala);

    reward = 6.0 * improvement + 0.2 * diversity - 0.02 * (~(improvement > 1e-7));

    r1 = rand(N, D);
    r2 = rand(N, D);
    V = w .* V + c1 .* r1 .* (pBest - X) + c2 .* r2 .* (repmat(gBest, N, 1) - X);
    X = X + V;

    X = applyALA(X, gBest, lb, ub, alaProb, alaJump, t, T);
    X = max(min(X, repmat(ub, N, 1)), repmat(lb, N, 1));

    nextDiversity = mean(std(X, 0, 1) ./ max((ub - lb), 1e-6));
    nextImprovement = improvement;
    st2 = packState(nextImprovement, nextDiversity);

    Q(st, action) = Q(st, action) + cfg.rl.alpha * (reward + cfg.rl.gamma * max(Q(st2, :)) - Q(st, action));

    fprintf('Iter %2d/%2d | best=%.6f | imp=%.4g | div=%.4f | action=%d\n', t, T, gBestFit, improvement, diversity, action);
end

result.globalBestPos = gBest;
result.globalBestFitness = gBestFit;
result.globalBestMetrics = gBestMetrics;
result.globalBestPath = gBestPath;
result.archive = archive;
result.Q = Q;
result.historyBest = historyBest;
end

function [fitness, metrics, path] = evaluateCandidate(w, cfg)
[path, ok, comp] = astar3dWeighted(cfg, w);

if ~ok
    fitness = 1e9;
    metrics = [];
    return;
end

metrics = comp;
J_len = metrics.length / cfg.ref.length;
J_risk = metrics.threat / cfg.ref.risk;
J_energy = metrics.energy / cfg.ref.energy;
J_smooth = metrics.smooth / cfg.ref.smooth;
J_time = metrics.time / cfg.ref.time;

mix = min(max(w(7) / cfg.paramUpper(7), 0), 1);
fitness = (1 - 0.5 * mix) * (0.34 * J_len + 0.24 * J_risk + 0.22 * J_energy + 0.10 * J_smooth + 0.10 * J_time) ...
        + 0.5 * mix * max([J_len, J_risk, J_energy, J_smooth, J_time]);

fitness = fitness + 0.005 * norm((w - cfg.seedWeights) ./ (cfg.paramUpper - cfg.paramLower + eps));
end

function state = packState(improvement, diversity)
improved = improvement > 1e-5;
highDiv = diversity > 0.16;
state = 1 + (~improved) + 2 * (~highDiv);
end

function [w, c1, c2, alaProb, alaJump] = actionToParams(action, base, ala)
switch action
    case 1
        w = base.w - 0.12;
        c1 = base.c1 + 0.20;
        c2 = base.c2 + 0.35;
        alaProb = max(0.12, ala.baseProb - 0.10);
        alaJump = max(0.08, ala.baseJump - 0.08);
    case 2
        w = base.w;
        c1 = base.c1;
        c2 = base.c2;
        alaProb = ala.baseProb;
        alaJump = ala.baseJump;
    case 3
        w = base.w + 0.14;
        c1 = base.c1 - 0.25;
        c2 = base.c2 - 0.10;
        alaProb = min(0.55, ala.baseProb + 0.18);
        alaJump = min(0.55, ala.baseJump + 0.20);
    otherwise
        w = base.w; c1 = base.c1; c2 = base.c2;
        alaProb = ala.baseProb; alaJump = ala.baseJump;
end
end

function X = applyALA(X, gBest, lb, ub, alaProb, alaJump, iter, maxIter)
[N, D] = size(X);
phase = 1 - iter / maxIter;
jumpScale = alaJump * (0.35 + 0.65 * phase);
for i = 1:N
    if rand < alaProb
        j = randi(N);
        social = rand(1, D) .* (gBest - X(i, :));
        antiCrowd = randn(1, D) .* (X(i, :) - X(j, :));
        randomWalk = jumpScale * (ub - lb) .* randn(1, D);
        X(i, :) = X(i, :) + 0.55 * social + 0.25 * antiCrowd + randomWalk;
    end
end
X = max(min(X, repmat(ub, N, 1)), repmat(lb, N, 1));
end

function x = clipVec(x, lb, ub)
x = min(max(x, lb), ub);
end