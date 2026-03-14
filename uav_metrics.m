function archive = updateArchive(archive, x, metrics, fit, path, maxSize)
if isempty(metrics)
    return;
end

candVec = metrics.vector;
toRemove = false(1, numel(archive));
isDominated = false;

for i = 1:numel(archive)
    a = archive(i).metrics.vector;
    if dominates(a, candVec)
        isDominated = true;
        break;
    end
    if dominates(candVec, a)
        toRemove(i) = true;
    end
end

if isDominated
    return;
end

archive = archive(~toRemove);
entry.x = x;
entry.metrics = metrics;
entry.fitness = fit;
entry.path = path;
archive = [archive, entry]; %#ok<AGROW>

if numel(archive) > maxSize
    V = zeros(numel(archive), numel(candVec));
    for i = 1:numel(archive)
        V(i, :) = archive(i).metrics.vector;
    end
    keep = crowdingPrune(V, maxSize);
    archive = archive(keep);
end
end

function yes = dominates(a, b)
yes = all(a <= b) && any(a < b);
end

function keep = crowdingPrune(V, K)
N = size(V, 1);
if N <= K
    keep = true(N, 1);
    return;
end

M = size(V, 2);
score = zeros(N, 1);
for m = 1:M
    [s, idx] = sort(V(:, m));
    score(idx(1)) = inf;
    score(idx(end)) = inf;
    span = max(s(end) - s(1), 1e-9);
    for i = 2:N-1
        score(idx(i)) = score(idx(i)) + (s(i+1) - s(i-1)) / span;
    end
end

[~, order] = sort(score, 'descend');
keep = false(N, 1);
keep(order(1:K)) = true;
end

function pareto = buildParetoSet(cfg, archive, bestX)
pareto = struct('weights', {}, 'path', {}, 'metrics', {});

if isempty(archive)
    [p, ok, m] = astar3dWeighted(cfg, bestX);
    if ok
        pareto(1).weights = bestX;
        pareto(1).path = p;
        pareto(1).metrics = m;
    end
    return;
end

for i = 1:numel(archive)
    pareto(i).weights = archive(i).x;
    pareto(i).path = archive(i).path;
    pareto(i).metrics = archive(i).metrics;
end

nExtra = min(20, max(8, floor(numel(archive) / 2)));
lb = cfg.paramLower; ub = cfg.paramUpper;
for k = 1:nExtra
    x = bestX + 0.18 * (ub - lb) .* randn(1, numel(bestX));
    x = clipVec(x, lb, ub);
    [p, ok, m] = astar3dWeighted(cfg, x);
    if ok
        entry.weights = x;
        entry.path = p;
        entry.metrics = m;
        pareto = [pareto, entry]; %#ok<AGROW>
    end
end

if isempty(pareto)
    return;
end
V = zeros(numel(pareto), numel(pareto(1).metrics.vector));
for i = 1:numel(pareto)
    V(i, :) = pareto(i).metrics.vector;
end

isND = true(numel(pareto), 1);
for i = 1:numel(pareto)
    for j = 1:numel(pareto)
        if i ~= j && dominates(V(j, :), V(i, :))
            isND(i) = false;
            break;
        end
    end
end
pareto = pareto(isND);

if numel(pareto) > cfg.plotMaxPaths
    keep = crowdingPrune(V(isND, :), cfg.plotMaxPaths);
    pareto = pareto(keep);
end

score = zeros(numel(pareto), 1);
for i = 1:numel(pareto)
    m = pareto(i).metrics;
    score(i) = 0.3*m.length + 0.25*(m.threat + m.safety) + 0.25*m.energy + 0.1*m.smooth + 0.1*m.time;
end
[~, idx] = sort(score, 'ascend');
pareto = pareto(idx);
end

function s = computeRouteStats(path, metrics)
s.nodes = size(path, 1);
s.directDist = norm(double(path(end, :) - path(1, :)));
s.detourRatio = metrics.length / max(s.directDist, 1e-6);
s.avgAlt = mean(path(:, 3));

dz = diff(path(:, 3));
s.climbDist = sum(max(dz, 0));
s.descDist = sum(max(-dz, 0));

s.maxTurnDeg = 0;
s.turnOver45 = 0;
if size(path, 1) >= 3
    for i = 3:size(path, 1)
        a = path(i - 1, :) - path(i - 2, :);
        b = path(i, :) - path(i - 1, :);
        if norm(a) < 1e-9 || norm(b) < 1e-9
            continue;
        end
        c = dot(a, b) / (norm(a) * norm(b));
        c = min(max(c, -1), 1);
        ang = acosd(c);
        s.maxTurnDeg = max(s.maxTurnDeg, ang);
        if ang > 45
            s.turnOver45 = s.turnOver45 + 1;
        end
    end
end
end

function comp = computePathMetrics(path, cfg, w, distObs)
threat = cfg.threatMap;
terrain = cfg.terrainMap;
wind = cfg.windField;

n = size(path, 1);
if n < 2
    comp.length = 0;
    comp.threat = 0;
    comp.energy = 0;
    comp.smooth = 0;
    comp.time = 0;
    comp.climb = 0;
    comp.wind = 0;
    comp.terrain = 0;
    comp.safety = 0;
    comp.total = 0;
    comp.vector = zeros(1, 5);
    return;
end

L = 0; threatCost = 0; terrainCost = 0; smoothCost = 0;
climbCost = 0; speedCost = 0; windCost = 0; safetyCost = 0;
energy = 0; timeCost = 0;

for i = 2:n
    p0 = path(i - 1, :);
    p1 = path(i, :);
    step = p1 - p0;
    ds = norm(step);
    L = L + ds;

    climbCost = climbCost + w(8) * max(step(3), 0) + 0.8 * w(8) * max(-step(3), 0);

    tx = p1(1); ty = p1(2); tz = p1(3);
    threatCost = threatCost + w(3) * threat(tx, ty, tz);
    terrainCost = terrainCost + w(4) * terrain(tx, ty, tz);

    dObs = distObs(tx, ty, tz);
    safetyCost = safetyCost + (w(3) * 0.2) * exp(-dObs / max(w(5), 1e-6));

    dirVec = step / max(ds, 1e-6);
    windVec = squeeze(wind(tx, ty, tz, :))';
    headwind = max(0, dot(windVec, dirVec));
    windCost = windCost + w(10) * headwind;

    spd = 1.0 + 0.55 * ds;
    speedCost = speedCost + w(9) * spd^2 * (1 + 0.4 * threat(tx, ty, tz));

    if i >= 3
        s0 = path(i - 1, :) - path(i - 2, :);
        if norm(s0) > 0 && ds > 0
            c = dot(s0, step) / (norm(s0) * ds);
            c = min(max(c, -1), 1);
            ang = acos(c);
            smoothCost = smoothCost + w(6) * (ang / pi);
        end
    end
end

moveCost = w(2) * L;
energy = moveCost + climbCost + terrainCost + speedCost + windCost;
timeCost = L + 0.35 * climbCost;

total = moveCost + threatCost + terrainCost + safetyCost + smoothCost + climbCost + speedCost + windCost;

comp.length = L;
comp.threat = threatCost;
comp.energy = energy;
comp.smooth = smoothCost;
comp.time = timeCost;
comp.climb = climbCost;
comp.wind = windCost;
comp.speed = speedCost;
comp.move = moveCost;
comp.terrain = terrainCost;
comp.safety = safetyCost;
comp.total = total;
comp.vector = [L, threatCost + safetyCost, energy, smoothCost, timeCost];
end