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