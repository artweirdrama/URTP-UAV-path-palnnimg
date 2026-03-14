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