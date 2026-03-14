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