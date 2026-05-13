function run_multi_uav_continuous_planner()
% Continuous-space multi-UAV 3D trajectory planning demo.
% Architecture: inflated-obstacle visibility graph + multi-UAV route sharing
% + collision-checked rounded corners + low-climb altitude profile.
%
% Run in MATLAB:
%   run_multi_uav_continuous_planner

clc; clearvars; close all;
rng(11);

cfg = continuousDefaultConfig();
cfg = chooseContinuousScenario(cfg);
env = buildContinuousEnvironment(cfg);
validateContinuousScene(cfg, env);

fprintf('\n--- Continuous Multi-UAV Planner ---\n');
fprintf('Map %d x %d x %d | UAV=%d | Goal=[%g,%g,%g]\n', ...
    cfg.mapSize(1), cfg.mapSize(2), cfg.mapSize(3), cfg.numUav, cfg.goal(1), cfg.goal(2), cfg.goal(3));
fprintf('Planner: inflated visibility graph + route-sharing cost + rounded-corner smoothing\n\n');

result = planContinuousMultiUAV(cfg, env);
printContinuousSummary(result);
animateContinuousRoutes(cfg, env, result);
plotContinuousCostReport(result);
end

function cfg = continuousDefaultConfig()
cfg.mapSize = [110, 100, 32];
cfg.flyRange = [3, 28];
cfg.numUav = 5;
cfg.maxUav = 8;

cfg.starts = [
      5,  5,  5;
    106,  5,  5;
      5, 96,  6;
    106, 96,  5;
     56,  4,  6
];
cfg.goal = [56, 52, 6];

% [x1 x2 y1 y2 height]
cfg.buildingRects = [
     9, 17,  8, 19, 12;   24, 33,  7, 20, 18;   42, 50,  8, 18, 11;   61, 70,  7, 20, 22;   80, 88,  8, 19, 14;   96,103,  8, 21, 17;
     8, 18, 30, 42, 10;   27, 35, 31, 43, 15;   45, 53, 30, 41, 20;   63, 72, 31, 44, 13;   83, 92, 31, 43, 23;
    10, 19, 55, 68, 16;   29, 38, 56, 70,  9;   46, 54, 55, 68, 19;   65, 74, 55, 68, 15;   84, 93, 56, 70, 21;
     8, 17, 80, 92, 11;   25, 34, 80, 94, 17;   43, 51, 81, 93, 13;   62, 71, 80, 94, 24;   82, 91, 80, 93, 15;   98,105, 79, 91, 10;
    18, 24, 23, 28,  8;   36, 42, 22, 28, 13;   55, 60, 23, 29, 10;   74, 79, 22, 29, 16;   91, 96, 23, 29, 12;
    18, 24, 72, 76,  9;   37, 43, 72, 77, 15;   55, 61, 72, 77, 11;   74, 80, 72, 77, 18;   92, 98, 71, 76, 14
];

cfg.threatRegions = [
    31, 27,  8,  9.0, 0.8;
    69, 27, 10, 10.0, 1.0;
    44, 51,  9,  8.5, 0.7;
    78, 58, 11,  9.0, 0.9;
    35, 82, 10,  8.0, 0.8;
    88, 82, 10,  8.0, 0.7
];

cfg.baseWind = [0.45, -0.18, 0.04];
cfg.windSwirl = 0.70;

% Continuous planner parameters.
cfg.safetyClearance = 1.6;       % building inflation in horizontal plane
cfg.cornerOffset = 1.2;          % extra outside-corner vertex offset
cfg.roundRadius = 3.0;           % maximum rounded-corner tangent length
cfg.routeSampleStep = 0.45;      % display/animation sample spacing
cfg.roadmapStep = 8.0;           % sparse continuous waypoint spacing for robust visibility graph
cfg.cruiseAltitude = 7.0;
cfg.maxCruiseAltitude = 10.0;    % keep low; this planner goes around buildings
cfg.climbWeight = 10.0;
cfg.threatWeight = 2.0;
cfg.sharedWeight = 0.9;          % mild route-sharing penalty; never dominate path length
cfg.windWeight = 0.7;
cfg.turnWeight = 0.22;
cfg.selectionSharedWeight = 0.18; % post-planning score weight for path overlap
cfg.selectionTurnWeight = 0.08;   % post-planning score weight for angular roughness
cfg.selectionClearanceWeight = 0.48;
cfg.animationPause = 0.015;
cfg.animationStride = 2;
cfg.useManualScene = false;
end

function cfg = chooseContinuousScenario(cfg)
fprintf('=============================================\n');
fprintf('Continuous Multi-UAV Planner\n');
fprintf('1) 预设大范围城市地图（推荐）\n');
fprintf('2) 手动绘制建筑物、起点和共同终点\n');
fprintf('=============================================\n');
raw = input('请选择场景模式 [1/2, 默认1]: ', 's');
if ~isempty(raw) && str2double(raw) == 2
    cfg = inputContinuousScene(cfg);
else
    fprintf('已选择: 预设大范围城市地图。\n');
end
end

function cfg = inputContinuousScene(cfg)
Nx = cfg.mapSize(1); Ny = cfg.mapSize(2); Nz = cfg.mapSize(3);
fig = figure('Color', 'w', 'Name', 'Manual Continuous Scene', 'NumberTitle', 'off');
ax = axes(fig); hold(ax, 'on'); grid(ax, 'on'); axis(ax, [1 Nx 1 Ny]); axis(ax, 'equal');
xlabel(ax, 'X'); ylabel(ax, 'Y');
title(ax, '左键点击建筑矩形两个对角点；右键/回车结束。然后点击无人机起点和共同终点。');
rects = zeros(0, 5);
while true
    [x1, y1, b1] = ginput(1);
    if isempty(b1) || b1 ~= 1, break; end
    [x2, y2, b2] = ginput(1);
    if isempty(b2) || b2 ~= 1, break; end
    xa = min(max(round(min(x1, x2)), 1), Nx); xb = min(max(round(max(x1, x2)), 1), Nx);
    ya = min(max(round(min(y1, y2)), 1), Ny); yb = min(max(round(max(y1, y2)), 1), Ny);
    h = askIntDialog(sprintf('建筑 x=[%d,%d], y=[%d,%d] 高度 [3..%d]:', xa, xb, ya, yb, Nz), 14, 3, Nz);
    rects = [rects; xa xb ya yb h]; %#ok<AGROW>
    patch(ax, [xa xb xb xa], [ya ya yb yb], [0.45 0.45 0.45], 'FaceAlpha', 0.25, 'EdgeColor', [0.2 0.2 0.2]);
end
if ~isempty(rects)
    cfg.buildingRects = rects;
end
n = askIntDialog(sprintf('无人机数量 [1..%d]:', cfg.maxUav), cfg.numUav, 1, cfg.maxUav);
cfg.numUav = n;
cfg.starts = zeros(n, 3);
for u = 1:n
    title(ax, sprintf('点击 UAV%d 起点', u));
    [x, y] = ginput(1);
    z = askIntDialog(sprintf('UAV%d 起点高度 [%d..%d]:', u, cfg.flyRange(1), cfg.flyRange(2)), cfg.cruiseAltitude, cfg.flyRange(1), cfg.flyRange(2));
    cfg.starts(u, :) = [round(x), round(y), z];
    scatter(ax, cfg.starts(u, 1), cfg.starts(u, 2), 60, 'filled');
end
title(ax, '点击共同目标点');
[x, y] = ginput(1);
z = askIntDialog(sprintf('共同目标高度 [%d..%d]:', cfg.flyRange(1), cfg.flyRange(2)), cfg.cruiseAltitude, cfg.flyRange(1), cfg.flyRange(2));
cfg.goal = [round(x), round(y), z];
close(fig);
end

function env = buildContinuousEnvironment(cfg)
Nx = cfg.mapSize(1); Ny = cfg.mapSize(2); Nz = cfg.mapSize(3);
env.footprint = false(Nx, Ny);
env.heightMap = zeros(Nx, Ny);
for i = 1:size(cfg.buildingRects, 1)
    r = cfg.buildingRects(i, :);
    xa = r(1); xb = r(2); ya = r(3); yb = r(4); h = min(r(5), Nz);
    env.footprint(xa:xb, ya:yb) = true;
    env.heightMap(xa:xb, ya:yb) = max(env.heightMap(xa:xb, ya:yb), h);
end
env.inflatedRects = inflateRects(cfg.buildingRects, cfg.safetyClearance, cfg.mapSize);

env.threat2 = zeros(Nx, Ny);
[X2, Y2] = ndgrid(1:Nx, 1:Ny);
for i = 1:size(cfg.threatRegions, 1)
    r = cfg.threatRegions(i, :);
    D = sqrt((X2-r(1)).^2 + (Y2-r(2)).^2);
    env.threat2 = env.threat2 + r(5) * max(0, 1 - D ./ max(r(4), 1e-6));
end
env.distToBuilding2 = distanceToMask2D(env.footprint);
end

function rects = inflateRects(rectsIn, margin, mapSize)
rects = rectsIn;
rects(:, 1) = max(1, rectsIn(:, 1) - margin);
rects(:, 2) = min(mapSize(1), rectsIn(:, 2) + margin);
rects(:, 3) = max(1, rectsIn(:, 3) - margin);
rects(:, 4) = min(mapSize(2), rectsIn(:, 4) + margin);
end

function validateContinuousScene(cfg, env)
for u = 1:cfg.numUav
    p = min(max(round(cfg.starts(u, 1:2)), [1 1]), cfg.mapSize(1:2));
    if env.footprint(p(1), p(2))
        error('UAV%d start [%d,%d] is inside a building. Move it outside buildings.', u, p(1), p(2));
    end
end
g = min(max(round(cfg.goal(1:2)), [1 1]), cfg.mapSize(1:2));
if env.footprint(g(1), g(2))
    error('Common goal [%d,%d] is inside a building. Move it outside buildings.', g(1), g(2));
end
end

function result = planContinuousMultiUAV(cfg, env)
N = cfg.numUav;
sharedPolylines = {};
result.routes = cell(N, 1);
result.polylines = cell(N, 1);
result.metrics = repmat(emptyMetrics(), N, 1);
result.clearanceUsed = zeros(N, 1);
result.routeMode = cell(N, 1);

for u = 1:N
    [poly, routeRects, ctrl, clearanceUsed, routeMode] = planOneContinuousRoute(cfg, env, u, sharedPolylines);
    if isempty(poly)
        warning('UAV%d failed to find a low-altitude horizontal route; using emergency high-altitude transition.', u);
        poly = [cfg.starts(u, 1:2); cfg.goal(1:2)];
        routeRects = zeros(0, 4);
        clearanceUsed = -1.0;
        routeMode = 'emergency-overflight';
    end

    poly = removeRedundantVertices(poly, routeRects);
    smooth2 = roundedPolyline(poly, routeRects, ctrl.roundRadius, cfg.routeSampleStep);
    smooth2 = repairAndValidatePolyline(smooth2, poly, routeRects, cfg.routeSampleStep);

    if clearanceUsed < 0
        route = emergencyHighRoute3D(cfg, smooth2, cfg.starts(u, :), cfg.goal);
    else
        z = lowClimbProfile(cfg, env, smooth2, cfg.starts(u, 3), cfg.goal(3));
        route = [smooth2, z];
        route(1, :) = cfg.starts(u, :);
        route(end, :) = cfg.goal;
    end

    result.polylines{u} = poly;
    result.routes{u} = route;
    result.metrics(u) = computeMetrics(cfg, env, route);
    result.clearanceUsed(u) = clearanceUsed;
    result.routeMode{u} = routeMode;
    sharedPolylines{end + 1} = smooth2; %#ok<AGROW>
end
end

function [poly, routeRects, ctrl, clearanceUsed, routeMode] = planOneContinuousRoute(cfg, env, u, sharedPolylines)
% Build several continuous-route candidates, then choose the shortest smooth
% route with only a mild penalty for overlap. This prevents late UAVs from
% taking huge border detours just to avoid already planned paths.
ctrl = adaptiveContinuousPolicy(cfg, env, u, sharedPolylines);
startXY = cfg.starts(u, 1:2);
goalXY = cfg.goal(1:2);
clearanceList = [cfg.safetyClearance, 1.2, 0.8, 0.4, 0.0];
clearanceList = unique(max(clearanceList, 0), 'stable');

poly = zeros(0, 2);
routeRects = env.inflatedRects;
clearanceUsed = cfg.safetyClearance;
routeMode = 'visibility';
bestScore = inf;

ctrlVariants = makeRouteControlVariants(ctrl);

for k = 1:numel(clearanceList)
    localEnv = env;
    localEnv.inflatedRects = inflateRects(cfg.buildingRects, clearanceList(k), cfg.mapSize);
    for cidx = 1:numel(ctrlVariants)
        trial = visibilityGraphRoute(cfg, localEnv, startXY, goalXY, sharedPolylines, ctrlVariants(cidx));
        if isempty(trial)
            continue;
        end

        trial = removeRedundantVertices(trial, localEnv.inflatedRects);
        if isPolylineFree(trial, localEnv.inflatedRects)
            score = routeSelectionScore(cfg, localEnv, trial, sharedPolylines, clearanceList(k));
            if score < bestScore
                bestScore = score;
                poly = trial;
                routeRects = localEnv.inflatedRects;
                clearanceUsed = clearanceList(k);
                ctrl = ctrlVariants(cidx);
                if clearanceList(k) == cfg.safetyClearance
                    routeMode = 'visibility';
                else
                    routeMode = 'relaxed-visibility';
                end
            end
        end
    end
end

if ~isempty(poly)
    return;
end

hardRects = inflateRects(cfg.buildingRects, 0.0, cfg.mapSize);
trial = fallbackPerimeterRoute(cfg, startXY, goalXY, hardRects);
if ~isempty(trial)
    poly = trial;
    routeRects = hardRects;
    clearanceUsed = 0.0;
    routeMode = 'perimeter-fallback';
    return;
end

trial = fallbackCoarseFreeRoute(cfg, startXY, goalXY, hardRects);
if ~isempty(trial)
    poly = trial;
    routeRects = hardRects;
    clearanceUsed = 0.0;
    routeMode = 'coarse-free-fallback';
end
end

function variants = makeRouteControlVariants(ctrl)
variants = repmat(ctrl, 1, 3);

% 1) learned/adaptive setting
variants(1) = ctrl;

% 2) short-route priority: ignore previous route overlap during graph search.
variants(2) = ctrl;
variants(2).sharedWeight = 0.0;
variants(2).clearanceWeight = max(0.9, 0.65 * ctrl.clearanceWeight);
variants(2).roundRadius = 1.12 * ctrl.roundRadius;

% 3) balanced setting: still discourages overlap, but not enough to dominate
% path length.
variants(3) = ctrl;
variants(3).sharedWeight = 0.35 * ctrl.sharedWeight;
variants(3).clearanceWeight = max(1.0, 0.85 * ctrl.clearanceWeight);
variants(3).roundRadius = 1.05 * ctrl.roundRadius;
end

function score = routeSelectionScore(cfg, env, poly, sharedPolylines, clearanceUsed)
if isempty(poly) || size(poly, 1) < 2
    score = inf;
    return;
end

seg = diff(poly, 1, 1);
len = sum(vecnorm(seg, 2, 2));
directLen = norm(poly(end, :) - poly(1, :));
detour = len / max(directLen, 1e-6);

dense = densify2D(poly, max(1.0, cfg.routeSampleStep * 3.0));
clearPenalty = 0;
threatPenalty = 0;
sharedPenalty = 0;
for i = 1:size(dense, 1)
    x = min(max(round(dense(i, 1)), 1), cfg.mapSize(1));
    y = min(max(round(dense(i, 2)), 1), cfg.mapSize(2));
    clearPenalty = clearPenalty + exp(-env.distToBuilding2(x, y) / 3.0);
    threatPenalty = threatPenalty + env.threat2(x, y);
    sharedPenalty = sharedPenalty + sharedDistancePenalty(dense(i, :), sharedPolylines);
end
clearPenalty = clearPenalty / max(size(dense, 1), 1);
threatPenalty = threatPenalty / max(size(dense, 1), 1);
sharedPenalty = sharedPenalty / max(size(dense, 1), 1);

turnPenalty = estimatePolylineTurn(poly);
relaxPenalty = 0.08 * max(0, cfg.safetyClearance - clearanceUsed);

% Large detours are visually obvious and usually not worth a small separation
% gain, so penalize detour ratio after collision-free candidates are generated.
detourPenalty = 1.8 * max(0, detour - 1.28)^2;

score = len * (1 + cfg.selectionClearanceWeight * clearPenalty + ...
    cfg.threatWeight * 0.35 * threatPenalty + ...
    cfg.selectionSharedWeight * sharedPenalty + ...
    cfg.selectionTurnWeight * turnPenalty + relaxPenalty + detourPenalty);
end

function turnPenalty = estimatePolylineTurn(poly)
turnPenalty = 0;
if size(poly, 1) < 3
    return;
end
for i = 2:size(poly, 1)-1
    a = poly(i, :) - poly(i - 1, :);
    b = poly(i + 1, :) - poly(i, :);
    if norm(a) < 1e-9 || norm(b) < 1e-9
        continue;
    end
    c = dot(a, b) / (norm(a) * norm(b));
    c = max(-1, min(1, c));
    turnPenalty = turnPenalty + acos(c) / pi;
end
end

function poly = fallbackPerimeterRoute(cfg, startXY, goalXY, rects)
% Conservative last-resort route through broad outer corridors. This is not
% used unless the visibility graph disconnects under all clearance levels.
sx = startXY(1); sy = startXY(2); gx = goalXY(1); gy = goalXY(2);
mx = cfg.mapSize(1); my = cfg.mapSize(2);
margin = 2.0;
candidatePolys = {
    [startXY; sx, margin; gx, margin; goalXY], ...
    [startXY; sx, my-margin; gx, my-margin; goalXY], ...
    [startXY; margin, sy; margin, gy; goalXY], ...
    [startXY; mx-margin, sy; mx-margin, gy; goalXY], ...
    [startXY; margin, sy; margin, margin; gx, margin; goalXY], ...
    [startXY; mx-margin, sy; mx-margin, my-margin; gx, my-margin; goalXY]
};

poly = zeros(0, 2);
bestLen = inf;
for i = 1:numel(candidatePolys)
    p = double(candidatePolys{i});
    p(:, 1) = min(max(p(:, 1), 1), cfg.mapSize(1));
    p(:, 2) = min(max(p(:, 2), 1), cfg.mapSize(2));
    p = uniqueConsecutiveRows(p);
    if isPolylineFree(p, rects)
        L = sum(vecnorm(diff(p), 2, 2));
        if L < bestLen
            bestLen = L;
            poly = p;
        end
    end
end
end

function poly = fallbackCoarseFreeRoute(cfg, startXY, goalXY, rects)
% Last-resort 2D free-space search. The result is immediately simplified by
% line-of-sight pruning, so it is only a safety net for disconnected cases.
Nx = cfg.mapSize(1); Ny = cfg.mapSize(2);
blocked = false(Nx, Ny);
for i = 1:size(rects, 1)
    r = rects(i, :);
    xa = max(1, ceil(r(1))); xb = min(Nx, floor(r(2)));
    ya = max(1, ceil(r(3))); yb = min(Ny, floor(r(4)));
    if xa <= xb && ya <= yb
        blocked(xa:xb, ya:yb) = true;
    end
end

s = min(max(round(startXY), [1 1]), [Nx Ny]);
g = min(max(round(goalXY), [1 1]), [Nx Ny]);
if blocked(s(1), s(2)) || blocked(g(1), g(2))
    poly = zeros(0, 2);
    return;
end

parent = zeros(Nx, Ny, 'uint32');
seen = false(Nx, Ny);
q = zeros(Nx*Ny, 2, 'uint16');
head = 1; tail = 1;
q(tail, :) = uint16(s);
seen(s(1), s(2)) = true;
M = [1 0; -1 0; 0 1; 0 -1; 1 1; 1 -1; -1 1; -1 -1];

while head <= tail
    c = double(q(head, :));
    head = head + 1;
    if all(c == g)
        break;
    end
    for k = 1:size(M, 1)
        n = c + M(k, :);
        if n(1) < 1 || n(1) > Nx || n(2) < 1 || n(2) > Ny
            continue;
        end
        if seen(n(1), n(2)) || blocked(n(1), n(2))
            continue;
        end
        tail = tail + 1;
        q(tail, :) = uint16(n);
        seen(n(1), n(2)) = true;
        parent(n(1), n(2)) = uint32(sub2ind([Nx, Ny], c(1), c(2)));
    end
end

if ~seen(g(1), g(2))
    poly = zeros(0, 2);
    return;
end

cur = g;
poly = cur;
while any(cur ~= s)
    pid = parent(cur(1), cur(2));
    [px, py] = ind2sub([Nx, Ny], double(pid));
    cur = [px, py];
    poly = [cur; poly]; %#ok<AGROW>
end

poly(1, :) = startXY;
poly(end, :) = goalXY;
poly = removeRedundantVertices(poly, rects);
if ~isPolylineFree(poly, rects)
    poly = zeros(0, 2);
end
end

function ctrl = adaptiveContinuousPolicy(cfg, env, u, sharedPolylines)
% Lightweight PPO-style high-level policy surrogate. It does not replace the
% geometry planner; it adapts safety/rounding/separation preferences per UAV.
density = nnz(env.footprint) / numel(env.footprint);
sharedLevel = min(1, numel(sharedPolylines) / max(cfg.numUav, 1));
threatLevel = mean(env.threat2(:));
logits = [
    1.1 - 1.2*density - 0.4*threatLevel, ...
    0.5 + 0.5*density + 0.2*sharedLevel, ...
    0.1 + 1.1*sharedLevel + 0.4*threatLevel
];
prob = softmax(logits);
[~, action] = max(prob);
switch action
    case 1
        ctrl.clearanceWeight = 1.6;
        ctrl.sharedWeight = cfg.sharedWeight * 0.75;
        ctrl.roundRadius = cfg.roundRadius * 1.25;
    case 2
        ctrl.clearanceWeight = 2.2;
        ctrl.sharedWeight = cfg.sharedWeight;
        ctrl.roundRadius = cfg.roundRadius;
    otherwise
        ctrl.clearanceWeight = 2.8;
        ctrl.sharedWeight = cfg.sharedWeight * 1.45;
        ctrl.roundRadius = cfg.roundRadius * 0.9;
end
ctrl.threatWeight = cfg.threatWeight;
ctrl.windWeight = cfg.windWeight;
ctrl.turnWeight = cfg.turnWeight;
ctrl.uavIndex = u;
end

function poly = visibilityGraphRoute(cfg, env, startXY, goalXY, sharedPolylines, ctrl)
nodes = buildVisibilityNodes(cfg, env, startXY, goalXY);
N = size(nodes, 1);
W = inf(N, N);
for i = 1:N
    for j = i+1:N
        if segmentFree(nodes(i, :), nodes(j, :), env.inflatedRects)
            c = segmentCost(cfg, env, nodes(i, :), nodes(j, :), sharedPolylines, ctrl);
            W(i, j) = c; W(j, i) = c;
        end
    end
end
pathIdx = dijkstraDense(W, 1, 2);
if isempty(pathIdx)
    poly = zeros(0, 2);
else
    poly = nodes(pathIdx, :);
end
end

function nodes = buildVisibilityNodes(cfg, env, startXY, goalXY)
nodes = [double(startXY); double(goalXY)];
pad = cfg.cornerOffset;
for i = 1:size(env.inflatedRects, 1)
    r = env.inflatedRects(i, :);
    cands = [
        r(1)-pad, r(3)-pad;
        r(2)+pad, r(3)-pad;
        r(2)+pad, r(4)+pad;
        r(1)-pad, r(4)+pad
    ];
    for k = 1:4
        p = min(max(cands(k, :), [1 1]), cfg.mapSize(1:2));
        if ~pointInAnyRect(p, env.inflatedRects, 1e-7)
            nodes = [nodes; p]; %#ok<AGROW>
        end
    end
end

% Sparse free-space roadmap nodes improve connectivity in large maps without
% forcing the final route to follow grid-like A* steps.
xs = 4:cfg.roadmapStep:cfg.mapSize(1)-3;
ys = 4:cfg.roadmapStep:cfg.mapSize(2)-3;
for ix = 1:numel(xs)
    for iy = 1:numel(ys)
        p = [xs(ix), ys(iy)];
        if ~pointInAnyRect(p, env.inflatedRects, 1e-7)
            nodes = [nodes; p]; %#ok<AGROW>
        end
    end
end

nodes = unique(round(nodes, 4), 'rows', 'stable');
end

function cost = segmentCost(cfg, env, a, b, sharedPolylines, ctrl)
d = norm(b - a);
n = max(3, ceil(d / 1.5));
t = linspace(0, 1, n)';
pts = (1 - t) .* a + t .* b;
clearCost = 0; threatCost = 0; sharedCost = 0; windCost = 0;
for i = 1:n
    x = min(max(round(pts(i, 1)), 1), cfg.mapSize(1));
    y = min(max(round(pts(i, 2)), 1), cfg.mapSize(2));
    clearCost = clearCost + exp(-env.distToBuilding2(x, y) / 4.0);
    threatCost = threatCost + env.threat2(x, y);
    sharedCost = sharedCost + sharedDistancePenalty(pts(i, :), sharedPolylines);
end
clearCost = clearCost / n;
threatCost = threatCost / n;
sharedCost = sharedCost / n;
dir = (b - a) / max(d, 1e-9);
wind2 = cfg.baseWind(1:2) + cfg.windSwirl * 0.1 * [sin(0.04*a(2)), cos(0.04*a(1))];
windCost = max(0, dot(wind2, dir));
cost = d * (1 + ctrl.clearanceWeight*clearCost + ctrl.threatWeight*threatCost + ctrl.sharedWeight*sharedCost + ctrl.windWeight*windCost);
end

function p = sharedDistancePenalty(pt, sharedPolylines)
if isempty(sharedPolylines)
    p = 0;
    return;
end
best = inf;
for i = 1:numel(sharedPolylines)
    q = sharedPolylines{i};
    if isempty(q), continue; end
    d = min(vecnorm(q - pt, 2, 2));
    best = min(best, d);
end
p = exp(-best / 5.0);
end

function idxPath = dijkstraDense(W, s, g)
N = size(W, 1);
d = inf(N, 1); prev = zeros(N, 1); used = false(N, 1);
d(s) = 0;
for iter = 1:N
    candidates = find(~used);
    if isempty(candidates), break; end
    [~, k] = min(d(candidates));
    u = candidates(k);
    if ~isfinite(d(u)), break; end
    used(u) = true;
    if u == g, break; end
    nbr = find(isfinite(W(u, :)) & ~used');
    for v = nbr
        nd = d(u) + W(u, v);
        if nd < d(v)
            d(v) = nd;
            prev(v) = u;
        end
    end
end
if ~isfinite(d(g))
    idxPath = [];
    return;
end
idxPath = g;
while idxPath(1) ~= s
    idxPath = [prev(idxPath(1)); idxPath]; %#ok<AGROW>
end
end

function poly = removeRedundantVertices(poly, rects)
if size(poly, 1) <= 2, return; end
out = poly(1, :); i = 1;
while i < size(poly, 1)
    j = size(poly, 1);
    while j > i + 1
        if segmentFree(poly(i, :), poly(j, :), rects)
            break;
        end
        j = j - 1;
    end
    out = [out; poly(j, :)]; %#ok<AGROW>
    i = j;
end
poly = out;
end

function smooth = roundedPolyline(poly, rects, roundRadius, sampleStep)
poly = double(uniqueConsecutiveRows(poly));
if size(poly, 1) <= 2
    smooth = densify2D(poly, sampleStep);
    return;
end
smooth = poly(1, :);
for i = 2:size(poly, 1)-1
    A = poly(i-1, :); B = poly(i, :); C = poly(i+1, :);
    v1 = B - A; v2 = C - B;
    n1 = norm(v1); n2 = norm(v2);
    if n1 < 1e-9 || n2 < 1e-9
        continue;
    end
    u1 = v1 / n1; u2 = v2 / n2;
    turn = acos(max(-1, min(1, dot(u1, u2))));
    if turn < deg2radLocal(8)
        appendSeg = densify2D([smooth(end, :); B], sampleStep);
        smooth = [smooth; appendSeg(2:end, :)]; %#ok<AGROW>
        continue;
    end
    cut = min([roundRadius, 0.42*n1, 0.42*n2]);
    P = B - cut*u1; Q = B + cut*u2;
    if ~segmentFree(smooth(end, :), P, rects)
        P = B;
    end
    curve = bezier2(P, B, Q, max(8, ceil((norm(P-B)+norm(Q-B))/sampleStep)));
    if isPolylineFree(curve, rects) && segmentFree(smooth(end, :), curve(1, :), rects)
        link = densify2D([smooth(end, :); curve(1, :)], sampleStep);
        smooth = [smooth; link(2:end, :); curve(2:end, :)]; %#ok<AGROW>
    else
        link = densify2D([smooth(end, :); B], sampleStep);
        smooth = [smooth; link(2:end, :)]; %#ok<AGROW>
    end
end
link = densify2D([smooth(end, :); poly(end, :)], sampleStep);
smooth = [smooth; link(2:end, :)];
smooth = uniqueConsecutiveRows(smooth);
end

function smooth = repairAndValidatePolyline(smooth, poly, rects, sampleStep)
if isPolylineFree(smooth, rects)
    return;
end
smooth = densify2D(poly, sampleStep);
if isPolylineFree(smooth, rects)
    return;
end
% Final conservative fallback: use original visibility graph vertices only.
smooth = poly;
if ~isPolylineFree(smooth, rects)
    error('Continuous path repair failed: route intersects inflated buildings. Increase map clearance or move start/goal.');
end
end

function curve = bezier2(P0, P1, P2, n)
t = linspace(0, 1, n)';
curve = (1-t).^2 .* P0 + 2*(1-t).*t .* P1 + t.^2 .* P2;
end

function dense = densify2D(poly, step)
poly = double(uniqueConsecutiveRows(poly));
dense = poly(1, :);
for i = 2:size(poly, 1)
    a = dense(end, :); b = poly(i, :);
    n = max(2, ceil(norm(b-a) / max(step, 1e-6)));
    t = linspace(0, 1, n)';
    seg = (1-t).*a + t.*b;
    dense = [dense; seg(2:end, :)]; %#ok<AGROW>
end
end

function ok = isPolylineFree(poly, rects)
ok = true;
for i = 2:size(poly, 1)
    if ~segmentFree(poly(i-1, :), poly(i, :), rects)
        ok = false;
        return;
    end
end
end

function ok = segmentFree(a, b, rects)
d = norm(b - a);
n = max(4, ceil(d * 2.5));
t = linspace(0, 1, n)';
pts = (1-t).*a + t.*b;
ok = true;
for i = 1:n
    if pointInAnyRect(pts(i, :), rects, 1e-7)
        ok = false;
        return;
    end
end
end

function yes = pointInAnyRect(p, rects, tol)
yes = false;
for i = 1:size(rects, 1)
    r = rects(i, :);
    if p(1) > r(1)+tol && p(1) < r(2)-tol && p(2) > r(3)+tol && p(2) < r(4)-tol
        yes = true;
        return;
    end
end
end

function z = lowClimbProfile(cfg, env, xy, zStart, zGoal)
N = size(xy, 1);
base = min(max(cfg.cruiseAltitude, cfg.flyRange(1)), min(cfg.flyRange(2), cfg.maxCruiseAltitude));
z = base * ones(N, 1);
for i = 1:N
    x = min(max(round(xy(i, 1)), 1), cfg.mapSize(1));
    y = min(max(round(xy(i, 2)), 1), cfg.mapSize(2));
    z(i) = min(cfg.maxCruiseAltitude, base + 0.9 * env.threat2(x, y));
end
z = movingAverage(z, max(5, round(N / 35)));
z(1) = zStart; z(end) = zGoal;
z = min(max(z, cfg.flyRange(1)), cfg.flyRange(2));
end

function route = emergencyHighRoute3D(cfg, xy, startPt, goalPt)
% Last-resort route: climb vertically first, cross above the city, then
% descend vertically at the common goal. This keeps the demo running when
% the horizontal low-altitude free space is topologically disconnected.
if isempty(xy)
    xy = [startPt(1:2); goalPt(1:2)];
end
xy = densify2D(double(xy), max(cfg.routeSampleStep, 0.45));

if isempty(cfg.buildingRects)
    maxH = cfg.cruiseAltitude;
else
    maxH = max(cfg.buildingRects(:, 5));
end
highZ = min(cfg.flyRange(2), max([cfg.maxCruiseAltitude + 4, maxH + 3, startPt(3), goalPt(3)]));
highZ = max(highZ, cfg.flyRange(1));

nAsc = max(8, ceil(abs(highZ - startPt(3)) / 0.6));
nDesc = max(8, ceil(abs(highZ - goalPt(3)) / 0.6));
asc = [repmat(startPt(1:2), nAsc, 1), linspace(startPt(3), highZ, nAsc)'];
cruise = [xy, highZ * ones(size(xy, 1), 1)];
desc = [repmat(goalPt(1:2), nDesc, 1), linspace(highZ, goalPt(3), nDesc)'];

route = [asc; cruise(2:end-1, :); desc];
route(1, :) = startPt;
route(end, :) = goalPt;
end

function out = movingAverage(v, win)
win = max(3, win + mod(win+1, 2)); half = floor(win/2); out = v;
for i = 1:numel(v)
    a = max(1, i-half); b = min(numel(v), i+half);
    out(i) = mean(v(a:b));
end
end

function m = computeMetrics(cfg, env, route)
m = emptyMetrics();
for i = 2:size(route, 1)
    p0 = route(i-1, :); p1 = route(i, :); step = p1 - p0; ds = norm(step);
    if ds < 1e-9, continue; end
    m.length = m.length + ds;
    m.climb = m.climb + max(step(3), 0);
    m.descend = m.descend + max(-step(3), 0);
    x = min(max(round(p1(1)), 1), cfg.mapSize(1)); y = min(max(round(p1(2)), 1), cfg.mapSize(2));
    m.risk = m.risk + env.threat2(x, y) * ds;
    wind2 = cfg.baseWind(1:2) + cfg.windSwirl * 0.1 * [sin(0.04*y), cos(0.04*x)];
    dir = step(1:2) / max(norm(step(1:2)), 1e-9);
    m.wind = m.wind + max(0, dot(wind2, dir));
    if i >= 3
        a = route(i-1, :) - route(i-2, :); b = step;
        if norm(a) > 1e-9 && norm(b) > 1e-9
            ang = acosd(max(-1, min(1, dot(a,b)/(norm(a)*norm(b)))));
            m.totalTurn = m.totalTurn + ang;
            m.maxTurn = max(m.maxTurn, ang);
        end
    end
end
m.energy = m.length + cfg.climbWeight*m.climb + 0.8*cfg.climbWeight*m.descend + 2.0*m.risk + cfg.windWeight*m.wind;
m.smooth = m.totalTurn / max(m.length, 1e-6);
end

function m = emptyMetrics()
m.length = 0; m.energy = 0; m.risk = 0; m.wind = 0; m.climb = 0; m.descend = 0;
m.totalTurn = 0; m.maxTurn = 0; m.smooth = 0;
end

function printContinuousSummary(result)
fprintf('==== Continuous Multi-UAV Route Summary ====\n');
for u = 1:numel(result.routes)
    m = result.metrics(u);
    fprintf('UAV%d | mode=%s | samples=%4d | clearance=%.1f | length=%.2f | energy=%.2f | climb=%.2f | risk=%.2f | smooth=%.3f | maxTurn=%.1f deg\n', ...
        u, result.routeMode{u}, size(result.routes{u}, 1), result.clearanceUsed(u), m.length, m.energy, m.climb, m.risk, m.smooth, m.maxTurn);
end
end

function animateContinuousRoutes(cfg, env, result)
fig = figure('Color', 'w', 'Name', 'Continuous Multi-UAV Planning', 'NumberTitle', 'off', ...
    'Position', [70, 70, 1380, 830], 'Renderer', 'opengl');
ax = axes(fig); hold(ax, 'on'); grid(ax, 'on');
axis(ax, [1 cfg.mapSize(1) 1 cfg.mapSize(2) 0 cfg.mapSize(3)]);
axis(ax, 'vis3d'); view(ax, 42, 28); camproj(ax, 'perspective'); rotate3d(fig, 'on');
set(ax, 'SortMethod', 'childorder', 'FontSize', 11);
xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z');
title(ax, '连续空间多无人机协同规划：可视图 + 圆角轨迹 + 低爬升');

drawGround(ax, cfg);
drawBuildings(ax, cfg);
drawThreats(ax, cfg);
plot3(ax, cfg.goal(1), cfg.goal(2), cfg.goal(3), 'p', 'MarkerSize', 16, 'MarkerFaceColor', [0.9 0.05 0.05], 'MarkerEdgeColor', 'k', 'LineWidth', 1.4, 'Clipping', 'off');
text(ax, cfg.goal(1)+1, cfg.goal(2)+1, cfg.goal(3)+1, '共同目标', 'Color', [0.7 0 0], 'FontWeight', 'bold', 'Clipping', 'off');

colors = lines(max(7, cfg.numUav));
N = cfg.numUav;
trailH = gobjects(N, 1); droneH = gobjects(N, 1); labelH = gobjects(N, 1);
for u = 1:N
    r = result.routes{u};
    plot3(ax, r(:,1), r(:,2), r(:,3)+0.12, '-', 'Color', 0.70*colors(u,:)+0.30*[1 1 1], 'LineWidth', 1.4, 'Clipping', 'off');
    trailH(u) = plot3(ax, r(1,1), r(1,2), r(1,3)+0.35, '-', 'Color', colors(u,:), 'LineWidth', 4.0, 'Clipping', 'off');
    droneH(u) = plot3(ax, r(1,1), r(1,2), r(1,3)+0.60, 'o', 'MarkerSize', 8, 'MarkerFaceColor', colors(u,:), 'MarkerEdgeColor', 'k', 'LineWidth', 0.9, 'Clipping', 'off');
    labelH(u) = text(ax, r(1,1)+0.8, r(1,2)+0.8, r(1,3)+1.1, sprintf('UAV%d', u), 'Color', colors(u,:), 'FontWeight', 'bold', 'Clipping', 'off');
end

maxLen = max(cellfun(@(r) size(r, 1), result.routes));
for k = 1:cfg.animationStride:maxLen
    for u = 1:N
        r = result.routes{u}; idx = min(k, size(r, 1));
        set(trailH(u), 'XData', r(1:idx,1), 'YData', r(1:idx,2), 'ZData', r(1:idx,3)+0.35);
        set(droneH(u), 'XData', r(idx,1), 'YData', r(idx,2), 'ZData', r(idx,3)+0.60);
        set(labelH(u), 'Position', [r(idx,1)+0.8, r(idx,2)+0.8, r(idx,3)+1.1]);
    end
    drawnow limitrate;
    pause(cfg.animationPause);
end
for u = 1:N
    r = result.routes{u};
    set(trailH(u), 'XData', r(:,1), 'YData', r(:,2), 'ZData', r(:,3)+0.35);
    set(droneH(u), 'XData', cfg.goal(1), 'YData', cfg.goal(2), 'ZData', cfg.goal(3)+0.60);
end
title(ax, '连续空间多无人机协同规划：全部到达共同目标');
drawnow;
end

function drawGround(ax, cfg)
patch(ax, 'XData', [1 cfg.mapSize(1) cfg.mapSize(1) 1], 'YData', [1 1 cfg.mapSize(2) cfg.mapSize(2)], ...
    'ZData', [0 0 0 0], 'FaceColor', [0.94 0.94 0.91], 'EdgeColor', 'none');
end

function drawBuildings(ax, cfg)
for i = 1:size(cfg.buildingRects, 1)
    r = cfg.buildingRects(i, :); h = r(5);
    if h >= 19
        roof = [0.92 0.18 0.14];
    elseif h >= 13
        roof = [0.96 0.65 0.10];
    else
        roof = [0.62 0.86 0.82];
    end
    drawBuildingBlock(ax, r(1), r(2), r(3), r(4), h, [0.72 0.76 0.82], 0.42, [0.36 0.38 0.42], roof);
end
end

function drawBuildingBlock(ax, xa, xb, ya, yb, h, faceColor, alphaVal, edgeColor, roofColor)
cx = (xa + xb) / 2; cy = (ya + yb) / 2; sx = xb - xa + 1; sy = yb - ya + 1;
drawCuboid(ax, cx, cy, 0, sx, sy, h, faceColor, edgeColor, alphaVal);
drawCuboid(ax, cx, cy, h + 0.05, 0.96*sx, 0.96*sy, 0.22, roofColor, edgeColor, 0.96);
if h >= 13 && sx > 3 && sy > 3
    drawCuboid(ax, cx + 0.18*sx, cy - 0.15*sy, h + 0.35, 0.25*sx, 0.22*sy, 0.7, [0.66 0.70 0.74], edgeColor, 0.95);
end
end

function drawCuboid(ax, cx, cy, z0, sx, sy, sz, faceColor, edgeColor, faceAlpha)
x0 = cx - sx/2; x1 = cx + sx/2; y0 = cy - sy/2; y1 = cy + sy/2; z1 = z0 + sz;
V = [x0 y0 z0; x1 y0 z0; x1 y1 z0; x0 y1 z0; x0 y0 z1; x1 y0 z1; x1 y1 z1; x0 y1 z1];
F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch(ax, 'Vertices', V, 'Faces', F, 'FaceColor', faceColor, 'FaceAlpha', faceAlpha, ...
    'EdgeColor', edgeColor, 'LineWidth', 0.26, 'Clipping', 'on');
end

function drawThreats(ax, cfg)
for i = 1:size(cfg.threatRegions, 1)
    r = cfg.threatRegions(i, :); [sx, sy, sz] = sphere(18); rr = 0.55 * r(4);
    surf(ax, r(1)+rr*sx, r(2)+rr*sy, r(3)+rr*sz, 'FaceColor', [1.0 0.18 0.18], ...
        'FaceAlpha', min(0.18, 0.07 + 0.08*r(5)), 'EdgeColor', 'none');
end
end

function plotContinuousCostReport(result)
N = numel(result.routes);
energies = zeros(N, 1);
for u = 1:N
    energies(u) = result.metrics(u).energy;
end
meanEnergy = mean(energies);
if meanEnergy < 1e-9
    meanEnergy = 1.0;
end

vals = zeros(N, 7);
for u = 1:N
    m = result.metrics(u);
    energyDev = abs(m.energy - meanEnergy) / meanEnergy * 100;
    vals(u, :) = [m.length, m.energy, m.risk, m.climb, m.smooth, m.maxTurn, energyDev];
end
fig = figure('Color', 'w', 'Name', 'Continuous Cost Report', 'Position', [130, 110, 980, 520]);
ax = axes(fig); bar(ax, vals, 'grouped'); grid(ax, 'on');
legend(ax, {'长度','能耗','风险','爬升','平滑度','最大转角','能耗一致性偏差(%)'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
xlabel(ax, '无人机编号'); ylabel(ax, '指标值'); title(ax, '连续空间多无人机路径代价对比');
end

function dist = distanceToMask2D(mask)
[Nx, Ny] = size(mask); INF = 1e9; dist = INF * ones(Nx, Ny);
q = zeros(numel(mask), 2, 'uint16'); head = 1; tail = 0;
[xs, ys] = find(mask);
for i = 1:numel(xs)
    tail = tail + 1; q(tail, :) = uint16([xs(i), ys(i)]); dist(xs(i), ys(i)) = 0;
end
if tail == 0, dist(:) = max(Nx, Ny); return; end
M = [1 0; -1 0; 0 1; 0 -1; 1 1; 1 -1; -1 1; -1 -1];
while head <= tail
    c = double(q(head, :)); head = head + 1;
    for k = 1:size(M, 1)
        nx = c(1)+M(k,1); ny = c(2)+M(k,2);
        if nx < 1 || nx > Nx || ny < 1 || ny > Ny, continue; end
        nd = dist(c(1), c(2)) + norm(M(k, :));
        if nd < dist(nx, ny)
            dist(nx, ny) = nd; tail = tail + 1; q(tail, :) = uint16([nx ny]);
        end
    end
end
end

function rows = uniqueConsecutiveRows(rows)
if isempty(rows), return; end
keep = [true; any(abs(diff(rows)) > 1e-9, 2)]; rows = rows(keep, :);
end

function p = softmax(logits)
z = logits - max(logits); e = exp(z); p = e ./ sum(e);
end

function a = deg2radLocal(d)
a = d * pi / 180;
end

function v = askIntDialog(prompt, defaultVal, lb, ub)
answ = inputdlg(prompt, 'Input', [1 60], {num2str(defaultVal)});
if isempty(answ)
    v = defaultVal;
else
    v = str2double(answ{1}); if isnan(v), v = defaultVal; end
end
v = min(max(round(v), lb), ub);
end
