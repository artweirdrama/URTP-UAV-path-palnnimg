function printParetoSummary(pareto)
fprintf('\n==== Pareto Path Summary ====\n');
for i = 1:numel(pareto)
    m = pareto(i).metrics;
    s = computeRouteStats(pareto(i).path, m);
    fprintf(['Path %2d | nodes=%3d | total=%.3f | len=%.3f | direct=%.3f | detour=%.3f | risk=%.3f | ', ...
             'energy=%.3f | smooth=%.3f | turns>45=%d | maxTurn=%.1f deg | time=%.3f\n'], ...
             i, size(pareto(i).path, 1), m.total, m.length, s.directDist, s.detourRatio, m.threat + m.safety, ...
             m.energy, m.smooth, s.turnOver45, s.maxTurnDeg, m.time);
end

fprintf('\nRecommended path (Path 1) weights:\n');
disp(pareto(1).weights);
end

function plotSceneAndParetoPaths(cfg, pareto)
fig = figure('Color', 'w', 'Position', [90, 85, 920, 730], 'Name', '3D Path Planning Visualization');
ax1 = axes(fig);
hold(ax1, 'on'); grid(ax1, 'on'); view(ax1, 38, 28);
axis(ax1, [1 cfg.mapSize(1) 1 cfg.mapSize(2) 1 cfg.mapSize(3)]);
xlabel(ax1, 'X轴'); ylabel(ax1, 'Y轴'); zlabel(ax1, 'Z轴');
title(ax1, '三维障碍物 / 威胁区 / 候选路径');

for i = 1:size(cfg.obstacles, 1)
    x = cfg.obstacles(i, 1);
    y = cfg.obstacles(i, 2);
    h = cfg.obstacles(i, 3);
    if h > 0
        drawColumn(x, y, h);
    end
end

for i = 1:size(cfg.threatRegions, 1)
    drawThreatSphere(cfg.threatRegions(i, :));
end

C = lines(max(3, numel(pareto)));
for i = 1:numel(pareto)
    p = pareto(i).path;
    lw = 1.3;
    if i == 1
        lw = 2.8;
    end
    plot3(ax1, p(:, 1), p(:, 2), p(:, 3), '-', 'Color', C(i, :), 'LineWidth', lw);
    if i <= 3
        idm = max(2, floor(size(p, 1) * (0.35 + 0.1 * i)));
        text(ax1, p(idm, 1), p(idm, 2), p(idm, 3) + 0.6, sprintf('路径%d', i), ...
            'Color', C(i, :), 'FontWeight', 'bold');
    end
end

plot3(ax1, cfg.start(1), cfg.start(2), cfg.start(3), 'o', ...
    'MarkerSize', 9, 'LineWidth', 2.0, 'MarkerEdgeColor', [0.0, 0.45, 0.0], 'MarkerFaceColor', [0.35, 0.95, 0.35]);
plot3(ax1, cfg.goal(1), cfg.goal(2), cfg.goal(3), 'p', ...
    'MarkerSize', 12, 'LineWidth', 2.0, 'MarkerEdgeColor', [0.7, 0.0, 0.0], 'MarkerFaceColor', [1.0, 0.3, 0.3]);
text(ax1, cfg.start(1), cfg.start(2), cfg.start(3) + 0.9, '起始点', 'FontWeight', 'bold');
text(ax1, cfg.goal(1), cfg.goal(2), cfg.goal(3) + 0.9, '目标点', 'FontWeight', 'bold');

bestPath = pareto(1).path;
bubbleStep = max(1, floor(size(bestPath, 1) / 7));
bubbleRadius = max(1.0, pareto(1).weights(5) * 0.55);
for k = 1:bubbleStep:size(bestPath, 1)
    drawSafetyBubble(bestPath(k, :), bubbleRadius);
end

set(ax1, 'FontSize', 11);
end

function plotCostFigures(pareto)
N = numel(pareto);
idx = 1:N;

lengthV = zeros(1, N);
riskV = zeros(1, N);
energyV = zeros(1, N);
smoothV = zeros(1, N);
timeV = zeros(1, N);
totalV = zeros(1, N);

for i = 1:N
    m = pareto(i).metrics;
    lengthV(i) = m.length;
    riskV(i) = m.threat + m.safety;
    energyV(i) = m.energy;
    smoothV(i) = m.smooth;
    timeV(i) = m.time;
    totalV(i) = m.total;
end

plotOneCostFigure(idx, lengthV, '路径长度', '长度代价', [0.10, 0.48, 0.80]);
plotOneCostFigure(idx, riskV, '风险成本', '风险代价', [0.90, 0.35, 0.10]);
plotOneCostFigure(idx, energyV, '能耗成本', '能耗代价', [0.95, 0.72, 0.10]);
plotOneCostFigure(idx, smoothV, '平滑成本', '平滑代价', [0.58, 0.22, 0.70]);
plotOneCostFigure(idx, timeV, '时间成本', '时间代价', [0.25, 0.68, 0.10]);
plotOneCostFigure(idx, totalV, '总成本', '总成本', [0.08, 0.15, 0.18]);
end

function plotOneCostFigure(idx, values, figName, yName, colorV)
fig = figure('Color', 'w', 'Position', [1040, 90, 620, 390], 'Name', ['成本分析 - ' figName]);
ax = axes(fig);
bar(ax, idx, values, 0.62, 'FaceColor', colorV, 'EdgeColor', [0.2, 0.2, 0.2]);
hold(ax, 'on'); grid(ax, 'on');
[vmin, id] = min(values);
bar(ax, idx(id), values(id), 0.62, 'FaceColor', [0.10, 0.78, 0.15], ...
    'EdgeColor', [0.1, 0.3, 0.1], 'LineWidth', 1.2);
text(ax, idx(id), vmin, sprintf('  最优 Path%d = %.3f', idx(id), vmin), ...
    'FontWeight', 'bold', 'Color', [0.1, 0.35, 0.1]);
title(ax, [figName '（越小越优）']);
xlabel(ax, 'Pareto路径编号');
ylabel(ax, yName);
xlim(ax, [0.3, numel(idx) + 0.7]);
set(ax, 'FontSize', 11);
end

function drawColumn(x, y, h)
[Xc, Yc, Zc] = cylinder(0.42, 12);
Zc = Zc * h;
surf(Xc + x, Yc + y, Zc + 1, 'FaceColor', [1.0, 0.95, 0.1], 'FaceAlpha', 0.55, ...
    'EdgeColor', [0.35, 0.35, 0.35], 'EdgeAlpha', 0.4);
end

function drawThreatSphere(region)
cx = region(1); cy = region(2); cz = region(3); r = region(4); lv = region(5);
[sx, sy, sz] = sphere(16);
rr = max(1.2, 0.55 * r);
surf(cx + rr * sx, cy + rr * sy, cz + rr * sz, ...
    'FaceColor', [1.0, 0.2, 0.2], 'FaceAlpha', min(0.08 + 0.12 * lv, 0.28), ...
    'EdgeColor', 'none');
end

function drawSafetyBubble(pt, r)
[sx, sy, sz] = sphere(10);
surf(pt(1) + r * sx, pt(2) + r * sy, pt(3) + r * sz, ...
    'FaceColor', [0.2, 0.35, 1.0], 'FaceAlpha', 0.08, 'EdgeColor', 'none');
end

function plotDetailedReport(cfg, pareto)
N = numel(pareto);
routeNames = cell(N, 1);
stats = repmat(struct('nodes', 0, 'directDist', 0, 'detourRatio', 0, 'turnOver45', 0, ...
    'maxTurnDeg', 0, 'avgAlt', 0, 'climbDist', 0, 'descDist', 0), N, 1);
for i = 1:N
    routeNames{i} = sprintf('Path%d', i);
    stats(i) = computeRouteStats(pareto(i).path, pareto(i).metrics);
end

directDist = norm(double(cfg.goal - cfg.start));
fig = figure('Color', 'w', 'Position', [110, 90, 1280, 700], 'Name', 'Route Analysis Report');

metaData = {
    sprintf('%d %d %d', cfg.start(1), cfg.start(2), cfg.start(3)), ...
    sprintf('%d %d %d', cfg.goal(1), cfg.goal(2), cfg.goal(3)), ...
    sprintf('%.4f', directDist), ...
    sprintf('%d', N)
    };
metaCols = {'起点', '终点', '起终点直线距离(m)', 'Pareto路径数量'};
uitable(fig, 'Data', metaData, 'ColumnName', metaCols, 'RowName', {}, ...
    'Position', [60, 620, 1160, 52], 'FontSize', 12);

rowNames = {'总成本', '路径长度(m)', '风险成本', '能耗成本', '平滑成本', '时间成本', ...
            '节点数量', '绕行系数', '平均高度', '上升距离', '下降距离', ...
            '最大转弯角(度)', '转弯>45度次数'};
M = numel(rowNames);
vals = zeros(M, N);
for i = 1:N
    m = pareto(i).metrics;
    s = stats(i);
    vals(:, i) = [m.total; m.length; m.threat + m.safety; m.energy; m.smooth; m.time; ...
                  s.nodes; s.detourRatio; s.avgAlt; s.climbDist; s.descDist; s.maxTurnDeg; s.turnOver45];
end

preferMin = [true true true true true true true true false true true true true];
bestCol = cell(M, 1);
for r = 1:M
    if preferMin(r)
        [~, id] = min(vals(r, :));
    else
        [~, id] = max(vals(r, :));
    end
    bestCol{r} = routeNames{id};
end

tableData = cell(M, N + 2);
for r = 1:M
    tableData{r, 1} = rowNames{r};
    for c = 1:N
        tableData{r, c + 1} = sprintf('%.4f', vals(r, c));
    end
    tableData{r, N + 2} = bestCol{r};
end

colNames = cell(1, N + 2);
colNames{1} = '指标';
for i = 1:N
    colNames{i + 1} = routeNames{i};
end
colNames{N + 2} = '最优项';

uitable(fig, 'Data', tableData, 'ColumnName', colNames, 'RowName', {}, ...
    'Position', [60, 80, 1160, 520], 'FontSize', 11);
end