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