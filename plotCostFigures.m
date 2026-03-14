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