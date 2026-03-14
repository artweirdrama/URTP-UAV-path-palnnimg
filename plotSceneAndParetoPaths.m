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