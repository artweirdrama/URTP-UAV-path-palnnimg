function [obs, threatRegions, startPt, goalPt] = inputSceneFromFigure(cfg)
Nx = cfg.mapSize(1); Ny = cfg.mapSize(2); Nz = cfg.mapSize(3);
zMin = max(1, cfg.flyRange(1));
zMax = min(Nz, cfg.flyRange(2));

fig = figure('Color', 'w', 'Name', 'Scene Input', 'NumberTitle', 'off', ...
    'Position', [180, 130, 880, 650]);
ax = axes('Parent', fig);
hold(ax, 'on');
grid(ax, 'on');
axis(ax, [1 Nx 1 Ny]);
axis(ax, 'equal');
xlabel(ax, 'X');
ylabel(ax, 'Y');
title(ax, sprintf(['Step1 障碍输入: 左键添加, 右键/回车结束\n', ...
    '后续输入威胁区、起点和终点, 高度范围 z=[%d, %d]'], zMin, zMax));

set(ax, 'XTick', 1:5:Nx, 'YTick', 1:5:Ny);

obs = zeros(0, 3);
obsMap = false(Nx, Ny);
threatRegions = zeros(0, 5);

while true
    [x, y, button] = ginput(1);
    if isempty(button) || button == 3 || button == 13
        break;
    end
    if button ~= 1
        continue;
    end

    xi = min(max(round(x), 1), Nx);
    yi = min(max(round(y), 1), Ny);

    if obsMap(xi, yi)
        continue;
    end

    h = askIntDialog(sprintf('Obstacle (%d,%d) height [0..%d]:', xi, yi, Nz), ...
        max(round((zMin + zMax) / 2), 1), 0, Nz);
    obs = [obs; xi, yi, h]; %#ok<AGROW>
    obsMap(xi, yi) = true;

    scatter(ax, xi, yi, 80, 's', 'filled', 'MarkerFaceColor', [0.2, 0.2, 0.2]);
    text(ax, xi + 0.25, yi + 0.25, sprintf('h=%d', h), 'Color', [0.15, 0.15, 0.15], 'FontSize', 8);
end

title(ax, 'Step2 威胁区输入: 左键添加威胁中心, 右键/回车结束');
while true
    [x, y, button] = ginput(1);
    if isempty(button) || button == 3 || button == 13
        break;
    end
    if button ~= 1
        continue;
    end

    xi = min(max(round(x), 1), Nx);
    yi = min(max(round(y), 1), Ny);
    cz = askIntDialog(sprintf('Threat (%d,%d) center z [%d..%d]:', xi, yi, zMin, zMax), ...
        round((zMin + zMax) / 2), zMin, zMax);
    rr = askRealDialog('Threat radius (grid, recommend 3~10):', 6.0, 1.0, max([Nx, Ny]));
    lv = askRealDialog('Threat level (recommend 0.5~2.0):', 1.0, 0.1, 5.0);
    threatRegions = [threatRegions; xi, yi, cz, rr, lv]; %#ok<AGROW>

    th = linspace(0, 2*pi, 80);
    plot(ax, xi + rr * cos(th), yi + rr * sin(th), '-', 'Color', [0.95, 0.2, 0.2], 'LineWidth', 1.2);
    scatter(ax, xi, yi, 40, 'filled', 'MarkerFaceColor', [0.9, 0.1, 0.1]);
    text(ax, xi + 0.25, yi + 0.25, sprintf('R=%.1f,L=%.1f', rr, lv), ...
        'Color', [0.68, 0.05, 0.05], 'FontSize', 8);
end

title(ax, sprintf('Step3 点击起点并输入 z [%d, %d]', zMin, zMax));
[sx, sy] = ginput(1);
sxi = min(max(round(sx), 1), Nx);
syi = min(max(round(sy), 1), Ny);
sz = askIntDialog(sprintf('Start z for (%d,%d) [%d..%d]:', sxi, syi, zMin, zMax), zMin, zMin, zMax);
startPt = [sxi, syi, sz];

title(ax, sprintf('Step4 点击终点并输入 z [%d, %d]', zMin, zMax));
[gx, gy] = ginput(1);
gxi = min(max(round(gx), 1), Nx);
gyi = min(max(round(gy), 1), Ny);
gz = askIntDialog(sprintf('Goal z for (%d,%d) [%d..%d]:', gxi, gyi, zMin, zMax), zMax, zMin, zMax);
goalPt = [gxi, gyi, gz];

for k = 1:size(obs, 1)
    ox = obs(k, 1); oy = obs(k, 2); oh = obs(k, 3);
    if ox == startPt(1) && oy == startPt(2) && startPt(3) <= oh
        error('Start point is inside an obstacle column. Please rerun and choose another start point or lower obstacle height.');
    end
    if ox == goalPt(1) && oy == goalPt(2) && goalPt(3) <= oh
        error('Goal point is inside an obstacle column. Please rerun and choose another goal point or lower obstacle height.');
    end
end

scatter(ax, startPt(1), startPt(2), 120, 'o', 'filled', 'MarkerFaceColor', [0.1, 0.7, 0.1]);
text(ax, startPt(1) + 0.25, startPt(2) + 0.25, sprintf('S z=%d', startPt(3)), ...
    'Color', [0.0, 0.5, 0.0], 'FontWeight', 'bold');

scatter(ax, goalPt(1), goalPt(2), 140, 'p', 'filled', 'MarkerFaceColor', [0.88, 0.1, 0.1]);
text(ax, goalPt(1) + 0.25, goalPt(2) + 0.25, sprintf('G z=%d', goalPt(3)), ...
    'Color', [0.68, 0.05, 0.05], 'FontWeight', 'bold');

title(ax, 'Input complete. Close this figure to start optimization and planning.');
uiwait(msgbox('Scene input complete. Close the input figure to continue.', 'Done', 'modal'));
if isvalid(fig)
    close(fig);
end
end

function v = askIntDialog(prompt, defaultVal, lb, ub)
answ = inputdlg(prompt, 'Input', [1 55], {num2str(defaultVal)});
if isempty(answ)
    v = defaultVal;
else
    vv = str2double(answ{1});
    if isnan(vv)
        vv = defaultVal;
    end
    v = round(vv);
end
v = min(max(v, lb), ub);
end

function v = askRealDialog(prompt, defaultVal, lb, ub)
answ = inputdlg(prompt, 'Input', [1 55], {num2str(defaultVal)});
if isempty(answ)
    v = defaultVal;
else
    vv = str2double(answ{1});
    if isnan(vv)
        vv = defaultVal;
    end
    v = vv;
end
v = min(max(v, lb), ub);
end