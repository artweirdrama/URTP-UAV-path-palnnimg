%% 无人机三维栅格环境建模 (20×20×15) - 圆柱体版本
clear; clc; close all;

%% 步骤1：生成建筑高度数据（20×20 平面）
X_SIZE = 20;
Y_SIZE = 20;
Z_SIZE = 15;

% 生成随机建筑高度（0表示空地/街道，3-15表示建筑高度）
rng(42); % 固定随机种子，结果可复现
building_density = 0.6; % 60%的栅格有建筑
building_height = zeros(X_SIZE, Y_SIZE);

for i = 1:X_SIZE
    for j = 1:Y_SIZE
        if rand() < building_density
            building_height(i,j) = randi([3, 15]); % 随机高度
        else
            building_height(i,j) = 0; % 街道/空地
        end
    end
end

%% 步骤2：绘制三维圆柱体（替换原bar3）
figure('Color','w');
hold on; % 保持绘图窗口，逐个绘制圆柱体

% 定义圆柱体底面分辨率（越多越接近圆柱）
theta = linspace(0, 2*pi, 20); 
% 圆柱体半径（对应原bar3的0.8宽度，保持视觉比例）
radius = 0.4; 

% 遍历所有栅格，绘制对应高度的圆柱体
for i = 1:X_SIZE
    for j = 1:Y_SIZE
        h = building_height(i,j);
        if h > 0 % 只绘制有建筑的位置
            % 生成圆柱体坐标
            [x_cyl, y_cyl, z_cyl] = cylinder(radius);
            % 缩放并平移圆柱体到对应栅格位置
            x_cyl = x_cyl + i; % X轴偏移，中心在(i, j)
            y_cyl = y_cyl + j; % Y轴偏移
            z_cyl = z_cyl * h; % Z轴按高度缩放
            
            % 绘制单个圆柱体并设置颜色
            s = surf(x_cyl, y_cyl, z_cyl, 'CData', z_cyl, 'FaceColor', 'interp');
            s.EdgeColor = 'none'; % 隐藏边线，让效果更平滑
        end
    end
end

%% 步骤3：美化图形（完全保留原有设置）
colorbar; % 添加颜色栏
title('城市三维环境模型（20×20×15 栅格）');
xlabel('X 栅格');
ylabel('Y 栅格');
zlabel('Z 栅格（高度）');
grid on;
axis equal;
view(35, 30);
camlight;
lighting gouraud;
hold off;

% 设置坐标轴范围（确保和原版本一致）
xlim([0, X_SIZE+1]);
ylim([0, Y_SIZE+1]);
zlim([0, Z_SIZE]);