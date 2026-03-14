%% 无人机三维栅格环境建模 (20×20×15) - bar3 版本
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

%% 步骤2：用 bar3 绘制三维柱状图（城市建筑）
figure('Color','w');
b = bar3(building_height, 0.8); % 0.8 是柱子宽度，避免重叠
colorbar; % 添加颜色栏

%% 步骤3：设置柱子颜色（按高度插值，和示例一致）
for k = 1:length(b)
    zdata = b(k).ZData;
    b(k).CData = zdata;
    b(k).FaceColor = 'interp';
end

%% 步骤4：美化图形
title('城市三维环境模型（20×20×15 栅格）');
xlabel('X 栅格');
ylabel('Y 栅格');
zlabel('Z 栅格（高度）');
grid on;
axis equal;
view(35, 30);
camlight;
lighting gouraud;