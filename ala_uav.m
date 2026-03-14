% ala_uav.m
clear; close all; clc;

% 环境设置
bounds = [0 100; 0 100; 0 50]; % x,y,z 范围
start = [5, 5, 5];
goal  = [95, 95, 40];

% 定义球形障碍物（每行：x y z r）
obstacles = [30 40 15 12;
             60 60 20 10;
             45 80 30 8];

% ALA 参数
numWaypoints = 6; % 中间可变路径点数（不含起点终点）
popSize = 40;
maxIter = 200;

% 搜索变量边界（只针对中间路径点）
lb = repmat([bounds(1,1), bounds(2,1), bounds(3,1)], 1, numWaypoints);
ub = repmat([bounds(1,2), bounds(2,2), bounds(3,2)], 1, numWaypoints);

% 运行 ALA
options.popSize = popSize;
options.maxIter = maxIter;
options.numWaypoints = numWaypoints;
options.lb = lb;
options.ub = ub;
options.start = start;
options.goal = goal;
options.obstacles = obstacles;
options.bounds = bounds;

[bestSol, bestFit, history] = ala_algorithm(options);

% 解码最佳解为完整路径
path = [start; reshape(bestSol,3,[])'; goal];

% 绘图
figure('Name','ALA UAV 3D Path Planning'); hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(35,30);

% 绘制障碍物
helpers_plotObstacles(obstacles);

% 绘制起终点与路径
plot3(start(1),start(2),start(3),'go','MarkerSize',10,'MarkerFaceColor','g');
plot3(goal(1),goal(2),goal(3),'ro','MarkerSize',10,'MarkerFaceColor','r');
plot3(path(:,1),path(:,2),path(:,3),'-b','LineWidth',2);
scatter3(path(:,1),path(:,2),path(:,3),40,'b','filled');

title(sprintf('Best cost = %.4f', bestFit));

% 可视化演化曲线
figure('Name','Fitness History'); plot(history,'-k','LineWidth',1.4); grid on;
xlabel('Iteration'); ylabel('Best Fitness'); title('ALA 收敛曲线');

fprintf('最优适应度: %.6f\n', bestFit);

% End of file
