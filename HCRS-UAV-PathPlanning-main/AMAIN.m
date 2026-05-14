clc; 
clear; 
close all; 

% ============================================================
% 1. 设置随机种子 (Reproducibility Setting)
% ============================================================
% To ensure the uniqueness and reproducibility of the demonstration results, the random seed is fixed here.
rng(1);  % 种子设为 1 

MaxIt = 200;          % 最大迭代次数
nPop = 100;           % 种群规模
model = CreateModel3(); 

% ================= 开始计时 =================
t_start = tic; 

AHS_GGWO;           

elapsed_time = toc(t_start); 
% ================= 结束计时 =================

fprintf('------------------------------------------------------\n');
fprintf('当前场景 (Scenario 3) 单次运行耗时: %.4f 秒\n', elapsed_time);
fprintf('------------------------------------------------------\n');

% 绘图 (不计入时间)
sols = {BestPositionGWO_GA(1),BestPositionGWO_GA(2),BestPositionGWO_GA(3),BestPositionGWO_GA(4),BestPositionGWO_GA(5) };
colors = {'k','r','g','y','b'};
APlotSolution(sols, model, 0.99, colors);