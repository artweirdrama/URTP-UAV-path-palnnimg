
clc; clear; close all;

MaxIt = 200;   
nPop  = 100;   
numExp = 30;   

model = CreateModel3();

all_feasible_costs = [];
all_times = zeros(numExp,1);  

for expIdx = 1:numExp
    fprintf('\n===== 第 %d/%d 次实验 =====\n', expIdx, numExp);
    % 重置随机数生成器以确保每次运行独立
    rng('shuffle');
    exp_start_time = tic;

    AHS_GGWO;

    exp_time = toc(exp_start_time);
    all_times(expIdx) = exp_time;   % 保存本次耗时
    
    fprintf('本次实验耗时: %.4f 秒 (%.2f 分钟)\n', exp_time, exp_time/60);
    feasible_costs_exp = [];  
    for i = 1:numel(wolves)
        cost_vec = wolves(i).Cost;

        % 判断是否已经是 1×4
        if ~isequal(size(cost_vec), [1 4])
            cost_vec = cost_vec(:)';
        end
        if all(isfinite(cost_vec)) && ~any(isnan(cost_vec))
            feasible_costs_exp = [feasible_costs_exp; cost_vec];
        end
    end
    if(size(feasible_costs_exp,1)==0)
        % 判断是否已经是 1×4
        if ~isequal(size(GlobalBest.Cost), [1 4])
            GlobalBest.Cost = GlobalBest.Cost(:)';
        end
        if all(isfinite(GlobalBest.Cost)) && ~any(isnan(GlobalBest.Cost))
                feasible_costs_exp = [feasible_costs_exp; GlobalBest.Cost];
        end
    end
    fprintf('本次可行解数量: %d\n', size(feasible_costs_exp,1));

    % 累加到总集合
    all_feasible_costs = [all_feasible_costs; feasible_costs_exp];
end

% ===============================
% 保存本算法所有实验的结果
% ===============================
save('Results_AHS-GGWO_3.mat', 'all_feasible_costs', 'all_times');
fprintf('\n✅ 已保存结果到文件 Results_AHS-GGWO_3.mat\n');

%% ===============================
% 30次实验的综合统计
% ================================

% 1. 耗时统计
mean_time = mean(all_times);
std_time  = std(all_times);

fprintf('\n===== 30次实验运行时间统计 =====\n');
fprintf('平均耗时: %.4f 秒 (%.2f 分钟)\n', mean_time, mean_time/60);
fprintf('耗时标准差: %.4f 秒\n', std_time);

% 2. 可行解统计
total_feasible = size(all_feasible_costs,1);
fprintf('\n===== 30次实验可行解统计 =====\n');
fprintf('总可行解数量: %d\n', total_feasible);

if total_feasible > 0
    % 每个成本分量的统计
    min_costs  = min(all_feasible_costs, [], 1);
    max_costs  = max(all_feasible_costs, [], 1);
    mean_costs = mean(all_feasible_costs, 1);
    std_costs  = std(all_feasible_costs, 0, 1);

    % 打印最终统计结果
    fprintf('\n成本分量 | %-12s | %-12s | %-12s | %-12s\n', ...
        '最小值', '最大值', '平均值', '标准差');
    fprintf('路径长度 | %-12.4f | %-12.4f | %-12.4f | %-12.4f\n', ...
            min_costs(1), max_costs(1), mean_costs(1), std_costs(1));
    fprintf('威胁规避 | %-12.4f | %-12.4f | %-12.4f | %-12.4f\n', ...
            min_costs(2), max_costs(2), mean_costs(2), std_costs(2));
    fprintf('高度成本 | %-12.4f | %-12.4f | %-12.4f | %-12.4f\n', ...
            min_costs(3), max_costs(3), mean_costs(3), std_costs(3));
    fprintf('路径平滑 | %-12.4f | %-12.4f | %-12.4f | %-12.4f\n', ...
            min_costs(4), max_costs(4), mean_costs(4), std_costs(4));
else
    fprintf('警告: 30次实验中没有找到任何可行解！\n');
end