function [bestSol, bestFit, history] = ala(options)
% options: popSize, maxIter, numWaypoints, lb, ub, start, goal, obstacles

% 参数读取
popSize = options.popSize;
maxIter = options.maxIter;
D = options.numWaypoints * 3; % 3D coords per waypoint
lb = options.lb;
ub = options.ub;
start = options.start;
goal = options.goal;
obstacles = options.obstacles;

% 初始化种群
pop = repmat(lb, popSize, 1) + rand(popSize, D) .* (repmat(ub-lb, popSize, 1));
fitness = zeros(popSize,1);
for i=1:popSize
    fitness(i) = fitness_path(pop(i,:), start, goal, obstacles);
end

[bestFit, idx] = min(fitness);
bestSol = pop(idx,:);
worstFit = max(fitness);

history = zeros(maxIter,1);

% ALA 主循环（带有简化的行为策略）
for t = 1:maxIter
    % 动态步长因子
    step = 0.5 * (1 - t/maxIter);
    for i=1:popSize
        r1 = rand(); r2 = rand(); r3 = rand();
        % 向最优个体靠拢
        Xnew = pop(i,:) + r1 * (bestSol - pop(i,:)) + step * randn(1,D);
        % 随机逃逸一部分（避免局部最优）
        if rand < 0.15
            Xnew = pop(i,:) + 0.3*(rand(1,D)-0.5).*(ub-lb);
        end
        % 边界处理
        Xnew = max(Xnew, lb);
        Xnew = min(Xnew, ub);
        fnew = fitness_path(Xnew, start, goal, obstacles);
        % 接受较好解
        if fnew < fitness(i)
            pop(i,:) = Xnew;
            fitness(i) = fnew;
        end
        % 更新全局最优
        if fnew < bestFit
            bestFit = fnew;
            bestSol = Xnew;
        end
    end
    % 记录历史
    history(t) = bestFit;
end

end
