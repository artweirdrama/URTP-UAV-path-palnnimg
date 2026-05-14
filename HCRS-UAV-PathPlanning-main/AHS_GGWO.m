
num_drones = model.num_drones;  
n = model.n;                    

VarSize = [1 n]; 
VarMin.x = model.xmin;  
VarMax.x = model.xmax;    
VarMin.y = model.ymin;       
VarMax.y = model.ymax;       
VarMin.z = model.zmin;       
VarMax.z = model.zmax;       
M = 3;     

CostFunction = @(sol) AMyCost(sol, model); 

nRep = 30;         
a = 2;             
rep_history = cell(MaxIt,1);   


empty_wolf.Position = struct(); 
empty_wolf.CartPosition = struct(); 
empty_wolf.Cost = [];         
empty_wolf.Best.Position = []; 
empty_wolf.Best.CartPosition = []; 
empty_wolf.Best.Cost = [];     
empty_wolf.IsDominated = [];   

GlobalBest = empty_wolf;

disp("初始化多无人机种群...");
drone_populations = repmat(struct('Position', [], 'CartPosition', [], 'Cost', []), num_drones, nPop);
sorted_indices = zeros(num_drones, nPop);


for k = 1:num_drones
    current_start = model.starts( : ,k)';
    current_end = model.ends( : ,k)';
    dist = norm(current_end - current_start);
    VarMax.r = 2 * dist / n;  
    VarMin.r = 0;  
    
    AngleRange = pi/4;       
    VarMin.psi = -AngleRange;       
    VarMax.psi = AngleRange;       
    
    dirVector = current_end - current_start;
    phi0 = atan2(dirVector(2), dirVector(1)); 
    VarMin.phi = phi0 - AngleRange;       
    VarMax.phi = phi0 + AngleRange; 

    valid_solution_found = false;
    fprintf('无人机 %d: 生成种群...\n', k);
    while ~valid_solution_found
        for i = 1:nPop

            drone_populations(k, i).Position = PGCreateRandomSolution(VarSize, VarMin, VarMax,model,current_start);
            
            drone_populations(k, i).CartPosition = PGSphericalToCart(drone_populations(k, i).Position, model,k);
            
            drone_populations(k, i).Cost = SingleDroneCost(drone_populations(k, i).CartPosition, model, VarMin,current_start,current_end);

            drone_populations(k, i).Best.Position = drone_populations(k, i).Position;
            drone_populations(k, i).Best.CartPosition = drone_populations(k, i).CartPosition;
            drone_populations(k, i).Best.Cost = drone_populations(k, i).Cost;

            total_costs = arrayfun(@(x) sum(x.Cost), drone_populations(k, i));
            if any(total_costs < inf)
                valid_solution_found = true;
            end
        end 
    end
   
    total_costs = arrayfun(@(x) sum(x.Cost), drone_populations(k, :));
    
    [~, sorted_indices(k, :)] = sort(total_costs);
end

wolves = repmat(empty_wolf, nPop, 1);

for i = 1:nPop
    drone_sols = struct();
    
    for k = 1:num_drones
        idx = sorted_indices(k, i);  
        drone_sols(k).Position = drone_populations(k, idx).Position;
        drone_sols(k).CartPosition = drone_populations(k, idx).CartPosition;
    end
    
    wolves(i).Position = copyDroneSolutions(drone_sols);
    

    wolves(i).Cost = CostFunction([drone_sols.CartPosition]);
    

    wolves(i).Best.Position = wolves(i).Position;
    wolves(i).Best.CartPosition = [drone_sols.CartPosition];
    wolves(i).Best.Cost = wolves(i).Cost;
    

    if i == 1 || PGDominates(wolves(i).Cost, GlobalBest.Cost)
        GlobalBest = wolves(i).Best;
    end
end
fprintf('初始化完成');
wolves = PGDetermineDomination(wolves);
rep = wolves(~[wolves.IsDominated]);

if numel(rep) >= 1
    sum_costs = arrayfun(@(w) sum(w.Cost), rep);
    [~, sorted_idx] = sort(sum_costs);
    GlobalBest = rep(sorted_idx(1));
else
    sum_costs = arrayfun(@(w) sum(w.Cost), wolves);
    [~, sorted_all_idx] = sort(sum_costs);
    GlobalBest = wolves(sorted_all_idx(1));
end

BestCost = zeros(MaxIt, 4); 
BestSumCost = zeros(MaxIt, 1);
useGWO = false;      
stagnationCount = 0; 
stagnationThreshold = 6; 
minImprovement = 0.01; 
GlobalBest_drone = cell(num_drones, 1); 
for k = 1:num_drones
    GlobalBest_drone{k} = cell(1, M); 
end
prevBestCost = inf; 
for it = 1:MaxIt
    a = 2 - it*(2/MaxIt); 
    % === AHS-GGWO: Adaptive Operator Switching Logic === 
    if it > 1
        improvement = (prevBestCost - BestSumCost(it-1)) / prevBestCost;
        
        if improvement < minImprovement
            stagnationCount = stagnationCount + 1;
        else
            stagnationCount = 0; 
        end
        if stagnationCount >= stagnationThreshold
            useGWO = ~useGWO; 
            stagnationCount = 0;  
        end
    end
    
    if it > 1
        prevBestCost = BestSumCost(it-1);
    else
        prevBestCost = sum(GlobalBest.Cost);
    end
   
    
    for k = 1:num_drones  
        current_start = model.starts( : ,k)';
        current_end = model.ends( : ,k)';
       
        dist = norm(current_end - current_start);
        VarMax.r = 2 * dist / n;
        VarMin.r = 0;          
        AngleRange = pi/4;       
        VarMin.psi = -AngleRange;       
        VarMax.psi = AngleRange;       
        dirVector = current_end - current_start;
        phi0 = atan2(dirVector(2), dirVector(1)); 
        VarMin.phi = phi0 - AngleRange;       
        VarMax.phi = phi0 + AngleRange; 
               
        % Extract elite non-dominated solutions from the evolutionary external archive 
        % to act as leaders (alpha, beta, delta) and guide the underlying GWO search.
        multi_new_leaders = selectLeadersFromRep(rep, M);
        if numel(multi_new_leaders) < 3
            multi_new_leaders(end+1:3) = multi_new_leaders(end);
        end
        new_leaders = cell(1,M);
        for m = 1:M
            if ~isempty(multi_new_leaders{m})
                curPos  = multi_new_leaders{m}.Position(k).Position;       
                curCart = multi_new_leaders{m}.Position(k).CartPosition;   
        
                curCost = SingleDroneCost(curCart, model, VarMin, current_start, current_end);
        
                new_leaders{m} = struct( ...
                    'Position', curPos, ...
                    'CartPosition', curCart, ...
                    'Cost', curCost ...
                );
            else
                new_leaders{m} = [];
            end
        end
        
        
        current_costs = arrayfun(@(x) sum(x.Cost), drone_populations(k, :));
        [~, sorted_idx] = sort(current_costs);
        local_alpha = drone_populations(k, sorted_idx(1));
        local_beta = drone_populations(k, sorted_idx(2));
        local_delta = drone_populations(k, sorted_idx(3));
        local{1} = local_alpha;
        local{2} = local_beta;
        local{3} = local_delta;
        
        for m = 1:3
            if it == 1
                if ~isempty(new_leaders{m}) && isstruct(new_leaders{m}) ...
                && isfield(new_leaders{m},'Cost') ...
                && sum(new_leaders{m}.Cost) < sum(local{m}.Cost)
                    GlobalBest_drone{k}{m} = new_leaders{m};
                
                else
                    GlobalBest_drone{k}{m} = local{m};
                end
            else
                hist_leader = GlobalBest_drone{k}{m};
                new_leader = new_leaders{m};
                local_leader = local{m};  
                
                candidates = {hist_leader, new_leader, local_leader};
                cost_sums = zeros(1, 3);  
                
                for i = 1:3
                    if ~isempty(candidates{i})
                        cost_sums(i) = sum(candidates{i}.Cost);  
                    else
                        cost_sums(i) = inf; 
                    end
                end
                
                [~, best_idx] = min(cost_sums);
                best_leader = candidates{best_idx};
                
                GlobalBest_drone{k}{m} = best_leader;
            end
        end
        
        valid_leaders = ~cellfun(@isempty, GlobalBest_drone{k});
        leader_positions = cell(size(valid_leaders));
        for m = 1:M
            if valid_leaders(m)
                leader_positions{m} = GlobalBest_drone{k}{m}.Position;
            end
        end
        
        for i = 1:nPop  
            if useGWO
                tmp = GlobalBest_drone{k}(valid_leaders);
                
                template = struct('Position',[],'CartPosition',[],'Cost',[]);
                
                for ii = 1:numel(tmp)
                    fn_template = fieldnames(template);
                    fn_current  = fieldnames(tmp{ii});
                    missing_fields = setdiff(fn_template, fn_current);
                    for f = 1:numel(missing_fields)
                        tmp{ii}.(missing_fields{f}) = [];
                    end
                    extra_fields = setdiff(fn_current, fn_template);
                    if ~isempty(extra_fields)
                        tmp{ii} = rmfield(tmp{ii}, extra_fields);
                    end
                    tmp{ii} = orderfields(tmp{ii}, template);
                end
                
                valid_leader_objects = [tmp{:}];
                
                candidate = GWO_Operator(...
                    drone_populations(k, i).Position, ...
                    valid_leader_objects, ...  
                    a);
            else
                candidate = GA_Operator(...
                    drone_populations(k, i).Position, ...
                    [drone_populations(k, :).Position], ... 
                    VarMin, ...
                    VarMax);
            end
            for comp = {'r','psi','phi'}
                field = comp{1};
                candidate.(field) = max(candidate.(field), VarMin.(field));
                candidate.(field) = min(candidate.(field), VarMax.(field));
            end
           
            candidate_cart = PGSphericalToCart(candidate, model,k);
            candidate_cost = SingleDroneCost(candidate_cart, model, VarMin, current_start, current_end);
            current_drone = drone_populations(k, i);
            current_drone.Position = candidate;
            current_drone.CartPosition = candidate_cart;
            current_drone.Cost = candidate_cost;
            if PGDominates(current_drone,current_drone.Best)
                current_drone.Best.Position = current_drone.Position;
                current_drone.Best.CartPosition = current_drone.CartPosition;
                current_drone.Best.Cost = current_drone.Cost;
            end
            drone_populations(k, i) = current_drone;
        end
    end
    drone_fronts = cell(1, num_drones);
    for k = 1:num_drones
        current_pop = drone_populations(k, :);
        [sorted_pop, FrontNo, CrowdDis] = EnvironmentalSelection(current_pop, nPop);
        unique_fronts = unique(FrontNo);
        fronts = cell(1, max(unique_fronts));
        for f = unique_fronts
            idxs = find(FrontNo == f);
            [~, order_in_front] = sort(CrowdDis(idxs), 'descend');
            fronts{f} = sorted_pop(idxs(order_in_front));
        end
        drone_fronts{k}.Front = fronts;
    end
    
    for i = 1:nPop
        drone_sols = struct();
        cur_layer = 1;
        
        for k = 1:num_drones
            fronts = drone_fronts{k}.Front;
            if numel(fronts) >= cur_layer && numel(fronts{cur_layer}) >= i
                selected = fronts{cur_layer}(i);
            else
                next_layer = min(cur_layer + 1, numel(fronts));
                idx = randi(numel(fronts{next_layer}));
                selected = fronts{next_layer}(idx);
            end
            drone_sols(k).Position     = selected.Position;
            drone_sols(k).CartPosition = selected.CartPosition;
        end
        
        all_cart = [drone_sols.CartPosition];
        total_cost = CostFunction(all_cart);
        
        % === Cooperative Recombination Mechanism ===
        % If spatial-temporal collision is detected (cost is inf), trigger dynamic correction
        % by probabilistically resampling path segments from different Pareto fronts.
        if any(isinf(total_cost))
            for k = 1:num_drones
                fronts = drone_fronts{k}.Front;
                if rand() < 0.7
                    high_layers = 1:min(2, numel(fronts));
                    sel_layer = high_layers(randi(numel(high_layers)));
                else
                    low_layers = 3:numel(fronts);
                    if isempty(low_layers)
                        low_layers = numel(fronts);
                    end
                    sel_layer = low_layers(randi(numel(low_layers)));
                end
                
                chosen_layer = fronts{sel_layer};
                picked = chosen_layer(randi(numel(chosen_layer)));
                
                drone_sols(k).Position     = picked.Position;
                drone_sols(k).CartPosition = picked.CartPosition;
            end
            
            all_cart = [drone_sols.CartPosition];
            total_cost = CostFunction(all_cart);
        end
        
        wolves(i).Position           = copyDroneSolutions(drone_sols);
        wolves(i).Cost               = total_cost;
        wolves(i).Best.Position      = wolves(i).Position;
        wolves(i).Best.CartPosition  = [drone_sols.CartPosition];
        wolves(i).Best.Cost          = wolves(i).Cost;
    end
    combined = [rep ; wolves];  
    combined = PGDetermineDomination(combined);
    combined = combined(~[combined.IsDominated]);
    % === Evolutionary External Archive Update Rules ===
    rep = UpdateRepWithGA(combined, model, VarMin, VarMax, nRep, CostFunction);
    rep = PGDetermineDomination(rep);
    rep = rep(~[rep.IsDominated]);
    % Trim the combined archive based on evaluation metrics to maintain diversity
    if numel(rep) > nRep
        sum_costs = arrayfun(@(w) sum(w.Cost), rep);  
        [~, sorted_idx] = sort(sum_costs);  
        rep = rep(sorted_idx); 
        rep = rep(1:nRep);  
    end

    if numel(rep) >= 1
        sum_costs = arrayfun(@(w) sum(w.Cost), rep);
        [~, idx] = sort(sum_costs);
        currentGlobalBest = rep(idx(1));
    else
        sum_costs = arrayfun(@(w) sum(w.Cost), wolves);
        [~, idx] = sort(sum_costs);
        currentGlobalBest = wolves(idx(1));
    end

    if sum(currentGlobalBest.Cost) < sum(GlobalBest.Cost)
        GlobalBest = currentGlobalBest;
    end
  

    BestCost(it, :) = GlobalBest.Cost'; 
    BestSumCost(it) = sum(GlobalBest.Cost);
    if it == MaxIt
        disp(['最终结果: 迭代 ', num2str(it), '/', num2str(MaxIt)...
                ' | 成本分量: [', num2str(GlobalBest.Cost(1)), ', ', ...
                                    num2str(GlobalBest.Cost(2)), ', ', ...
                                    num2str(GlobalBest.Cost(3)), ', ', ...
                                    num2str(GlobalBest.Cost(4)), ']',]);
    end
    rep_history{it} = rep;   
end

BestPositionGWO_GA = [GlobalBest.Position.CartPosition];
BestCostGWO_GA = GlobalBest.Cost;
repNumber = numel(rep);
sols = cell(1, num_drones);
for k = 1:num_drones
    sols{k} = cell(1, nPop);
    for i = 1:nPop
        pos = drone_populations(k,i).CartPosition;
        sols{k}{i} = struct('x', pos.x(:), ...
                            'y', pos.y(:), ...
                            'z', pos.z(:));
    end
end

function leaders = selectLeadersFromRep(rep, M)
    if isempty(rep)
        leaders = cell(1,M);
        for i = 1:M
            leaders{i} = []; 
        end
        return;
    end
    
    nPop = numel(rep);
    [~, FrontNo, CrowdDis] = EnvironmentalSelection(rep, nPop);
    [~, sortIdx] = sortrows([FrontNo', -CrowdDis'], [1, 2]);
    sortedPopulation = rep(sortIdx);

    numLeaders = min(M, nPop);
    leaders = cell(1,M);
    for i = 1:numLeaders
        leaders{i} = sortedPopulation(i);
        if size(leaders{i}.Cost, 1) > 1
            leaders{i}.Cost = leaders{i}.Cost';
        end
    end
    
    if numLeaders < M
        for j = numLeaders+1:M
            leaders{j} = leaders{numLeaders}; 
        end
    end
end

function candidate = GWO_Operator(current, leaders, a)
    candidate = struct();
    comp_types = {'r', 'psi', 'phi'};
    
    M = min(numel(leaders), 3);
    if M < 3
        leaders = [leaders, repmat(leaders(end), 1, 3 - M)];
    end
    
    for c = 1:numel(comp_types)
        comp = comp_types{c};
        current_comp = current.(comp);  
        n_points = numel(current_comp); 
        
        X1 = zeros(1, n_points);
        X2 = zeros(1, n_points);
        X3 = zeros(1, n_points);
        
        for m = 1:3
            leader_comp = leaders(m).Position.(comp);
            if iscolumn(leader_comp)
                leader_comp = leader_comp';
            end
            
            A = 2 * a * rand(1, n_points) - a; 
            C = 2 * rand(1, n_points);          
            D = abs(C .* leader_comp - current_comp); 
            X_leader = leader_comp - A .* D;   
            if m == 1
                X1 = X_leader; 
            elseif m == 2
                X2 = X_leader; 
            else
                X3 = X_leader; 
            end
        end
        candidate.(comp) = (X1 + X2 + X3) / 3;
    end
end

function [pop, FrontNo, CrowdDis] = EnvironmentalSelection(pop, nPop)
    costs = [pop.Cost]';   
    [FrontNo, MaxFNo] = NDSort(costs, nPop);
    CrowdDis = CrowdingDistance(costs, FrontNo);
    next = false(1, numel(pop));
    for f = 1:MaxFNo
        current_front = find(FrontNo == f);
        if sum(next) + numel(current_front) <= nPop
            next(current_front) = true;
        else
            [~, idx] = sort(CrowdDis(current_front), 'descend');
            num_select = nPop - sum(next);
            next(current_front(idx(1:num_select))) = true;
            break;
        end
    end
    
    pop = pop(next);
    FrontNo = FrontNo(next);
    CrowdDis = CrowdDis(next);
end

function new_solution = GA_Operator(parent, population, VarMin, VarMax)
    idx = randi(numel(population));
    parent2 = population(idx);
    
    child = struct();
    for comp = {'r','psi','phi'}
        field = comp{1};
        child.(field) = sbx(parent.(field), parent2.(field), VarMin.(field), VarMax.(field));
    end
   
    new_solution = struct();
    for comp = {'r','psi','phi'}
        field = comp{1};
        new_solution.(field) = mutate(child.(field), VarMin.(field), VarMax.(field));
    end
end

function child = sbx(parent1, parent2, lb, ub)
    proC = 0.5; 
    disC = 20;    
    
    n = length(parent1); 
    if isscalar(lb)
        lb = repmat(lb, 1, n);
    end
    if isscalar(ub)
        ub = repmat(ub, 1, n);
    end
    
    child1 = parent1;
    child2 = parent2;
    
    for i = 1:n
        if rand() <= proC
            if abs(parent1(i) - parent2(i)) > 1e-10
                if parent1(i) < parent2(i)
                    y1 = parent1(i);
                    y2 = parent2(i);
                else
                    y1 = parent2(i);
                    y2 = parent1(i);
                end
                
                beta = 1.0 + (2.0 * (y1 - lb(i)) / (y2 - y1));
                alpha = 2.0 - beta^(-(disC + 1.0));
                u = rand();
                if u <= (1.0/alpha)
                    beta_q = (u * alpha)^(1.0/(disC + 1.0));
                else
                    beta_q = (1.0/(2.0 - u * alpha))^(1.0/(disC + 1.0));
                end
                c1 = 0.5 * ((y1 + y2) - beta_q * (y2 - y1));
                c2 = 0.5 * ((y1 + y2) + beta_q * (y2 - y1));
                c1 = max(lb(i), min(c1, ub(i)));
                c2 = max(lb(i), min(c2, ub(i)));
                if rand() < 0.5
                    child1(i) = c1;
                    child2(i) = c2;
                else
                    child1(i) = c2;
                    child2(i) = c1;
                end
            end
        end
    end
    if rand() < 0.5
        child = child1;
    else
        child = child2;
    end
end

function mutated = mutate(x, lb, ub)
    nmu = 20;     
    pm = 0.05;        
    n_points = numel(x); 
    if isscalar(lb)
        lb = repmat(lb, 1, n_points);
    end
    if isscalar(ub)
        ub = repmat(ub, 1, n_points);
    end
    
    scale = 0.1 * (ub - lb);
    
    if iscolumn(scale)
        scale = scale';
    end
    
    mutated = x; 
    mutate_points = randperm(n_points, ceil(n_points*pm));
    
    for i = mutate_points
        u = rand();
        if u <= 0.5
            delta = (2*u)^(1/(nmu+1)) - 1;
        else
            delta = 1 - (2*(1-u))^(1/(nmu+1));
        end
        
        mutated(i) = x(i) + delta * scale(i);
        
        mutated(i) = max(lb(i), min(mutated(i), ub(i)));
    end
end


function solCopy = copyDroneSolutions(drone_sols)
    numDrones = numel(drone_sols);
    solCopy = struct('Position',[],'CartPosition',[]);
    solCopy = repmat(solCopy,1,numDrones);
    for k = 1:numDrones
        solCopy(k).Position = drone_sols(k).Position;              
        solCopy(k).CartPosition = drone_sols(k).CartPosition;      
    end
end

function rep = UpdateRepWithGA(rep, model, VarMin, VarMax, nRep, CostFunction)

    if isempty(rep) || isempty(rep(1).Position)
        return;
    end

    num_rep = numel(rep);
    num_drones = numel(rep(1).Position);

    num_off = min(ceil(num_rep/2), 10);  
    sel = randperm(num_rep, num_off);
    offspring = repmat(rep(1), num_off, 1);

    for idx = 1:num_off
        r = sel(idx);
        if num_rep > 1
            idx2 = randi(num_rep);
            while idx2 == r
                idx2 = randi(num_rep);
            end
        else
            idx2 = r;
        end

        childPos = rep(r).Position;

        for k = 1:num_drones
            p1 = rep(r).Position(k).Position;
            p2 = rep(idx2).Position(k).Position;

            newGene = struct();
            for comp = {'r','psi','phi'}
                f = comp{1};
                val = sbx(p1.(f), p2.(f), VarMin.(f), VarMax.(f));
                val = mutate(val, VarMin.(f), VarMax.(f));
                val = max(val, VarMin.(f));
                val = min(val, VarMax.(f));
                newGene.(f) = val;
            end
            childPos(k).Position = newGene;
            childPos(k).CartPosition = PGSphericalToCart(newGene, model, k);
        end
    
        all_cart = [childPos.CartPosition];
    
        coords_ok = true;
        for cc = 1:numel(all_cart)
            if any(~isreal(struct2array(all_cart(cc))) | isnan(struct2array(all_cart(cc))) | isinf(struct2array(all_cart(cc))))
                coords_ok = false;
                break;
            end
        end
    
        offspring(idx).Position = childPos;
        offspring(idx).CartPosition = all_cart;
    
        if coords_ok
            offspring(idx).Cost = CostFunction(all_cart);
        else
            offspring(idx).Cost = inf(1,4);  
        end
    
        offspring(idx).Best = offspring(idx);
    end

    rep = [rep; offspring];
    
    if numel(rep) > nRep
        sum_costs = arrayfun(@(w) sum(w.Cost), rep);
        [~, idx] = sort(sum_costs);
        rep = rep(idx(1:nRep));
    end
end
