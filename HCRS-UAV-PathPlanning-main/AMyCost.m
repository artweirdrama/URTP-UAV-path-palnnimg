
function cost = AMyCost(sol, model)
    J1 = 0;  % 路径效率成本（与路径长度相关）
    J2 = 0;  % 威胁规避成本（与障碍物距离相关）
    J3 = 0;  % 高度保持成本（与飞行高度相关）
    J4 = 0;  % 路径平滑成本（与路径曲率相关）
    J_collision = 0;  
    
    num_drones = model.num_drones;
    
    drone_paths = cell(1, num_drones);
    n = model.n;  
    
    for k = 1:num_drones

        drone_sol = sol(k);
        
        current_start = model.starts(:, k)';  
        current_end = model.ends(:, k)';     
        
        dist = norm(current_end - current_start);  
        VarMax.r = 2 * dist / n; 
        VarMin.r = 0;             
        AngleRange = pi/4;
        VarMin.psi = -AngleRange;  % 最小俯仰角
        VarMax.psi = AngleRange;   % 最大俯仰角
        
        dirVector = current_end - current_start;
        phi0 = atan2(dirVector(2), dirVector(1));  
        VarMin.phi = phi0 - AngleRange; 
        VarMax.phi = phi0 + AngleRange;  
        drone_cost = SingleDroneCost(drone_sol, model, VarMin, current_start, current_end);
        J1 = J1 + drone_cost(1);  
        J2 = J2 + drone_cost(2);  
        J3 = J3 + drone_cost(3); 
        J4 = J4 + drone_cost(4);  
        
        [x_all, y_all, z_abs, t_all] = buildFullPathWithTime(drone_sol, model, current_start, current_end);
        
        drone_paths{k} = struct('x', x_all, 'y', y_all, 'z', z_abs, 't', t_all);
    end
    if num_drones > 1
        J_collision = calculateTimeSpaceCollisionCost(drone_paths, model);
        
        J2 = J2 + J_collision;
    end
    cost = [J1; J2; J3; J4];
end

function [x_all, y_all, z_abs, t_all] = buildFullPathWithTime(sol, model, start, end_point)
    x = sol.x;      
    y = sol.y;      
    z_rel = sol.z;  
    xs = start(1);      ys = start(2);      zs = start(3);
    xf = end_point(1); yf = end_point(2);  zf = end_point(3);
    x_all = [xs x xf];
    y_all = [ys y yf];
    z_all = [zs z_rel zf];
    H = model.H;  
    z_abs = zeros(1, numel(x_all));
    for i = 1:numel(x_all)
        xi = min(max(round(x_all(i)), 1), size(H, 2));
        yi = min(max(round(y_all(i)), 1), size(H, 1));
       
        terrain_z = H(yi, xi);
        
        if i == 1 || i == numel(x_all)
            z_abs(i) = z_all(i); 
        else
            z_abs(i) = z_all(i) + terrain_z;  
        end
    end
    t_all = calculatePathTiming(x_all, y_all, z_abs, model);
end

function t_all = calculatePathTiming(x_all, y_all, z_abs, model)
    default_speed = model.drone_speed;
    min_time_increment = 0.1;  
    n_points = length(x_all);
    t_all = zeros(1, n_points);
    t_all(1) = 0; 
    for i = 2:n_points
        dist = norm([x_all(i) - x_all(i-1), ...
                    y_all(i) - y_all(i-1), ...
                    z_abs(i) - z_abs(i-1)]);

        if dist > 1e-6  
            time_increment = dist / default_speed;
        else
            time_increment = min_time_increment; 
        end
        time_increment = max(time_increment, min_time_increment);

        t_all(i) = t_all(i-1) + time_increment;
    end
end


function J_collision = calculateTimeSpaceCollisionCost(drone_paths, model)
    num_drones = length(drone_paths);

    min_separation = 2.0;        
    collision_zone = 4.0;   
    %collision_cost_max = inf;   
    collision_cost_max = inf;
    collision_cost_warning = 1; 
    J_collision = 0;
    time_resolution = 1.2; 
    max_time = 0;
    for k = 1:num_drones
        max_time = max(max_time, max(drone_paths{k}.t));
    end
    time_grid = 0:time_resolution:max_time;
    interpolated_positions = cell(num_drones, 1);
    
    for k = 1:num_drones
        path = drone_paths{k};
        [unique_times, unique_idx] = unique(path.t, 'stable');
        
        if length(unique_times) < length(path.t)
            warning('无人机 %d 检测到重复时间戳，已自动处理', k);
            path.t = unique_times;
            path.x = path.x(unique_idx);
            path.y = path.y(unique_idx);
            path.z = path.z(unique_idx);
        end
        
        valid_times = time_grid <= max(path.t);
        
        if length(unique_times) > 1 && sum(valid_times) > 1
            try
                x_interp = spline(path.t, path.x, time_grid(valid_times));
                y_interp = spline(path.t, path.y, time_grid(valid_times));
                z_interp = spline(path.t, path.z, time_grid(valid_times));
                
                interpolated_positions{k} = struct(...
                    'x', x_interp, ...
                    'y', y_interp, ...
                    'z', z_interp, ...
                    'valid_times', valid_times);
            catch ME
                warning('无人机 %d 路径插值失败，使用终点位置: %s', k, ME.message);
                interpolated_positions{k} = struct(...
                    'x', repmat(path.x(end), 1, sum(valid_times)), ...
                    'y', repmat(path.y(end), 1, sum(valid_times)), ...
                    'z', repmat(path.z(end), 1, sum(valid_times)), ...
                    'valid_times', valid_times);
            end
        else
            interpolated_positions{k} = struct(...
                'x', repmat(path.x(end), 1, sum(valid_times)), ...
                'y', repmat(path.y(end), 1, sum(valid_times)), ...
                'z', repmat(path.z(end), 1, sum(valid_times)), ...
                'valid_times', valid_times);
        end
    end
    for t_idx = 1:length(time_grid)
        for k1 = 1:num_drones
            for k2 = (k1+1):num_drones
                pos1 = interpolated_positions{k1};
                pos2 = interpolated_positions{k2};
                
                if t_idx <= length(pos1.valid_times) && t_idx <= length(pos2.valid_times) && ...
                   pos1.valid_times(t_idx) && pos2.valid_times(t_idx)
                    
                    if length(pos1.x) >= t_idx && length(pos2.x) >= t_idx
                        p1 = [pos1.x(t_idx), pos1.y(t_idx), pos1.z(t_idx)];
                        p2 = [pos2.x(t_idx), pos2.y(t_idx), pos2.z(t_idx)];
                        
                        dist = norm(p1 - p2);
                        
                        if dist < min_separation
                            collision_cost = collision_cost_max;
                        elseif dist < collision_zone
                            ratio = (dist - min_separation) / (collision_zone - min_separation);
                            collision_cost = collision_cost_warning * (1 - ratio);
                        else
                            collision_cost = 0;
                        end
                        J_collision = J_collision + collision_cost;
                    end
                end
            end
        end
    end
end