function APlotSolution(sols, model, smooth, colors)
    figure(1)
    PlotModel(model); 
    hold on;

    % 循环绘制所有路径
    for drone_id = 1:numel(sols)
        sol = sols{drone_id};
        color = colors{drone_id};
        
        % 处理路径（传入当前无人机ID）
        [xyzp, x_all, y_all, z_all] = processSinglePath(sol, model, smooth, drone_id);
        
        % 绘制路径
        plot3(xyzp(1,:), xyzp(2,:), xyzp(3,:), 'Color', color, 'LineWidth', 1.5);
        
        % 标记当前无人机的起点终点
        plot3(x_all(1), y_all(1), z_all(1), 'ks', 'MarkerSize',7, 'MarkerFaceColor','k');
        plot3(x_all(end), y_all(end), z_all(end), 'ko', 'MarkerSize',7, 'MarkerFaceColor','k');
    end
    hold off;
    axis vis3d;
    grid on;

    %% 俯视图（保留障碍物）
    figure(3)
    mesh(model.X, model.Y, model.H); % 地形
    colormap summer;
    set(gca, 'Position', [0 0 1 1]);
    axis equal vis3d on;
    shading interp;
    material dull;                  
    camlight left;                  
    lighting gouraud; 
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    hold on;
    % 绘制障碍物（红色同心圆）
    threats = model.threats;
    for i = 1:size(threats,1)
        threat = threats(i,:);
        threat_x = threat(1); threat_y = threat(2); threat_radius = threat(4);
        threat_z = max(model.H(:)) + 1;
        for j = 1:3
            theta = linspace(0,2*pi,2000);
            x = threat_radius*cos(theta) + threat_x;
            y = threat_radius*sin(theta) + threat_y;
            z = zeros(size(x)) + threat_z;
            plot3(threat_x, threat_y, threat_z, 'o', 'Color','red', 'MarkerSize',3, 'MarkerFaceColor','red');
            plot3(x, y, z, '-', 'Color','red', 'LineWidth',1);
            threat_radius = threat_radius - 20;
        end
    end
    for drone_id = 1:numel(sols)
        sol = sols{drone_id};
        color = colors{drone_id};
        
        % 处理路径（传入当前无人机ID）
        [xyzp, x_all, y_all, z_all] = processSinglePath(sol, model, smooth, drone_id);
        
        plot3(xyzp(1,:), xyzp(2,:), xyzp(3,:), 'Color', color, 'LineWidth', 1.5);
        
        % 标记当前无人机的起点终点
        plot3(x_all(1), y_all(1), z_all(1), 'ks', 'MarkerSize',7, 'MarkerFaceColor','k');
        plot3(x_all(end), y_all(end), z_all(end), 'ko', 'MarkerSize',7, 'MarkerFaceColor','k');
    end
    view(0,90);
    hold off;
end

function [xyzp, x_all, y_all, z_all] = processSinglePath(sol, model, smooth, drone_id)
    x = sol.x; 
    y = sol.y; 
    z = sol.z;
    
    current_start = model.starts(:, drone_id);
    current_end = model.ends(:, drone_id);
    
    x_all = [current_start(1), x, current_end(1)];
    y_all = [current_start(2), y, current_end(2)];
    z_all = [current_start(3), z, current_end(3)];

    for i = 1:numel(x_all)
        x_coord = max(min(round(x_all(i)), model.xmax), model.xmin);
        y_coord = max(min(round(y_all(i)), model.ymax), model.ymin);
        
        z_map = model.H(y_coord, x_coord);
        z_all(i) = z_all(i) + z_map;
    end

    xyz = [x_all; y_all; z_all];
    xyzp = zeros(size(xyz));
    for k = 1:3
        pp = csaps(1:size(xyz,2), xyz(k,:), smooth);
        xyzp(k,:) = ppval(pp, 1:size(xyz,2));
    end
end

