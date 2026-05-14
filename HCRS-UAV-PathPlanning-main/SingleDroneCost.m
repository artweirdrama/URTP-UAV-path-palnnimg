function cost = SingleDroneCost(sol, model, varmin, start, end_point)
    J_inf = inf;
    n = model.n;
    H = model.H;
    x = sol.x; y = sol.y; z = sol.z; 
    x_all = [start(1), x, end_point(1)];
    y_all = [start(2), y, end_point(2)];
    z_all = [start(3), z, end_point(3)];  
    N = length(x_all);  
    z_abs = zeros(1, N);
    for i = 1:N
        z_abs(i) = z_all(i) + H(round(y_all(i)), round(x_all(i)));
    end
    E_dist = 0;  
    dist_efficiency = 0.9; 
    
    for i = 1:N-1
        diff = [x_all(i+1)-x_all(i); 
               y_all(i+1)-y_all(i);
               z_abs(i+1)-z_abs(i)]; 
        
        step_len = norm(diff);
        if step_len <= varmin.r
            E_dist = J_inf;
            break;  
        end
        E_dist = E_dist + step_len * (dist_efficiency / 2);  
    end
    threats = model.threats;        
    threat_num = size(threats,1);   
    drone_size = 1;               
    danger_dist = 4*drone_size;    
    risk_factor = 0.26; 
    
    E_safety = 0;    
    for i = 1:threat_num
        threat = threats(i,:);
        threat_radius = threat(4);
        safe_margin = threat_radius + drone_size + danger_dist;
        
        for j = 1:N-1
            dist = PGDistP2S([threat(1) threat(2)],...          
                          [x_all(j) y_all(j)],...          
                          [x_all(j+1) y_all(j+1)]);       
            
            if dist > safe_margin
                current_risk = 0;    
            elseif dist < threat_radius
                current_risk = J_inf;
            else
                current_risk = (safe_margin - dist) * risk_factor;
            end
            
            E_safety = E_safety + current_risk;
        end
    end
    zmax = model.zmax;  
    zmin = model.zmin;  
    z_mid = (zmax + zmin)/2;
    E_alt = 0;
    
    for i = 1:n 
        if z(i) < 0
            E_alt = E_alt + J_inf; 
        else
            dev = abs(z(i) - z_mid);
            E_alt = E_alt + dev * (3/20); 
        end
    end
    E_smooth = 0; 
    damp_coeff = 0.12;
    
    for i = 1:N-2
        vec1 = [x_all(i+1)-x_all(i); y_all(i+1)-y_all(i); z_abs(i+1)-z_abs(i)];
        vec2 = [x_all(i+2)-x_all(i+1); y_all(i+2)-y_all(i+1); z_abs(i+2)-z_abs(i+1)];
        heading_angle = atan2(norm(cross(vec1, vec2)), dot(vec1, vec2));
        E_smooth = E_smooth + abs(heading_angle) * (damp_coeff * 2);
        delta_z = z_abs(i+1) - z_abs(i);
        h_dist = norm([x_all(i+1) - x_all(i), y_all(i+1) - y_all(i)]);
        
        if h_dist > 1e-5 
            climb_angle = atan(delta_z / h_dist);
        else
            climb_angle = 0;
        end
        E_smooth = E_smooth + abs(climb_angle) * (damp_coeff * 2);
    end
    cost = [E_dist; E_safety; E_alt; E_smooth];
end