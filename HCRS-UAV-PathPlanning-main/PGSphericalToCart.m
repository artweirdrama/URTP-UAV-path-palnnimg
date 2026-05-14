
function position = PGSphericalToCart(sol, model,drone_idx)
    r = sol.r;
    psi = sol.psi;
    phi = sol.phi;
    
    %% 初始化起点坐标
    xs = model.starts(1, drone_idx);  % 起点X坐标
    ys = model.starts(2, drone_idx);  % 起点Y坐标
    zs = model.starts(3, drone_idx);  % 起点Z_
    

    x(1) = xs + r(1)*cos(psi(1))*sin(phi(1)); 
    x(1) = min(max(x(1), model.xmin), model.xmax); 
    
    y(1) = ys + r(1)*cos(psi(1))*cos(phi(1));
    y(1) = min(max(y(1), model.ymin), model.ymax);

    z(1) = zs + r(1)*sin(psi(1));
    z(1) = min(max(z(1), model.zmin), model.zmax);
    

    for i = 2:model.n

        x(i) = x(i-1) + r(i)*cos(psi(i))*sin(phi(i));
        x(i) = min(max(x(i), model.xmin), model.xmax); 
        

        y(i) = y(i-1) + r(i)*cos(psi(i))*cos(phi(i));
        y(i) = min(max(y(i), model.ymin), model.ymax);

        z(i) = z(i-1) + r(i)*sin(psi(i));
        z(i) = min(max(z(i), model.zmin), model.zmax);
    end

    position = struct('x', x, 'y', y, 'z', z);
end