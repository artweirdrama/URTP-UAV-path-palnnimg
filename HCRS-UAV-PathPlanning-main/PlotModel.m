

function PlotModel(model)

    mesh(model.X, model.Y, model.H); 
    
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
    
    hold on
    threats = model.threats;          
    threat_num = size(threats,1);     
    
    h=250; 

   
    for i = 1:threat_num
       
        threat = threats(i,:);
        threat_x = threat(1);     
        threat_y = threat(2);     
        threat_z = threat(3);     
        threat_radius = threat(4);
        [xc,yc,zc] = cylinder(threat_radius); 
      
        xc = xc + threat_x;       % X方向平移
        yc = yc + threat_y;       % Y方向平移
        zc = zc * h + threat_z;   % Z方向缩放和平移
        
        % 绘制圆柱体表面网格
        c = mesh(xc, yc, zc); 
        set(c, 'edgecolor','none', 'facecolor','#FF0000', 'FaceAlpha',.3);
    end

end