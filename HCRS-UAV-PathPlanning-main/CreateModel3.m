function model = CreateModel3()
    H = imread('ChrismasTerrain2.tif'); 
    H(H < 0) = 0;  
    
    MAPSIZE_X = size(H, 2); 
    MAPSIZE_Y = size(H, 1); 
    [X, Y] = meshgrid(1:MAPSIZE_X, 1:MAPSIZE_Y); 

    model.threats = [
        500, 500, 150, 70;   
        350, 200, 150, 100;   
        700, 450, 150, 80;   
        200, 700, 150, 110;   
        800, 250, 150, 110;   
        500, 750, 150, 80
    ];
    model.num_drones = 5;  
    model.n = 10;          
    
    % 定义3架无人机的起点坐标 [X; Y; Z]
    model.starts = [
        50,300,150; 
        100,250,150;
        150,200,150;
        200,150,150;
        250,100,250
    ]';
    
    % 定义3架无人机的终点坐标 [X; Y; Z]
    model.ends = [
        600,950,150; 
        400,950,150;
        700,850,150;
        750,800,150;
        800,750,150
    ]';
    % =================================================

    % ================ 地图边界与高度限制 =================
    model.xmin = 1;          % X方向最小坐标
    model.xmax = MAPSIZE_X;  % X方向最大坐标
    model.ymin = 1;          % Y方向最小坐标
    model.ymax = MAPSIZE_Y;  % Y方向最大坐标
    model.zmin = 100;        % 最低飞行高度
    model.zmax = 200;        % 最高飞行高度
    model.drone_speed = 5.0; % 无人机飞行速度 (m/s)
    % =================================================

    % ================ 存储地形数据 =================
    model.MAPSIZE_X = MAPSIZE_X;
    model.MAPSIZE_Y = MAPSIZE_Y;
    model.X = X;
    model.Y = Y;
    model.H = H;
    % ==============================================

    % 调用绘图函数（需同步修改PlotModel支持多无人机）
    PlotModel(model); 
end

