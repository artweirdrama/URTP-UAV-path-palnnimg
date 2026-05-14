function model = CreateModel1()
    H = imread('ChrismasTerrain2.tif'); 
    H(H < 0) = 0;  
    MAPSIZE_X = size(H, 2); 
    MAPSIZE_Y = size(H, 1); 
    [X, Y] = meshgrid(1:MAPSIZE_X, 1:MAPSIZE_Y); 

    model.threats = [
        400,350,150,70;
        500, 600, 150, 100;    
        200, 600, 150, 110;   
        650, 300, 150, 110;   
    ];

    model.num_drones = 5;  
    model.n = 10;    
    
    % 定义3架无人机的起点坐标 [X; Y; Z]
    model.starts = [
        100,300,150;
        180,250,150;
        200,200,150;
        250,150,200;
        300,100,200
    ]';
    
    % 定义3架无人机的终点坐标 [X; Y; Z]
    model.ends = [
        500,900,150;
        600,850,150 ;
        700,800,150;
        800,700,150;
        900,600,150
        
  
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

