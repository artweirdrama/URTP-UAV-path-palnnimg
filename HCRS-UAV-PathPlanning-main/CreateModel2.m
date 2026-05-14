
function model = CreateModel2()
    H = imread('ChrismasTerrain2.tif'); 
    H(H < 0) = 0; 
    
    MAPSIZE_X = size(H, 2); 
    MAPSIZE_Y = size(H, 1); 
    [X, Y] = meshgrid(1:MAPSIZE_X, 1:MAPSIZE_Y); 

    model.threats = [
        300, 300, 250, 70;   
        600, 400, 250, 70;   
        450, 500, 250, 70;   
        750, 500, 250, 50;   
        250, 600, 250, 60;   
        550, 850, 150, 60   
    ];
    model.num_drones = 5;     
    model.n = 10;           
    
    % 定义3架无人机的起点坐标 [X; Y; Z]
    model.starts = [
        50, 350, 200;
        100, 300, 200; 
        % 无人机2: 避开威胁1和威胁5
        150, 250, 150; 
        % 无人机3: 避开威胁1和威胁5
        200, 200, 200; 
        250, 150, 200; 
    ]';
    
    % 定义3架无人机的终点坐标 [X; Y; Z]
    model.ends = [
        650,900,150;
        700, 850, 150;
        % 无人机2: 避开威胁4和威胁6
        820, 800, 150;
        850,750,150;
        900, 700, 150;
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

    % 调用绘图函数（需支持多无人机显示）
    test(model); 
end

