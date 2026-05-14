clear;
clc;
% 添加所需的功能函数路径
addpath('Function_Plot');        % 添加绘图函数路径
addpath('Function_Dubins');      % 添加Dubins路径相关函数路径
addpath('Function_Trajectory');   % 添加轨迹生成函数路径

%% 初始化数据
Property.obs_last=0;              % 记录当前轨迹规划中已避开的障碍物
Property.invasion=0;              % 记录轨迹规划过程中是否有侵入障碍物（威胁区域）
Property.mode=2;                  % 设置轨迹生成模式：1-最短路径；2-常规路径
Property.ns=50;                   % 设置起始弧段离散点数量
Property.nl=50;                   % 设置直线段离散点数量
Property.nf=50;                   % 设置终止弧段离散点数量
Property.max_obs_num=5;           % 设置每次路径规划最大检测障碍物数量
Property.max_info_num=20;         % 设置每个规划步骤存储的最大路径段数量
Property.max_step_num=4;          % 设置路径的最大规划步数
Property.Info_length=33;          % 设置每个路径信息的长度
Property.radius=100*1e3;          % 设置无人机转弯半径（毫米）
Property.scale=1/1000;            % 设置比例尺
Property.increment=20*1e3;        % 设置路径长度增量的调整范围
Property.selection1=3;            % 设置路径筛选模式1
Property.selection2=1;            % 设置路径筛选模式2
                                 % =1：路径不与障碍物相交
                                 % =2：路径的转向角不超过3*pi/2
                                 % =3：同时满足1和2

% 设置起始点信息 [x坐标, y坐标, 偏航角, 起始弧半径]
StartInfo=[ 50*1e3,  30*1e3,  0,     100*1e3; ];     % 无人机3起始点信息（单位：毫米）

% 设置终止点信息 [x坐标, y坐标, 偏航角, 终止弧半径]
FinishInfo=[550*1e3, 120*1e3, 0,    100*1e3;  ];       % 无人机3终止点信息（单位：毫米）

% 设置障碍物（威胁圈）信息 [x坐标, y坐标, 威胁圈半径]
ObsInfo=[  150*1e3,  50*1e3,  50*1e3;             % 障碍物1信息
           300*1e3, 100*1e3,  20*1e3;             % 障碍物2信息
           400*1e3,  50*1e3,  50*1e3;             % 障碍物3信息
           200*1e3, 200*1e3,  50*1e3];            % 障碍物4信息（单位：毫米）

% 获取无人机数量和障碍物数量
[uav_num,~]=size(StartInfo);                      % 获取无人机数量
[obs_num,~]=size(ObsInfo);                        % 获取障碍物数量

% 初始化无人机协同状态结构体
Coop_State(1:uav_num)=struct(...                  % 无人机飞行路径信息的结构体
    'traj_length',[],...                          % 所有路径长度数组
    'traj_length_max',0,...                       % 最大路径长度
    'traj_length_min',0,...                       % 最小路径长度
    'TrajSeqCell',[],...                          % 路径序列单元数组
    'ideal_length',12*1e5,...                     % 期望路径长度
    'optim_length',0,...                          % 优化后的路径长度
    'traj_index_top',0,...                        % 大于且最接近期望路径长度的路径索引
    'traj_index_bottom',0,...                     % 小于且最接近期望路径长度的路径索引
    'TrajSeq_Coop',[]);                           % 协同路径序列矩阵

%% 按顺序规划每个无人机从起点到终点的路径
for uav_index=1:1                                 % 遍历每个无人机
    start_info=StartInfo(uav_index,:);            % 获取当前无人机的起始点信息
    finish_info=FinishInfo(uav_index,:);          % 获取当前无人机的终止点信息
    Property.radius=start_info(4);                % 根据初始信息设置无人机转弯半径
    
    % 计算当前无人机所有可行的飞行路径
    TrajSeqCell=Traj_Collection(start_info,finish_info,ObsInfo,Property);  
    
    % 从可行飞行路径中选择基础路径并优化生成协同路径
    Coop_State(uav_index)=Coop_State_Update(TrajSeqCell,Coop_State(uav_index),ObsInfo,Property);       

    % 绘制多条路径修改结果
    Plot_Traj_Multi_Modification(TrajSeqCell,ObsInfo,Property);
end

% 绘制协同路径规划结果
Plot_Traj_Coop(Coop_State,ObsInfo,Property,1,1);