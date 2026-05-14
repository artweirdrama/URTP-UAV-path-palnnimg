%********************************************************************************************
% Discription:  绘制连接起点和终点的所有路径，以及它们的修改路径
% input:        TrajSeqCell             无人机路径信息的元胞数组
% input:        ObsInfo                 障碍物信息矩阵
% input:        Property                路径规划参数结构体
%********************************************************************************************

function Plot_Traj_Multi_Modification(TrajSeqCell,ObsInfo,Property)
[~,n]=size(TrajSeqCell);                    % 获取路径数量
scale=Property.scale;                        % 获取绘图比例

for i=1:n
    TrajSeq=TrajSeqCell{1,i};
    [dubins_num,~]=size(TrajSeq);           % 获取Dubins路径段数量
    increment_num=dubins_num*2;              % 计算增量数量（起始圆半径和终止圆半径的增量）
    
    Increment=zeros(1,increment_num);        % 初始化半径增量数组
    
    if i==1
        % 绘制基础图像和标准路径
        [o1,l1]=Plot_Traj_Single(TrajSeq,ObsInfo,Property,0);    
    else
        % 获取离散航路点序列并绘制主要路径
        [Traj_x,Traj_y]=Traj_Discrete(TrajSeq,Property);         
        hold on;
        l1=plot(Traj_x*scale,Traj_y*scale,'k');                  % 绘制基本路径
        l1.LineWidth=1.5;                                         % 设置线宽
    end
    
    % 通过循环随机生成修改后的路径
    for j=1:100                              
        % 随机生成半径增量
        for k=1:increment_num
            Increment(k)=rand*Property.increment;    % 随机生成大于0的增量
        end
        
        % 基于增量生成新的路径序列矩阵
        [TrajSeq_new,flag]=Traj_Seq_Modification(TrajSeq,Increment,ObsInfo,Property);
        [dubins_num,~]=size(TrajSeq_new);          % 获取新路径的Dubins路径段数量
        
        % 判断路径是否满足条件：不与障碍物相交且终止弧度小于6rad
        if flag==2&&TrajSeq_new(dubins_num,23)<6   
            hold on;                               % 在同一图上绘制修改后的路径
            [Traj_x,Traj_y]=Traj_Discrete(TrajSeq_new,Property);  % 获取离散航路点序列
            l2=plot(Traj_x*scale,Traj_y*scale,'k');               % 绘制修改后的路径
            l2.LineWidth=0.5;                      % 设置路径宽度
            l2.Color(4)=0.2;                       % 设置路径透明度
        end
    end
end

% 添加图例
L=legend([l1,l2,o1],{'Path-Basic',...
    'Path-Modification','Threaten Area'});
L.Location='northeast';                      % 设置图例位置
L.FontSize=12;                              % 设置图例字体大小
end