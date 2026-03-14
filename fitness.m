function cost = fitness_path(x, start, goal, obstacles)
% x: 1 x (3*numWaypoints) 向量
% 计算路径的代价：路径长度 + 碰撞惩罚 + 平滑度惩罚

numPoints = length(x)/3;
wp = reshape(x,3,[])';
path = [start; wp; goal];

% 路径长度
len = 0;
for i=1:size(path,1)-1
    len = len + norm(path(i+1,:) - path(i,:));
end

% 碰撞检测：若任一线段与任一球形障碍相交，加大惩罚
penalty = 0;
for i=1:size(path,1)-1
    p1 = path(i,:); p2 = path(i+1,:);
    for j=1:size(obstacles,1)
        c = obstacles(j,1:3); r = obstacles(j,4);
        if helpers_segmentSphereIntersect(p1,p2,c,r)
            penalty = penalty + 1e4; % 大惩罚
        end
    end
end

% 平滑度：角度变化越大惩罚越高
smooth = 0;
for i=2:size(path,1)-1
    v1 = path(i,:) - path(i-1,:);
    v2 = path(i+1,:) - path(i,:);
    if norm(v1)>0 && norm(v2)>0
        ang = acos( max(-1,min(1, dot(v1,v2)/(norm(v1)*norm(v2)))) );
        smooth = smooth + ang^2;
    end
end

% 总成本
cost = len + 50*smooth + penalty;
end
