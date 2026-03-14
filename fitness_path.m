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
        if segmentSphereIntersect(p1,p2,c,r)
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

function flag = segmentSphereIntersect(p1,p2,c,r)
% 判断线段 p1-p2 是否与球 (c,r) 相交
d = p2 - p1;
f = p1 - c;
a = dot(d,d);
b = 2*dot(f,d);
c0 = dot(f,f) - r^2;

% solve a*t^2 + b*t + c0 = 0
disc = b^2 - 4*a*c0;
if disc < 0
    flag = false;
    return;
end

t1 = (-b - sqrt(disc)) / (2*a);
t2 = (-b + sqrt(disc)) / (2*a);

% 如果任一交点在 [0,1] 范围内，则相交
if (t1>=0 && t1<=1) || (t2>=0 && t2<=1)
    flag = true;
else
    flag = false;
end
end
