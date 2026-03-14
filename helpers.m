function flag = helpers_segmentSphereIntersect(p1,p2,c,r)
% 判断线段 p1-p2 是否与球 (c,r) 相交
% 返回 true/false

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

function helpers_plotObstacles(obstacles)
% 绘制球形障碍物
[xs,ys,zs] = sphere(20);
for i=1:size(obstacles,1)
    c = obstacles(i,1:3); r = obstacles(i,4);
    surf(xs*r + c(1), ys*r + c(2), zs*r + c(3), 'FaceAlpha',0.3, 'EdgeColor','none');
end
end
