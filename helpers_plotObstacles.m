function helpers_plotObstacles(obstacles)
% 绘制球形障碍物
[xs,ys,zs] = sphere(20);
for i=1:size(obstacles,1)
    c = obstacles(i,1:3); r = obstacles(i,4);
    surf(xs*r + c(1), ys*r + c(2), zs*r + c(3), 'FaceAlpha',0.3, 'EdgeColor','none');
end
end
