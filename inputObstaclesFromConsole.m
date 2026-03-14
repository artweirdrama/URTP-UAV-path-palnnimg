function obs = inputObstaclesFromConsole(mapSize)
fprintf('Input obstacle count N: ');
N = input('');
obs = zeros(N, 3);
for i = 1:N
    fprintf('Obstacle %d as [x y h]: ', i);
    v = input('');
    if numel(v) ~= 3
        error('Each obstacle must be [x y h].');
    end
    x = min(max(round(v(1)), 1), mapSize(1));
    y = min(max(round(v(2)), 1), mapSize(2));
    h = min(max(round(v(3)), 0), mapSize(3));
    obs(i, :) = [x, y, h];
end
end