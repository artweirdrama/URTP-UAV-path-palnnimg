function s = computeRouteStats(path, metrics)
s.nodes = size(path, 1);
s.directDist = norm(double(path(end, :) - path(1, :)));
s.detourRatio = metrics.length / max(s.directDist, 1e-6);
s.avgAlt = mean(path(:, 3));

dz = diff(path(:, 3));
s.climbDist = sum(max(dz, 0));
s.descDist = sum(max(-dz, 0));

s.maxTurnDeg = 0;
s.turnOver45 = 0;
if size(path, 1) >= 3
    for i = 3:size(path, 1)
        a = path(i - 1, :) - path(i - 2, :);
        b = path(i, :) - path(i - 1, :);
        if norm(a) < 1e-9 || norm(b) < 1e-9
            continue;
        end
        c = dot(a, b) / (norm(a) * norm(b));
        c = min(max(c, -1), 1);
        ang = acosd(c);
        s.maxTurnDeg = max(s.maxTurnDeg, ang);
        if ang > 45
            s.turnOver45 = s.turnOver45 + 1;
        end
    end
end
end