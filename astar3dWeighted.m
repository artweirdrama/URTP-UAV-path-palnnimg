function [path, ok, comp] = astar3dWeighted(cfg, w)
occ = cfg.occupancy;
threat = cfg.threatMap;
terrain = cfg.terrainMap;
wind = cfg.windField;

Nx = cfg.mapSize(1); Ny = cfg.mapSize(2); Nz = cfg.mapSize(3);
start = cfg.start;
goal = cfg.goal;

if any(start < 1) || start(1) > Nx || start(2) > Ny || start(3) > Nz || ...
   any(goal < 1) || goal(1) > Nx || goal(2) > Ny || goal(3) > Nz
    path = [];
    ok = false;
    comp = struct();
    return;
end

if occ(start(1), start(2), start(3)) || occ(goal(1), goal(2), goal(3))
    path = [];
    ok = false;
    comp = struct();
    return;
end

zMin = max(1, cfg.flyRange(1));
zMax = min(Nz, cfg.flyRange(2));
if start(3) < zMin || start(3) > zMax || goal(3) < zMin || goal(3) > zMax
    path = [];
    ok = false;
    comp = struct();
    return;
end

if strcmp(cfg.neighborMode, '6')
    M = [1,0,0; -1,0,0; 0,1,0; 0,-1,0; 0,0,1; 0,0,-1];
else
    M = [];
    for dx = -1:1
        for dy = -1:1
            for dz = -1:1
                if dx == 0 && dy == 0 && dz == 0
                    continue;
                end
                M = [M; dx, dy, dz]; %#ok<AGROW>
            end
        end
    end
end

Ntot = Nx * Ny * Nz;
INF = 1e18;

g = INF * ones(Ntot, 1);
parent = zeros(Ntot, 1, 'uint32');
closed = false(Ntot, 1);

fHeap = zeros(cfg.maxExpandedNodes * 2, 2);
heapSize = 0;

sid = sub2ind([Nx, Ny, Nz], start(1), start(2), start(3));
gid = sub2ind([Nx, Ny, Nz], goal(1), goal(2), goal(3));

g(sid) = 0;
[fHeap, heapSize] = heapPush(fHeap, heapSize, [heuristic(start, goal, w(1)), sid]);

expanded = 0;

distObs = distanceToObstacle(occ);

while heapSize > 0 && expanded < cfg.maxExpandedNodes
    [curr, fHeap, heapSize] = heapPop(fHeap, heapSize);
    u = curr(2);
    if closed(u)
        continue;
    end
    closed(u) = true;
    expanded = expanded + 1;

    if u == gid
        break;
    end

    [x, y, z] = ind2sub([Nx, Ny, Nz], u);

    for k = 1:size(M, 1)
        nx = x + M(k, 1);
        ny = y + M(k, 2);
        nz = z + M(k, 3);

        if nx < 1 || nx > Nx || ny < 1 || ny > Ny || nz < zMin || nz > zMax
            continue;
        end
        if occ(nx, ny, nz)
            continue;
        end

        v = sub2ind([Nx, Ny, Nz], nx, ny, nz);
        if closed(v)
            continue;
        end

        step = M(k, :);
        stepDist = norm(step);

        baseMove = w(2) * stepDist;
        climb = w(8) * max(step(3), 0);
        descend = 0.8 * w(8) * max(-step(3), 0);

        threatCost = w(3) * threat(nx, ny, nz);
        terrainCost = w(4) * terrain(nx, ny, nz);

        dObs = distObs(nx, ny, nz);
        safetyCost = (w(3) * 0.2) * exp(-dObs / max(w(5), 1e-6));

        windVec = squeeze(wind(nx, ny, nz, :))';
        dirVec = step / max(stepDist, 1e-6);
        headwind = max(0, dot(windVec, dirVec));
        windCost = w(10) * headwind;

        speed = 1.0 + 0.55 * stepDist;
        speedCost = w(9) * speed^2 * (1 + 0.4 * threat(nx, ny, nz));

        smoothCost = 0;
        pu = parent(u);
        if pu ~= 0
            [px, py, pz] = ind2sub([Nx, Ny, Nz], pu);
            prevStep = [x - px, y - py, z - pz];
            if norm(prevStep) > 0
                cosAng = dot(prevStep, step) / (norm(prevStep) * norm(step));
                cosAng = min(max(cosAng, -1), 1);
                ang = acos(cosAng);
                smoothCost = w(6) * (ang / pi);
            end
        end

        edge = baseMove + climb + descend + threatCost + terrainCost + safetyCost + speedCost + windCost + smoothCost;
        gv = g(u) + edge;

        if gv < g(v)
            g(v) = gv;
            parent(v) = uint32(u);
            fv = gv + heuristic([nx, ny, nz], goal, w(1));
            [fHeap, heapSize] = heapPush(fHeap, heapSize, [fv, v]);
        end
    end
end

if ~closed(gid) && g(gid) >= 1e17
    path = [];
    ok = false;
    comp = struct();
    return;
end

p = gid;
path = zeros(0, 3);
while p ~= 0
    [x, y, z] = ind2sub([Nx, Ny, Nz], p);
    path = [x, y, z; path]; %#ok<AGROW>
    if p == sid
        break;
    end
    p = parent(p);
end

if isempty(path) || any(path(1, :) ~= start)
    ok = false;
    comp = struct();
    return;
end

ok = true;
comp = computePathMetrics(path, cfg, w, distObs);
end

function h = heuristic(p, goal, w_h)
d = norm(double(goal - p));
h = w_h * d;
end

function distObs = distanceToObstacle(occ)
[Nx, Ny, Nz] = size(occ);
INF = 1e9;
distObs = INF * ones(Nx, Ny, Nz);

q = zeros(numel(occ), 3, 'uint16');
head = 1; tail = 0;

[idxX, idxY, idxZ] = ind2sub([Nx, Ny, Nz], find(occ));
for i = 1:numel(idxX)
    x = idxX(i); y = idxY(i); z = idxZ(i);
    distObs(x, y, z) = 0;
    tail = tail + 1;
    q(tail, :) = uint16([x, y, z]);
end

if tail == 0
    distObs(:) = max([Nx, Ny, Nz]);
    return;
end

nbr = [1,0,0; -1,0,0; 0,1,0; 0,-1,0; 0,0,1; 0,0,-1];

while head <= tail
    cur = double(q(head, :));
    head = head + 1;
    cx = cur(1); cy = cur(2); cz = cur(3);
    base = distObs(cx, cy, cz);

    for k = 1:6
        nx = cx + nbr(k, 1);
        ny = cy + nbr(k, 2);
        nz = cz + nbr(k, 3);
        if nx < 1 || nx > Nx || ny < 1 || ny > Ny || nz < 1 || nz > Nz
            continue;
        end
        nd = base + 1;
        if nd < distObs(nx, ny, nz)
            distObs(nx, ny, nz) = nd;
            tail = tail + 1;
            q(tail, :) = uint16([nx, ny, nz]);
        end
    end
end
end

function [heap, heapSize] = heapPush(heap, heapSize, item)
heapSize = heapSize + 1;
if heapSize > size(heap, 1)
    heap = [heap; zeros(size(heap, 1), 2)]; %#ok<AGROW>
end
heap(heapSize, :) = item;
i = heapSize;
while i > 1
    p = floor(i / 2);
    if heap(p, 1) <= heap(i, 1)
        break;
    end
    tmp = heap(p, :);
    heap(p, :) = heap(i, :);
    heap(i, :) = tmp;
    i = p;
end
end

function [item, heap, heapSize] = heapPop(heap, heapSize)
item = heap(1, :);
heap(1, :) = heap(heapSize, :);
heapSize = heapSize - 1;
i = 1;
while true
    l = 2 * i;
    r = l + 1;
    if l > heapSize
        break;
    end
    m = l;
    if r <= heapSize && heap(r, 1) < heap(l, 1)
        m = r;
    end
    if heap(i, 1) <= heap(m, 1)
        break;
    end
    tmp = heap(i, :);
    heap(i, :) = heap(m, :);
    heap(m, :) = tmp;
    i = m;
end
end