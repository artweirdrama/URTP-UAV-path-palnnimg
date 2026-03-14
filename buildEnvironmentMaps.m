function [occ, threatMap, terrainMap, windField] = buildEnvironmentMaps(cfg)
Nx = cfg.mapSize(1); Ny = cfg.mapSize(2); Nz = cfg.mapSize(3);
occ = false(Nx, Ny, Nz);

for i = 1:size(cfg.obstacles, 1)
    x = cfg.obstacles(i, 1);
    y = cfg.obstacles(i, 2);
    h = cfg.obstacles(i, 3);
    if x < 1 || x > Nx || y < 1 || y > Ny
        continue;
    end
    top = min(max(round(h), 0), Nz);
    if top >= 1
        occ(x, y, 1:top) = true;
    end
end

[X, Y, Z] = ndgrid(1:Nx, 1:Ny, 1:Nz);

terrainBase = cfg.terrainAmp * ( ...
    sin(cfg.terrainFreq(1) * X) + 0.7 * cos(cfg.terrainFreq(2) * Y) + 0.25 * sin(0.16 * (X + Y)) ...
    );
terrainMap = terrainBase - min(terrainBase(:));
terrainMap = terrainMap / max(terrainMap(:) + eps);

threatMap = zeros(Nx, Ny, Nz);
for i = 1:size(cfg.threatRegions, 1)
    cx = cfg.threatRegions(i, 1);
    cy = cfg.threatRegions(i, 2);
    cz = cfg.threatRegions(i, 3);
    r  = cfg.threatRegions(i, 4);
    lv = cfg.threatRegions(i, 5);
    D = sqrt((X - cx).^2 + (Y - cy).^2 + (Z - cz).^2);
    local = lv * max(0, 1 - D / max(r, 1e-6));
    threatMap = threatMap + local;
end

wx = cfg.baseWind(1) + cfg.windSwirl * 0.4 * sin(0.11 * Y) .* cos(0.09 * Z);
wy = cfg.baseWind(2) + cfg.windSwirl * 0.4 * cos(0.10 * X) .* sin(0.08 * Z);
wz = cfg.baseWind(3) + cfg.windSwirl * 0.2 * sin(0.07 * (X + Y));
windField = cat(4, wx, wy, wz);
end