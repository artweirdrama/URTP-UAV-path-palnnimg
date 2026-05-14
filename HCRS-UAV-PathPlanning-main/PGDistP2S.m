function dist = PGDistP2S(x, a, b)
    ab = b - a;
    ax = x - a;
    ab_sqr = sum(ab.^2);
    if ab_sqr < 1e-10
        dist = norm(ax);
        return;
    end
    t = max(0, min(1, dot(ax, ab) / ab_sqr));
    projection = a + t * ab;
    dist = norm(x - projection);
end