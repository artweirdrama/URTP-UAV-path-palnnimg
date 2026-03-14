function keep = crowdingPrune(V, K)
N = size(V, 1);
if N <= K
    keep = true(N, 1);
    return;
end

M = size(V, 2);
score = zeros(N, 1);
for m = 1:M
    [s, idx] = sort(V(:, m));
    score(idx(1)) = inf;
    score(idx(end)) = inf;
    span = max(s(end) - s(1), 1e-9);
    for i = 2:N-1
        score(idx(i)) = score(idx(i)) + (s(i+1) - s(i-1)) / span;
    end
end

[~, order] = sort(score, 'descend');
keep = false(N, 1);
keep(order(1:K)) = true;
end