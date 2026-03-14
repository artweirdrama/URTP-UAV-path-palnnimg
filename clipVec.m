function x = clipVec(x, lb, ub)
% Clip vector x element-wise to [lb, ub]
x = min(max(x, lb), ub);
end