function yes = dominates(a, b)
yes = all(a <= b) && any(a < b);
end