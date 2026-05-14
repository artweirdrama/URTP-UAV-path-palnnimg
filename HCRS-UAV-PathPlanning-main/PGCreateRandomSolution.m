function sol = PGCreateRandomSolution(VarSize, VarMin, VarMax, model,start)
    sol.r = unifrnd(VarMin.r, VarMax.r, VarSize);
    sol.psi = unifrnd(VarMin.psi, VarMax.psi, VarSize);
    sol.phi = unifrnd(VarMin.phi, VarMax.phi, VarSize);
end