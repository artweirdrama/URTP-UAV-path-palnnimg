function b = PGDominates(x,y)
    if isstruct(x), x = x.Cost; end
    if isstruct(y), y = y.Cost; end

    if ~isrow(x)
        x = x(:)';   
    end
    if ~isrow(y)
        y = y(:)'; 
    end
    c1 = all(x <= y); 
    c2 = any(x < y);   

    b = c1 && c2;     

end