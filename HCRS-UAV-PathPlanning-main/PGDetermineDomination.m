
function pop = PGDetermineDomination(pop)

    if isempty(pop)
        return;
    end
    

    nPop = numel(pop);  
    
    for i = 1:nPop
        pop(i).IsDominated = false;  
    end

    for i = 1:nPop-1
        
        if any(pop(i).Cost == Inf)
            pop(i).IsDominated = true;  
        end

        for j = i+1:nPop
            
            if PGDominates(pop(i), pop(j))
               pop(j).IsDominated = true;
            end
            
            if PGDominates(pop(j), pop(i))
               pop(i).IsDominated = true;
            end
            
        end  % end inner for
        
    end  % end outer for

    if any(pop(end).Cost == Inf)
            pop(end).IsDominated = true;
    end
    
    for j = 1:nPop-1
        
        if PGDominates(pop(end), pop(j))
           pop(j).IsDominated = true;
        end
        
        if PGDominates(pop(j), pop(end))
           pop(end).IsDominated = true;
           break;  
        end 
        
    end  % end for
    
end  % end function