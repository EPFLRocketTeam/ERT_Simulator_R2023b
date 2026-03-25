function vUnit = normalizeVect(v)
    vNorm = norm(v);
    if vNorm == 0
       vUnit = v*0; 
    else
       vUnit = v/vNorm;
    end
end