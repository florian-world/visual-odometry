function dist = hamming_dist(v1,v2)
    % INPUT:
    % v1 = NxK
    % v2 = NxK
    % OUTPUT:
    % dist = Nx1
    
    dist = sum(abs(v1-v2),2);
end