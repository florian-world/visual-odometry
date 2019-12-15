function noRep = findNewCandidates(Points1,Points2)
%Returns bitmask of where Points1 is not close to Points2, up to some
%pixel_threshold.

% noRep = true(N,1);
pixel_threshold = 10;
distances = pdist2(Points1,Points2);
noRep = all(distances >= pixel_threshold,2);

end

