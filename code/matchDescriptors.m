function [M1, M2] = matchDescriptors(d1,d2,l1,l2)
%MATCH DESCRIPTORS 
%   Input:
%           d1: [256xP] array containing P descriptors
%           d2: [256xQ] array containing Q descriptors
%           l1: [2xP] array containing P [x,y] locations
%           l2: [2xQ] array containing Q [x,y] locations
%
%   Output:
%           M1: [Mx2] matrix of [x,y] coordinates of matches from l1
%           M2: [Mx2] matrix of [x,y] cooridnates of matches from l2


global MATCHING_LAMBDA

[dists,matches] = pdist2(double(d1'),double(d2'),'hamming', 'Smallest', 1);

sorted_dists = sort(dists);
sorted_dists = sorted_dists(sorted_dists~=0);
min_non_zero_dist = sorted_dists(1);

matches(dists >= MATCHING_LAMBDA * min_non_zero_dist) = 0;

% remove double matches
unique_matches = zeros(size(matches));
[~,unique_match_idxs,~] = unique(matches, 'stable');
unique_matches(unique_match_idxs) = matches(unique_match_idxs);

matches = unique_matches;

idxs_l2_logical = matches > 0;
idxs_l1 = matches(idxs_l2_logical);


M1 = l1(:,idxs_l1)';
M2 = l2(:,idxs_l2_logical)';

assert(all(size(M1) == size(M2)));

end