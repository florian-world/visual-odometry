function [M1, M2, D2] = matchDescriptors(d1,d2,l1,l2)
%MATCH DESCRIPTORS 
%   Input:
%           d1: [256xP] array containing P descriptors
%           d2: [256xQ] array containing Q descriptors
%           l1: [Px2] array containing P [x,y] locations
%           l2: [Qx2] array containing Q [x,y] locations
%
%   Output:
%           M1: [2xM] matrix of [x,y] coordinates of matches from l1
%           M2: [2xM] matrix of [x,y] cooridnates of matches from l2
%           D2: [256xM] matrix of descriptors of img2


global MATCHING_THRESHOLD

[dists,matches] = pdist2(double(d1'),double(d2'),'hamming', 'Smallest', 1);

sorted_dists = sort(dists);
sorted_dists = sorted_dists(sorted_dists~=0);
min_non_zero_dist = sorted_dists(1);

% matches(dists >= MATCHING_LAMBDA * min_non_zero_dist) = 0;
matches(dists >= MATCHING_THRESHOLD) = 0;

% remove double matches
unique_matches = zeros(size(matches));
[~,unique_match_idxs,~] = unique(matches, 'stable');
unique_matches(unique_match_idxs) = matches(unique_match_idxs);

matches = unique_matches;

idxs_l2_logical = matches > 0;
idxs_l1 = matches(idxs_l2_logical);


M1 = l1(idxs_l1,:);
M2 = l2(idxs_l2_logical,:);
D2 = d2(:,idxs_l2_logical);

assert(all(size(M1) == size(M2)));

end