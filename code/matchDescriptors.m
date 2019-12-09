function [M1, M2] = matchDescriptors(d1,d2,l1,l2)
%MATCH DESCRIPTORS 
%   Input:
%           d1: [256xP] array containing P descriptors
%           d2: [256xQ] array containing Q descriptors
%           l1: [2xP] array containing P [x,y] locations
%           l2: [2xQ] array containing Q [x,y] locations
%
%   Output:
%           M-by-2 matrix of [x,y] coordinate


[dist,iindex] = pdist2(d1,d2,'hamming');

end