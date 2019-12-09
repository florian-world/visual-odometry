function descriptors = describeKeyPoints(I_unfilt,u,v)
    % INPUT:
    % I = r x c Image
    % u = Nx1
    % v = Nx1
    % (u,v) = keypoints pixel coordinates
    % OUTPUT:
    % descriptors = Nx256
    
    % BRIEF DESCRIPTOR:
    %   -Sample 256 intensity pairs (p1(i),p2(i)) where i=1,..,256
    %    within a squared patch around the keypoint
    %   -Create an empty 256-element descriptor
    %   -For each pair (ith), if Ip1(i)<Ip2(i) set ith bit at 1 else 0
    %
    % Pattern is generated randomly only once then the same pattern is
    % used for all patches
    global PATCHRADIUS
    I = imgaussfilt(I_unfilt,3);
    % patch size
    patch_edge = (2*PATCHRADIUS+1);
    patch_elements = patch_edge^2;
    % pattern generation
    s = 2;
    rng(s)
    pair_ind = round(patch_elements*rand(256,2)); % pattern linear indecies
    
%     I = randn(600);
%     u = [100;200;300];
%     v = [150;300;450];
    
    % grid to describe patch around (0,0) 
    [R,C]=meshgrid(-PATCHRADIUS:PATCHRADIUS,-PATCHRADIUS:PATCHRADIUS);
    % Generating one grid for each keypoint
    Rn = repmat(R,1,1,size(u,1));
    Cn = repmat(C,1,1,size(u,1));
    % each keypoint coordinate is assigned to a matrix as big as the patch 
    u1 = reshape(u,1,1,size(u,1)).*ones(size(Rn));
    v1 = reshape(v,1,1,size(u,1)).*ones(size(Cn));
    % linear index for each element of each patch
    ind = sub2ind(size(I),(u1+Rn),(v1+Cn));
    % array of patches
    parchArray = I(ind); 
    % extending geneerated patern to all patches
    p_ind1 = repmat(pair_ind(:,1),1,size(u,1));
    p_ind2 = repmat(pair_ind(:,2),1,size(u,1));
    p_ind11 = p_ind1+ones(size(p_ind1)).*([0:size(u,1)-1]*patch_elements);
    p_ind21 = p_ind2+ones(size(p_ind2)).*([0:size(u,1)-1]*patch_elements);
    % selecting points from the pattern in each patch
    Point1 = parchArray(p_ind11);
    Point2 = parchArray(p_ind21);
    % computing difference for each pair
    difference = Point2-Point1;
    % evaluating difference
    descriptors=difference>0;
end