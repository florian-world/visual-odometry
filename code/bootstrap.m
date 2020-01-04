function State = bootstrap(Imgs)
%BOOTSTRAP Takes two initial frames as input and returns the first state of
%          the initialization to be used by the continuos VO pipeline. 
%   
%   Input:
%       Imgs: cell array of bootstrap images
%
%   Output:
%       State:  Struct that describes the state containing following
%               fields:
%           Keypoints:                   [2xK] array containing K keypoint pairs 
%           Landmarks:                   [3xK] array containing K landmark coordinates
%           CandidateKeypoints:          [2xM] array containing M candidate keypoints
%           InitCandidateKeypoints:      [2xM] array containing M initial observations of candidate keypoints
%           InitCandidatePoses:          [12xM] array containing M initial camera poses of first
%                                         observation of candidate keypoints
global K PATCHRADIUS

assert(length(Imgs) >= 2);

[height, width] = size(Imgs{1});
% Detect corners on both frames
roi = [PATCHRADIUS+1,PATCHRADIUS+1,width-(2*PATCHRADIUS),height-(2*PATCHRADIUS)];
corners1 = detectHarrisFeatures(Imgs{1},'ROI',roi);

% Point tracking using KLT
pointTracker = vision.PointTracker('MaxBidirectionalError',1); % Set to Inf for speedup
initialize(pointTracker,corners1.Location,Imgs{1});

[trackedPoints,trackedPointsValidity] = pointTracker(Imgs{2});
KLTMatch1 = corners1.Location(trackedPointsValidity,:);
KLTMatch2 = trackedPoints(trackedPointsValidity,:);

for i = 3:length(Imgs)
    pointTracker = vision.PointTracker('MaxBidirectionalError',1); % Set to Inf for speedup
    initialize(pointTracker,KLTMatch2,Imgs{i-1});
    [trackedPoints,trackedPointsValidity] = pointTracker(Imgs{i});
    KLTMatch1 = KLTMatch1(trackedPointsValidity,:);
    KLTMatch2 = trackedPoints(trackedPointsValidity,:);
end

[F_KLT, inliersIdx] = estimateFundamentalMatrix(KLTMatch1, KLTMatch2, 'Method', ...
                              'RANSAC', 'NumTrials', 15000);

Inliers1 = KLTMatch1(inliersIdx,:);
Inliers2 = KLTMatch2(inliersIdx,:);

% figure(2);
% showMatchedFeatures(Im1, Im2, Inliers1, Inliers2);
% showMatchedFeatures(Im1, Im2, Inliers1, Inliers2, 'montage');

% Recover essential matrix from F, then decompose into R,T
E = K'*F_KLT*K;
[Rots,u3] = decomposeEssentialMatrix(E);
Inliers1_hom = Inliers1';
Inliers1_hom(3,:) = 1;
Inliers2_hom = Inliers2';
Inliers2_hom(3,:) = 1;
[R,T] = disambiguateRelativePose(Rots,u3,Inliers1_hom,Inliers2_hom,K,K);

% Triangulate points to create pointcloud
M1 = K*eye(3,4);
M2 = K*[R, T];
X_hom = linearTriangulation(Inliers1_hom,Inliers2_hom,M1,M2); % Output of this is homogenous

mask_in_sight = X_hom(3,:)>0; % ignore negative z values

if nnz(~mask_in_sight) > 0
    fprintf("Ignoring %d inliers during bootstrapping, because their landmarks would have negative z coordinates\n", ...
        nnz(~mask_in_sight));
end

Landmarks = X_hom(1:3,mask_in_sight);
Keypoints = Inliers2(mask_in_sight,1:2)';           

State.Keypoints = Keypoints;
State.Landmarks = Landmarks;
% These state entries are not calculated during bootstrapping
State.CandidateKeypoints = [];
State.InitCandidateKeypoints = [];
State.InitCandidatePoses = [];
State.LastKeyframePose = eye(3,4);

fprintf("\nBOOTSTRAP COMPLETED: initialized with %d keypoints and %d corresponding landmarks\n", ...
    size(State.Keypoints, 2), size(State.Landmarks,2));

end