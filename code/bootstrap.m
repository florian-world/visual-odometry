function State = bootstrap(Im1,Im2)
%BOOTSTRAP Takes two initial frames as input and returns the first state of
%          the initialization to be used by the continuos VO pipeline. 
%   
%   Input:
%       Im1:    First bootstrap frame
%       Im2:    Second bootstrap frame
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

imSize = size(Im1);
height = imSize(1);
width = imSize(2);
% Detect corners on both frames
roi = [PATCHRADIUS+1,PATCHRADIUS+1,width-(2*PATCHRADIUS),height-(2*PATCHRADIUS)];
corners1 = detectHarrisFeatures(Im1,'ROI',roi);

% Point tracking using KLT
pointTracker = vision.PointTracker('MaxBidirectionalError',1); % Set to Inf for speedup
initialize(pointTracker,corners1.Location,Im1);

[trackedPoints,trackedPointsValidity] = pointTracker(Im2);
KLTMatch1 = corners1.Location(trackedPointsValidity,:);
KLTMatch2 = trackedPoints(trackedPointsValidity,:);

[F_KLT, inliersIdx] = estimateFundamentalMatrix(KLTMatch1, KLTMatch2, 'Method', ...
                              'RANSAC', 'NumTrials', 15000);
                          
Inliers1 = KLTMatch1(inliersIdx,:);
Inliers2 = KLTMatch1(inliersIdx,:);
                 
% Recover essential matrix from F, then decompose into R,T
E = K'*F_KLT*K;
[Rots,u3] = decomposeEssentialMatrix(E);
Inliers1_hom = flipud(Inliers1');
Inliers1_hom(3,:) = 1;
Inliers2_hom = flipud(Inliers2');
Inliers2_hom(3,:) = 1;
[R,T] = disambiguateRelativePose(Rots,u3,Inliers1_hom,Inliers2_hom,K,K);

% Triangulate points to create pointcloud
M1 = K * eye(3,4);
M2 = K * [R', -T];
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