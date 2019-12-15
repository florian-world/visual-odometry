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

F_KLT = estimateFundamentalMatrix(KLTMatch1, KLTMatch2, 'Method', ...
                              'RANSAC', 'NumTrials', 2000);
                 
% Recover essential matrix from F, then decompose into R,T
E = K'*F_KLT*K;
[Rots,u3] = decomposeEssentialMatrix(E);
KLTMatch1(:,3)=1;
KLTMatch2(:,3)=1;
[R,T] = disambiguateRelativePose(Rots,u3,KLTMatch1',KLTMatch2',K,K);

% Triangulate points to create pointcloud
M1 = K * eye(3,4);
M2 = K * [R, T];
X_hom = linearTriangulation(KLTMatch1',KLTMatch2',M1,M2); % Output of this is homogenous
Landmarks = X_hom(1:3,:);

Keypoints = KLTMatch2(:,1:2)';                     
  
State.Keypoints = Keypoints;
State.Landmarks = Landmarks;
% These state entries are not calculated during bootstrapping
State.CandidateKeypoints = [];
State.InitCandidateKeypoints = [];
State.InitCandidatePoses = [];

end