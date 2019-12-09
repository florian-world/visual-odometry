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
%           Descriptors:                 [256xK] array containing K descriptors of previous frame
%           CandidateKeypoints:          [2xM] array containing M candidate keypoints
%           InitCandidateKeypoints:      [2xM] array containing M initial observations of
%                                         candidate keypoints
%           InitCandidatePoses:          [12xM] array containing M initial camera poses of first
%                                         observation of candidate keypoints
global K

% Detect corners on both frames
% TODO: select only certain number of corners based on params (Simon)
corners1 = detectHarrisFeatures(Im1);
corners2 = detectHarrisFeatures(Im2);

% Match keypoints by calculating BRIEF descriptors and matching
% TODO: implement (Mambo)
matches = matchDescribeCorners(corners1.Location, corners2.Location);

% Estimate relative pose between initial frames and create 3D pointcloud
% Check if det(F) = 0, if not correct as in Ex. 6 (Simon)
F = estimateFundamentalMatrix(matches(1), matches(2), 'Method', ...
                              'RANSAC', 'NumTrials', 2000);
                 
% Recover essential matrix from F, then decompose into R,T
E = K'*F*K;
[Rots,u3] = decomposeEssentialMatrix(E);
p1 = matches(1); % TODO: These must be the homogeneous coords of the matches
p2 = matches(2); % TODO: These must be the homogeneous coords of the matches
[R,T] = disambiguateRelativePose(Rots,u3,p1,p2,K,K);

% Triangulate points to create pointcloud
M1 = K * eye(3,4);
M2 = K * [R, T];
X_hom = linearTriangulation(p1,p2,M1,M2); % Output of this is homogenous
Landmarks = X_hom(1:3,:);

Keypoints = matches(2); %TODO: Check dimension, also of X                      
  
State.Keypoints = Keypoints;
State.Landmarks = Landmarks;
State.Decriptors = Descriptors;
% These state entries are not calculated during bootstrapping
State.CandidateKeypoints = [];
State.InitCandidateKeypoints = [];
State.InitCadidatePoses = [];

end

