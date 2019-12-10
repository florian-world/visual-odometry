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
%           CandidateDescriptors:        [256xM] array containing M descriptors of candidate keypoints 
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
corners2 = detectHarrisFeatures(Im2,'ROI',roi);

% Describe Keypoints
Desc1 = describeKeyPoints(Im1,corners1.Location(:,1),corners1.Location(:,2));
Desc2 = describeKeyPoints(Im2,corners2.Location(:,1),corners2.Location(:,2));

% Match keypoints by calculating BRIEF descriptors and matching
[Match1, Match2, Idx1, Idx2] = matchDescriptors(Desc1,Desc2,corners1.Location,corners2.Location);
Descriptors = Desc2(:,Idx2);

% Estimate relative pose between initial frames and create 3D pointcloud
% Check if det(F) = 0, if not correct as in Ex. 6 (Simon)
F = estimateFundamentalMatrix(Match1, Match2, 'Method', ...
                              'RANSAC', 'NumTrials', 2000);
                 
% Recover essential matrix from F, then decompose into R,T
E = K'*F*K;
[Rots,u3] = decomposeEssentialMatrix(E);
Match1(:,3)=1; % TODO: These must be the homogeneous coords of the matches
Match2(:,3)=1; % TODO: These must be the homogeneous coords of the matches
[R,T] = disambiguateRelativePose(Rots,u3,Match1',Match2',K,K);

% Triangulate points to create pointcloud
M1 = K * eye(3,4);
M2 = K * [R, T];
X_hom = linearTriangulation(Match1',Match2',M1,M2); % Output of this is homogenous
Landmarks = X_hom(1:3,:);

Keypoints = Match2(:,1:2)'; %TODO: Check dimension, also of X                      
  
State.Keypoints = Keypoints;
State.Landmarks = Landmarks;
State.Descriptors = Descriptors;
% These state entries are not calculated during bootstrapping
State.CandidateKeypoints = [];
State.CandidateDescriptors = [];
State.InitCandidateKeypoints = [];
State.InitCandidatePoses = [];

end