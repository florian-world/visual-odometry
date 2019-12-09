function [curState,curPose] = processFrame(prevState,image)
% main VO implemented here
% prevState contains:
%           Keypoints:                   [2xK] array containing K keypoint pairs
%           Landmarks:                   [3xK] array containing K landmark coordinates
%           Descriptors:                 [256xK] array containing K descriptors of previous frame
%           CandidateKeypoints:          [2xM] array containing M candidate keypoints
%           InitCandidateKeypoints:      [2xM] array containing M initial observations of
%                                         candidate keypoints
%           InitCandidatePoses:          [12xM] array containing M initial camera poses of first
%                                         observation of candidate keypoints


global K PATCHRADIUS


curState = prevState;
curPose = eye(3,4);

% TODO

[height, width] = size(image);
% Detect corners on both frames
roi = [PATCHRADIUS+1,PATCHRADIUS+1,width-(2*PATCHRADIUS),height-(2*PATCHRADIUS)];
corners2 = detectHarrisFeatures(image,'ROI',roi);

Desc2 = describeKeyPoints(image,corners2.Location(:,1),corners2.Location(:,2));

% Match keypoints by calculating BRIEF descriptors and matching
% TODO: implement (Mambo)
[Match1, Match2, Descriptors] = matchDescriptors(prevState.Descriptors,Desc2,prevState.Keypoints',corners2.Location);

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


Keypoints = Match2(:,1:2)'; %TODO: Check dimension, also of X                      

curPose = [R T];
curState.Keypoints = Keypoints;
curState.Descriptors = Descriptors;

end

