function [curState,curPose] = processFrame(prevState,image)
% main VO implemented here:
%           - always performs localization using the landmarks and
%           descriptors from the last keyframe and matching them with the
%           newly computed ones of the current frame
%           - adds and handles candiate keypoints
%           - if image is used as a keyframe, new features are triangulated
%           and added to the landmarks
%   Input:
%       prevState, containing:
%           Keypoints:                   [2xK] array containing K keypoint pairs
%           Landmarks:                   [3xK] array containing K landmark coordinates
%           Descriptors:                 [256xK] array containing K descriptors of previous frame
%           CandidateKeypoints:          [2xM] array containing M candidate keypoints
%           CandidateDescriptors:        [256xM] array containing M descriptors of candidate keypoints 
%           InitCandidateKeypoints:      [2xM] array containing M initial observations of
%                                         candidate keypoints
%           InitCandidatePoses:          [12xM] array containing M initial camera poses of first
%                                         observation of candidate keypoints
%       image: [height x width] current frame
%
%   Output:
%       curState,curPose


global K PATCHRADIUS


%% TODO: this section is duplicated code (bootstrap does the same) unify? or at least simplify

[height, width] = size(image);
% Detect corners on both frames
roi = [PATCHRADIUS+1,PATCHRADIUS+1,width-(2*PATCHRADIUS),height-(2*PATCHRADIUS)];
corners2 = detectHarrisFeatures(image,'ROI',roi);

Desc2 = describeKeyPoints(image,corners2.Location(:,1),corners2.Location(:,2));

% Match keypoints by calculating BRIEF descriptors and matching
[Match1, Match2, ~, Idx2] = matchDescriptors(prevState.Descriptors,Desc2,prevState.Keypoints',corners2.Location);
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

Keypoints = Match2(:,1:2)';

%% Track candidate keypoints

curPose = [R T];
curState = prevState;

% Check if there are any candidate keypoints yet (only empty directly after
% keyframe)
if isempty(prevState.CandidateKeypoints)
    curState.CandidateKeypoints = corners2.Location(~Idx2,:)';
    curState.CandidateDescriptors = Desc2(:,~Idx2);
    curState.InitCandidateKeypoints = corners2.Location(~Idx2,:)';
    curState.InitCandidatePoses = repmat(curPose(:),1,sum(~Idx2));
else
    % Track candidate keypoints and update candidate list
    DescNoMatch = Desc2(:,~Idx2);
    PointsNoMatch = corners2.Location(~Idx2,:);
    [~, ~, CandIdx1, CandIdx2] = matchDescriptors(prevState.CandidateDescriptors,DescNoMatch,prevState.CandidateKeypoints',PointsNoMatch);
    % First only keep candidates that were succesfully tracked and update
    % non-init components
    newCandidateKeypoints = PointsNoMatch(CandIdx2,:)';
    newCandidateDescriptors = DescNoMatch(:,CandIdx2);
    newInitCandidateKeypoints = prevState.InitCandidateKeypoints(:,CandIdx1);
    newInitCandidatePoses = prevState.InitCandidatePoses(:,CandIdx1);
    % Then append new candidates
    newCandidateKeypoints = [newCandidateKeypoints PointsNoMatch(~CandIdx2,:)'];
    newCandidateDescriptors = [newCandidateDescriptors DescNoMatch(:,~CandIdx2)];
    newInitCandidateKeypoints = [newInitCandidateKeypoints PointsNoMatch(~CandIdx2,:)'];
    newInitCandidatePoses = [newInitCandidatePoses repmat(curPose(:),1,sum(~CandIdx2))];
    % Write updated candidates to state
    curState.CandidateKeypoints = newCandidateKeypoints;
    curState.CandidateDescriptors = newCandidateDescriptors;
    curState.InitCandidateKeypoints = newInitCandidateKeypoints;
    curState.InitCandidatePoses = newInitCandidatePoses;
end


%% Keyframe detection and triangulation of new landmarks
%
% TODO: add code for checking if this frame is a keyframe (+ triangulation
%       --> new landmarks)

keyframeDetected = true;

if (keyframeDetected)
    curState.Keypoints = Keypoints;
    curState.Descriptors = Descriptors;
end


%% Plot

global COLOR_CANDIDATE COLOR_LANDMARK
scatter(Keypoints(1, :), Keypoints(2, :), 60, COLOR_LANDMARK, 'x', 'LineWidth', 3);
scatter(curState.CandidateKeypoints(1, :), curState.CandidateKeypoints(2, :), 10, COLOR_CANDIDATE, 'filled');



end

