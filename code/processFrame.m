function [curState,curPose] = processFrame(prevState,prev_image,image)
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
%       prev_image: [height x width] previous frame
%       image: [height x width] current frame
%
%   Output:
%       curState,curPose


global K PATCHRADIUS FRAME_NUM FIRST_KEYFRAME KEYFRAME_TRANSLATION TOT_TRANSLATION


%% TODO: this section is duplicated code (bootstrap does the same) unify? or at least simplify

[height, width] = size(image);
% Detect corners on both frames
roi = [PATCHRADIUS+1,PATCHRADIUS+1,width-(2*PATCHRADIUS),height-(2*PATCHRADIUS)];
corners2 = detectHarrisFeatures(image,'ROI',roi);

Desc2 = describeKeyPoints(image,corners2.Location(:,1),corners2.Location(:,2));

% Match keypoints by calculating BRIEF descriptors and matching
[Match1, Match2, ~, Idx2] = matchDescriptors(prevState.Descriptors,Desc2,prevState.Keypoints',corners2.Location);
Descriptors = Desc2(:,Idx2);

% KLT
pointTracker = vision.PointTracker('MaxBidirectionalError',1); % Set to Inf for speedup
initialize(pointTracker,prevState.Keypoints',prev_image);

[trackedPoints,trackedPointsValidity] = pointTracker(image);
KLTMatch1 = prevState.Keypoints(:,trackedPointsValidity)';
KLTMatch2 = trackedPoints(trackedPointsValidity,:);

% Estimate relative pose between initial frames and create 3D pointcloud
% Check if det(F) = 0, if not correct as in Ex. 6 (Simon)
% F = estimateFundamentalMatrix(Match1, Match2, 'Method', ...
%                               'RANSAC', 'NumTrials', 2000);
                          
F_KLT = estimateFundamentalMatrix(KLTMatch1, KLTMatch2, 'Method', ...
                              'RANSAC', 'NumTrials', 2000);
                 
% Recover essential matrix from F, then decompose into R,T
E = K'*F_KLT*K;
[Rots,u3] = decomposeEssentialMatrix(E);
KLTMatch1(:,3)=1; % TODO: These must be the homogeneous coords of the matches
KLTMatch2(:,3)=1; % TODO: These must be the homogeneous coords of the matches
[R,T] = disambiguateRelativePose(Rots,u3,KLTMatch1',KLTMatch2',K,K);

Keypoints = KLTMatch2(:,1:2)';

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
    curState.InitCandidatePoses = newInitCandidatePoses; %does this overwrite also init pose of all canditate or only the new?
end


%% Keyframe detection and triangulation of new landmarks
%
% TODO: add code for checking if this frame is a keyframe (+ triangulation
%       --> new landmarks)

%keyframeDetected = true;
TOT_TRANSLATION=TOT_TRANSLATION+T;
if (isKeyFrame(curState.Landmarks)) || (FRAME_NUM == FIRST_KEYFRAME)
    disp('Keyframeframe');
    curState.Keypoints = Keypoints;
    curState.Descriptors = Descriptors;
    [comp, nvec] = triangNewKPoint(curState,R);
    if ~all(comp == 0)
        correspondCandidate=curState.CandidateKeypoints(:,comp>0);
        correspondCandidateDescriptors=curState.CandidateDescriptors(:,comp>0);
        validCand=curState.InitCandidatePoses(:,comp>0);
        nvec=nvec(:,comp>0);
        nvec(3,:)=1;
        correspondCandidate1 = [correspondCandidate;ones(1,length(correspondCandidate))];
        newLand=zeros(4,length(validCand));
        for jj=1:length(validCand)
            candPose=reshape(validCand(:,jj),[3,4]);
            newLand(:,jj)=linearTriangulation(nvec(:,jj),correspondCandidate1(:,jj),candPose,curPose);
        end
        %eliminate triangulated from candidate
        curState.InitCandidatePoses=curState.InitCandidatePoses(:,comp>0);
        curState.CandidateKeypoints=curState.CandidateKeypoints(:,comp>0);
        curState.CandidateDescriptors=curState.CandidateDescriptors(:,comp>0);
        curState.InitCandidateKeypoints=curState.InitCandidateKeypoints(:,comp>0);
        %curState.InitCandidatePoses=curState.InitCandidatePoses(:,comp>0); alerady cut??
        curState.Landmarks=[curState.Landmarks,newLand(1:3,:)];
        curState.Keypoints=[curState.Keypoints,correspondCandidate];
        curState.Descriptors=[curState.Descriptors,correspondCandidateDescriptors];
    end
end


%% Plot

global COLOR_CANDIDATE COLOR_LANDMARK
scatter(Keypoints(1, :), Keypoints(2, :), 60, COLOR_LANDMARK, 'x', 'LineWidth', 3);
scatter(curState.CandidateKeypoints(1, :), curState.CandidateKeypoints(2, :), 10, COLOR_CANDIDATE, 'filled');



end

