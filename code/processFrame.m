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
%           InitCandidateKeypoints:      [2xM] array containing M initial observations of
%                                         candidate keypoints
%           InitCandidatePoses:          [12xM] array containing M initial camera poses of first
%                                         observation of candidate keypoints
%       prev_image: [height x width] previous frame
%       image: [height x width] current frame
%
%   Output:
%       curState,curPose


global K PATCHRADIUS FRAME_NUM KEYFRAME_TRANSLATION TOT_TRANSLATION KEYFRAME_THRESHOLD


%% TODO: this section is duplicated code (bootstrap does the same) unify? or at least simplify

[height, width] = size(image);
% Detect corners on both frames
roi = [PATCHRADIUS+1,PATCHRADIUS+1,width-(2*PATCHRADIUS),height-(2*PATCHRADIUS)];
newCorners = detectHarrisFeatures(image,'ROI',roi);

% KLT
pointTracker = vision.PointTracker('MaxBidirectionalError',1); % Set to Inf for speedup
initialize(pointTracker,prevState.Keypoints',prev_image);

size(prevState.Keypoints,2)
[trackedPoints,trackedPointsValidity] = pointTracker(image);
KLTMatch1 = prevState.Keypoints(:,trackedPointsValidity)';
KLTMatch2 = trackedPoints(trackedPointsValidity,:);
                          
F_KLT = estimateFundamentalMatrix(KLTMatch1, KLTMatch2, 'Method', ...
                              'RANSAC', 'NumTrials', 2000);
                 
% Recover essential matrix from F, then decompose into R,T
E = K'*F_KLT*K;
[Rots,u3] = decomposeEssentialMatrix(E);
KLTMatch1(:,3)=1;
KLTMatch2(:,3)=1;
[R,T] = disambiguateRelativePose(Rots,u3,KLTMatch1',KLTMatch2',K,K);

Keypoints = KLTMatch2(:,1:2)';

%% Track candidate keypoints

curPose = [R T];
curState = prevState;

curState.Keypoints = Keypoints;
curState.Landmarks = prevState.Landmarks(:,trackedPointsValidity);

PointsNoMatch = trackedPoints(~trackedPointsValidity,:);
% Remove any negative points from PointsNoMatch
posIdxs = all(PointsNoMatch > 0, 2);
PointsNoMatch = PointsNoMatch(posIdxs,:);

% Check if there are any candidate keypoints yet (only empty directly after
% keyframe)
if isempty(prevState.CandidateKeypoints)
    % Add newly detected corners that aren't duplicates
    newCornersMask = findNewCandidates(newCorners.Location,PointsNoMatch);
    newDetectedCorners = newCorners.Location(newCornersMask,:);
    curState.CandidateKeypoints = [PointsNoMatch' newDetectedCorners'];
    curState.InitCandidateKeypoints = [PointsNoMatch' newDetectedCorners'];
    curState.InitCandidatePoses = repmat(curPose(:),1,size(PointsNoMatch,1)+size(newDetectedCorners,1));
else
    % Track candidate keypoints and update candidate list
    candPointTracker = vision.PointTracker('MaxBidirectionalError',1); % Set to Inf for speedup
    initialize(candPointTracker,prevState.CandidateKeypoints',prev_image);
    [trackedCandPoints,trackedCandPointsValidity] = candPointTracker(image);
    uniqueNewIdxs = findNewCandidates(PointsNoMatch,trackedCandPoints);
    
    % First only keep candidates that were succesfully tracked and update
    % non-init components
    newCandidateKeypoints = trackedCandPoints(trackedCandPointsValidity,:)';
    newInitCandidateKeypoints = prevState.InitCandidateKeypoints(:,trackedCandPointsValidity);
    newInitCandidatePoses = prevState.InitCandidatePoses(:,trackedCandPointsValidity);
    % Then append new candidates
    newCornersMask = findNewCandidates(newCorners.Location,[PointsNoMatch(uniqueNewIdxs,:); newCandidateKeypoints']);
    newDetectedCorners = newCorners.Location(newCornersMask,:);
    newCandidateKeypoints = [newCandidateKeypoints PointsNoMatch(uniqueNewIdxs,:)' newDetectedCorners'];
    newInitCandidateKeypoints = [newInitCandidateKeypoints PointsNoMatch(uniqueNewIdxs,:)' newDetectedCorners'];
    newInitCandidatePoses = [newInitCandidatePoses repmat(curPose(:),1,sum(uniqueNewIdxs)+sum(newCornersMask))];
    % Check if any candidates are already actual keypoints
    noKeypointsMask = findNewCandidates(newCandidateKeypoints',curState.Keypoints');
    newCandidateKeypoints = newCandidateKeypoints(:,noKeypointsMask);
    newInitCandidateKeypoints = newInitCandidateKeypoints(:,noKeypointsMask);
    newInitCandidatePoses = newInitCandidatePoses(:,noKeypointsMask);
    % Write updated candidates to state
    curState.CandidateKeypoints = newCandidateKeypoints;
    curState.InitCandidateKeypoints = newInitCandidateKeypoints;
    curState.InitCandidatePoses = newInitCandidatePoses; %does this overwrite also init pose of all canditate or only the new?
end


%% Keyframe detection and triangulation of new landmarks
%
% TODO: add code for checking if this frame is a keyframe (+ triangulation
%       --> new landmarks)

%keyframeDetected = true;
TOT_TRANSLATION=TOT_TRANSLATION+T;
totRot=norm(rotationMatrixToVector(R))
if (size(prevState.Keypoints,2)<KEYFRAME_THRESHOLD) || (totRot>0.03) && (size(prevState.Keypoints,2)<KEYFRAME_THRESHOLD*2.5)
    disp('Keyframeframe');
    curState.Keypoints = Keypoints;
    [comp, nvec] = triangNewKPoint(curState,R);
    if ~all(comp == 0)
        correspondCandidate=curState.CandidateKeypoints(:,comp>0);
        validCand=curState.InitCandidatePoses(:,comp>0);
        nvec=nvec(:,comp>0);
        nvec(3,:)=1;
        correspondCandidate1 = [correspondCandidate;ones(1,size(correspondCandidate,2))];
        newLand=zeros(4,length(validCand));
        for jj=1:size(validCand,2)
            candPose=reshape(validCand(:,jj),[3,4]);
            newLand(:,jj)=linearTriangulation(nvec(:,jj),correspondCandidate1(:,jj),candPose,curPose);
        end
        %eliminate triangulated from candidate
        curState.InitCandidatePoses=curState.InitCandidatePoses(:,comp>0);
        curState.CandidateKeypoints=curState.CandidateKeypoints(:,comp>0);
        curState.InitCandidateKeypoints=curState.InitCandidateKeypoints(:,comp>0);
        %curState.InitCandidatePoses=curState.InitCandidatePoses(:,comp>0); alerady cut??
        curState.Landmarks=[curState.Landmarks,newLand(1:3,:)];
        curState.Keypoints=[curState.Keypoints,correspondCandidate];
    end
end


%% Plot

global COLOR_CANDIDATE COLOR_LANDMARK
scatter(Keypoints(1, :), Keypoints(2, :), 60, COLOR_LANDMARK, 'x', 'LineWidth', 3);
scatter(curState.CandidateKeypoints(1, :), curState.CandidateKeypoints(2, :), 10, COLOR_CANDIDATE, 'filled');



end

