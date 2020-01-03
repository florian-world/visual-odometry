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


global K PATCHRADIUS KEYFRAME_THRESHOLD


[height, width] = size(image);
% Detect corners on both frames
roi = [PATCHRADIUS+1,PATCHRADIUS+1,width-(2*PATCHRADIUS),height-(2*PATCHRADIUS)];
newCorners = detectHarrisFeatures(image,'ROI',roi);

% KLT
pointTracker = vision.PointTracker('MaxBidirectionalError',1); % Set to Inf for speedup
initialize(pointTracker,prevState.Keypoints',prev_image);

% size(prevState.Keypoints,2);
[trackedPoints,trackedPointsValidity] = pointTracker(image);
% KLTMatch1 = prevState.Keypoints(:,trackedPointsValidity)';
KLTMatch2 = trackedPoints(trackedPointsValidity,:);
Landmarks = prevState.Landmarks(:,trackedPointsValidity);
                          
% F_KLT = estimateFundamentalMatrix(KLTMatch1, KLTMatch2, 'Method', ...
%                               'RANSAC', 'NumTrials', 2000);
%                  
% % Recover essential matrix from F, then decompose into R,T
% E = K'*F_KLT*K;
% [Rots,u3] = decomposeEssentialMatrix(E);
% KLTMatch1(:,3)=1;
% KLTMatch2(:,3)=1;
% [R,T] = disambiguateRelativePose(Rots,u3,KLTMatch1',KLTMatch2',K,K);

% run p3p
[R,T, inlierIdx] = ransacLocalizationP3P(KLTMatch2',Landmarks,K);

R = R';
T = -T;

% equivalent matlab call:
% [R, T,inlierIdx] = estimateWorldCameraPose(double(KLTMatch2),Landmarks',cameraParameters("IntrinsicMatrix", K'), "MaxNumTrials", 2000);
% T = T';

Keypoints = KLTMatch2(:,1:2)';

%% Track candidate keypoints

curPose = [R T];
curState = prevState;

fprintf("Pos estimate: (%3.1f, %3.1f, %3.1f)       localized with %.1f%% inliers in %d keypoints\n", T(1), T(2), T(3), nnz(inlierIdx)/length(inlierIdx)*100, length(inlierIdx));

curState.Keypoints = Keypoints;
curState.Landmarks = Landmarks;

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

%keyframeDetected = true;
% totRot=norm(rotationMatrixToVector(R));
if (isKeyFrame(curState, curPose))
    curState.LastKeyframePose = curPose;
    curState.Keypoints = Keypoints;
    [candidateMask, ~] = triangNewKPoint(curState,R);
    if any(candidateMask)
        selectedCandKPs=curState.CandidateKeypoints(:,candidateMask);
        selectedCandInitKPs=curState.InitCandidateKeypoints(:,candidateMask);
        selectedCandInitPoses=curState.InitCandidatePoses(:,candidateMask);
        selectedCandKPs(3,:) = 1;
        selectedCandInitKPs(3,:) = 1;
        newLandmarks=zeros(4,length(selectedCandInitPoses));
        for jj=1:size(selectedCandInitPoses,2)
            candPose=reshape(selectedCandInitPoses(:,jj),[3,4]);
%             fprintf("Triangulating new keypoint, first observed at [%4.1f %4.1f] with pose\n", selectedCandInitKPs(1,jj), selectedCandInitKPs(2,jj));
%             candPose
%             fprintf("and now in [%4.1f %4.1f] at pose:",selectedCandKPs(1,jj), selectedCandKPs(2,jj));
%             curPose
            isSamePose = all(abs(candPose - curPose) <= eps,'all');
            newLandmarks(:,jj)=linearTriangulation(selectedCandInitKPs(:,jj),selectedCandKPs(:,jj),K'*invPose(candPose),K'*invPose(curPose));
            if (isSamePose)
                fprintf("Trying to triangulate keyframe frome same pose, this should never happen\n"); 
            end
        end
        %newLandmarks=-newLandmarks;
        % TODO this is not correct local z might be pointing in any direction
        %inSightMask = newLandmarks(3,:) > 0;
        %newLandmarks = newLandmarks(1:3, inSightMask);
        rotLandmarks=zeros(3,size(newLandmarks,2));
        for l=1:size(newLandmarks,2)
            rotLandmarks(:,l)=curPose(:,1:3)'*newLandmarks(1:3,l);
        end
         inSightMask = rotLandmarks(3,:) > curPose(3,4);
         newLandmarks = newLandmarks(1:3, inSightMask);
        
        %eliminate triangulated from candidate
        curState.InitCandidatePoses=curState.InitCandidatePoses(:,~candidateMask);
        curState.CandidateKeypoints=curState.CandidateKeypoints(:,~candidateMask);
        curState.InitCandidateKeypoints=curState.InitCandidateKeypoints(:,~candidateMask);
        curState.Landmarks=[curState.Landmarks,newLandmarks(1:3,:)];
        curState.Keypoints=[curState.Keypoints,selectedCandKPs(1:2,inSightMask)];
        %xlabel(sprintf("%d landmarks added, pos of last new landmark: (%.2f, %.2f, %.2f)", ...
            %size(newLandmarks,2), newLandmarks(1:3,end)));
        
    end
end

%% Plot

global COLOR_CANDIDATE COLOR_LANDMARK
scatter(Keypoints(1, :), Keypoints(2, :), 60, COLOR_LANDMARK, 'x', 'LineWidth', 3);
scatter(curState.CandidateKeypoints(1, :), curState.CandidateKeypoints(2, :), 10, COLOR_CANDIDATE, 'filled');

end