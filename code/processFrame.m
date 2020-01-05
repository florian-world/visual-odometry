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

[trackedPoints,trackedPointsValidity] = pointTracker(image);
KLTMatch2 = trackedPoints(trackedPointsValidity,:);
Landmarks = prevState.Landmarks(:,trackedPointsValidity);
                          
% run p3p
[R,T, inlierIdx] = ransacLocalizationP3P(KLTMatch2',Landmarks,K);


if (numel(R) == 0)
    error("Tracking lost!");
end

% equivalent matlab call:
% [R, T,inlierIdx] = estimateWorldCameraPose(double(KLTMatch2),Landmarks',cameraParameters("IntrinsicMatrix", K'), "MaxNumTrials", 2000);
% T = T';

Keypoints = KLTMatch2(:,1:2)';

curPose = invPose([R T]);
curState = prevState;

R = curPose(:,1:3);
T = curPose(:,4);

fprintf("Pos estimate: (%3.1f, %3.1f, %3.1f)       localized with %.1f%% inliers in %d keypoints\n", T(1), T(2), T(3), nnz(inlierIdx)/length(inlierIdx)*100, length(inlierIdx));

% Ignore out of sight landmarks...
landmarksLocal = getLandmarksInLocalFrame(curPose, Landmarks);
    
mask = landmarksLocal(3,:)>0; % ignore negative z values

if nnz(~mask) > 0
    fprintf("Ignoring %d landmarks with now negative z coordinate \n", nnz(~mask));
end

Keypoints = Keypoints(:,mask);
Landmarks = Landmarks(:,mask);

curState.Keypoints = Keypoints;
curState.Landmarks = Landmarks;


%% Track candidate keypoints

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
    numberOfNewCandidates = size(PointsNoMatch,1) + size(newDetectedCorners, 1);
else
    % Track candidate keypoints and update candidate list
    candPointTracker = vision.PointTracker('MaxBidirectionalError',1); % Set to Inf for speedup
    initialize(candPointTracker,prevState.CandidateKeypoints',prev_image);
    [trackedCandPoints,trackedCandPointsValidity] = candPointTracker(image);
    uniqueNewIdxs = findNewCandidates(PointsNoMatch,trackedCandPoints);
    
    numberOfNewCandidates = nnz(uniqueNewIdxs);
    
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

fprintf("Tracking %d candidates, +%d newly added.\n", size(curState.CandidateKeypoints,2), numberOfNewCandidates);

xlabel(sprintf("%3d landmarks, %3d candidates, %3.1f%% inliers", size(curState.Landmarks,2), size(curState.CandidateKeypoints,2), nnz(inlierIdx)/length(inlierIdx)*100));


%% Keyframe detection and triangulation of new landmarks
%

%keyframeDetected = true;
% totRot=norm(rotationMatrixToVector(R));
if (isKeyFrame(curState, curPose) || size(curState.Landmarks,2)<150 )
    curState.LastKeyframePose = curPose;
    curState.Keypoints = Keypoints;
%     [candidateMask, ~] = triangNewKPoint(curState,R);
    candidateMask = filterGoodCandidates(curState.InitCandidateKeypoints, ...
        curState.CandidateKeypoints, curState.InitCandidatePoses, curPose);
    
    if any(candidateMask)
        selectedCandKPs=curState.CandidateKeypoints(:,candidateMask);
        selectedCandInitKPs=curState.InitCandidateKeypoints(:,candidateMask);
        selectedCandInitPoses=curState.InitCandidatePoses(:,candidateMask);
        selectedCandKPs(3,:) = 1;
        selectedCandInitKPs(3,:) = 1;
        newLandmarks=zeros(4,size(selectedCandInitPoses,2));
        for jj=1:size(selectedCandInitPoses,2)
            candPose=reshape(selectedCandInitPoses(:,jj),[3,4]);
%             fprintf("Triangulating new keypoint, first observed at [%4.1f %4.1f] with pose\n", selectedCandInitKPs(1,jj), selectedCandInitKPs(2,jj));
%             candPose
%             fprintf("and now in [%4.1f %4.1f] at pose:",selectedCandKPs(1,jj), selectedCandKPs(2,jj));
%             curPose
            
            newLandmarks(:,jj)=linearTriangulation(selectedCandInitKPs(:,jj),selectedCandKPs(:,jj),K*invPose(candPose),K*invPose(curPose));
            isSamePose = all(abs(candPose - curPose) <= eps,'all');
            assert(~isSamePose, "Trying to triangulate keyframe frome same pose, this should never happen\n");
        end
        %newLandmarks=-newLandmarks;
        % TODO this is not correct local z might be pointing in any direction
        %inSightMask = newLandmarks(3,:) > 0;
        %newLandmarks = newLandmarks(1:3, inSightMask);
%         rotLandmarks=zeros(3,size(newLandmarks,2));
%         for l=1:size(newLandmarks,2)
%             rotLandmarks(:,l)=curPose(:,1:3)*newLandmarks(1:3,l);
%         end
        newLandmarksLocal = getLandmarksInLocalFrame(curPose,newLandmarks);
        inSightMask = newLandmarksLocal(3,:) > 0;
        
        fprintf("Ignoring %d candidate landmarks which are not in sight\n", nnz(~inSightMask));
%         newLandmarks = newLandmarks(1:3, inSightMask);
        
%       % FOR REFERENCE: this is how it was decided to use the localization with all new keypoints:
%
%         [Rnew,Tnew, inlierIdxNew] = ransacLocalizationP3P(selectedCandKPs(1:2,inSightMask),newLandmarks(1:3,:),K);
%         fprintf("Newly localization from selected candidate keypoints yielded [%2.1f %2.1f %2.1f] with %2.1f%% inliers\n", -Rnew'*Tnew, nnz(inlierIdxNew)/length(inlierIdxNew)*100);
%         
%         [Rnew,Tnew, inlierIdxNewNew] = ransacLocalizationP3P([curState.Keypoints,selectedCandKPs(1:2,inSightMask)],[curState.Landmarks,newLandmarks(1:3,:)],K);
%         s1 = size(curState.Keypoints, 2);
%         s2 = size(selectedCandKPs(1:2,inSightMask), 2);
%         tmpMask = [repmat(false, 1, s1), repmat(true,1,s2)];
%         relNumInlier = nnz(inlierIdxNewNew & tmpMask);
%         fprintf("Newly localization with all new keypoints yielded [%2.1f %2.1f %2.1f] with %2.1f%% inliers, and %2.1f%% on the new candidates.\n", -Rnew'*Tnew, nnz(inlierIdxNewNew)/length(inlierIdxNewNew)*100, relNumInlier/s2*100);
        
        
        [~,~, newInlierMask] = ransacLocalizationP3P([curState.Keypoints,selectedCandKPs(1:2,:)],[curState.Landmarks,newLandmarks(1:3,:)],K);
        s1 = size(curState.Keypoints, 2);
        newInliers = newInlierMask((s1+1):end);
        
        fprintf("Only keeping %d out of %d candidate landmarks being inliers on newly RANSAC p3p localization\n", nnz(newInliers), length(newInliers));
        
        inSightMask = inSightMask & newInliers;
        newLandmarks = newLandmarks(1:3, inSightMask);
%         relNumInlier = nnz(inlierIdxNewNew & tmpMask);
%         fprintf("Newly localization with all new keypoints yielded [%2.1f %2.1f %2.1f] with %2.1f%% inliers, and %2.1f%% on the new candidates.\n", -Rnew'*Tnew, nnz(inlierIdxNewNew)/length(inlierIdxNewNew)*100, relNumInlier/s2*100);
        
        %eliminate triangulated from candidate
        curState.InitCandidatePoses=curState.InitCandidatePoses(:,~candidateMask);
        curState.CandidateKeypoints=curState.CandidateKeypoints(:,~candidateMask);
        curState.InitCandidateKeypoints=curState.InitCandidateKeypoints(:,~candidateMask);
        curState.Landmarks=[curState.Landmarks,newLandmarks(1:3,:)];
        curState.Keypoints=[curState.Keypoints,selectedCandKPs(1:2,inSightMask)];
        
        if size(newLandmarks,2) > 0
            fprintf("%d landmarks added, pos of last new landmark: (%.2f, %.2f, %.2f)\n", ...
                size(newLandmarks,2), newLandmarks(1:3,end));
        else
            fprintf("No landmarks added in this keyframe\n");
        end
    end
end

%% Plot

global COLOR_CANDIDATE COLOR_LANDMARK
scatter(Keypoints(1, :), Keypoints(2, :), 60, COLOR_LANDMARK, 'x', 'LineWidth', 3);
scatter(curState.CandidateKeypoints(1, :), curState.CandidateKeypoints(2, :), 10, COLOR_CANDIDATE, 'filled');


s1 = size(curState.CandidateKeypoints,2);
s2 = size(curState.InitCandidateKeypoints,2);
s3 = size(curState.InitCandidatePoses,2);

assert(s1 == s2);
assert(s1 == s3);

s1 = size(curState.Landmarks,2);
s2 = size(curState.Keypoints,2);

assert(s1 == s2);


end