function [landmarks_local] = getLandmarksInLocalFrame(curPose,landmarks)
%getLandmarksInLocalFrame Simply applies the proper coordinate
%transformation
%   INPUT:
%           curPose     3x4
%           landmarks   3xK
%   OUTPUT:
%           landmarks_local

landmarks(4,:) = 1;

Tf_C_W = invPose(curPose); % maps pts from world to current camera frame

landmarks_local = Tf_C_W * landmarks;

end

