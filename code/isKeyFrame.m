function isKF =isKeyFrame(curState, curPose)
    %ISKEYFRAME determines if current frame is a keyframe
    %   
    %   Input:
    %       curState:     usual state struct, to access Landmarks 3xK
    %       curPose:      [3x1] Translation from origin
    %
    %   Output:
    %       isKF: bool if current frame should be treated as a new keyframe

    global MAGIC_KEYFRAME_THRESHOLD
    
    pts_W = curState.Landmarks; % 3xK
    pts_W(4,:) = 1;
    
    R_W_C = curPose(:,1:3);
    t_W_C = curPose(:,4);
    
    Tf_C_W = [R_W_C', -t_W_C]; % maps pts from world to current camera frame
    
    pts_C = Tf_C_W * pts_W;
    
    mask = pts_C(3,:)>0; % ignore negative z values
    
%     fprintf("ignoring %d landmarks behind\n", nnz(~mask));
    
    pts_C = pts_C(:,mask);
    
    averageDepth = mean(pts_C(3,:));
    keyframeDistance = sqrt(sum(curPose(:,4).^2 - curState.LastKeyframePose(:,4).^2));
    
    isKF = keyframeDistance / averageDepth > MAGIC_KEYFRAME_THRESHOLD;
    
    fprintf("ISKEYFRAME? Average depth = %.2f, keyframe distance = %.2f, %.2f compared against %.2f --> %s\n", ...
        averageDepth, keyframeDistance, keyframeDistance/averageDepth, MAGIC_KEYFRAME_THRESHOLD, mat2str(isKF));
    
% FORMER APPROACH: (for reference)
%     avgD=0;
%     lenD=length(landmarks);
%     for i=1:lenD
%         D=norm(landmarks(:,i)-TOT_TRANSLATION);
%         avgD=avgD+D;
%     end
%     kdist=norm(KEYFRAME_TRANSLATION-TOT_TRANSLATION);
%     keyF=kdist/(avgD/lenD);
%     isKF=false;
%     if keyF>MAGIC_KEYFRAME_THRESHOLD
%         isKF=true;
%         KEYFRAME_TRANSLATION=TOT_TRANSLATION;
%     end
end