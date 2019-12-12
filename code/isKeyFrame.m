function isKF =isKeyFrame(landmarks)
    %ISKEYFRAME determines if current frame is a keyframe
    %   
    %   Input:
    %       landmarks:    [3xK] array containing K landmark coordinates
    %       curTranslation:    [3x1] Translation from origin
    %
    %   Output:
    %       isKF: bool if current frame should be treated as a new keyframe

    global MAGIC_KEYFRAME_THRESHOLD KEYFRAME_TRANSLATION TOT_TRANSLATION

    avgD=0;
    lenD=length(landmarks);
    for i=1:lenD
        D=norm(landmarks(:,i)-TOT_TRANSLATION);
        avgD=avgD+D;
    end
    kdist=norm(KEYFRAME_TRANSLATION-TOT_TRANSLATION);
    keyF=kdist/(avgD/lenD)
    isKF=false;
    if keyF>MAGIC_KEYFRAME_THRESHOLD
        isKF=true;
        KEYFRAME_TRANSLATION=TOT_TRANSLATION;
    end
end