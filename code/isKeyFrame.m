function isKF =isKeyFrame(landmarks,curTranslation)
%ISKEYFRAME determines if current frame is a keyframe
%   
%   Input:
%       landmarks:    [3xK] array containing K landmark coordinates
%       curTranslation:    [3x1] Translation from origin
%
%   Output:
%       isKF: bool if current frame should be treated as a new keyframe

global MAGIC_KEYFRAME_THRESHOLD KEYFRAME_TRANSLATION

avgD=0;
lenD=length(landmarks);
for i=1:lenD
    D=pdist2(landmarks(:,i),curTranslation','euclidean');
    avgD=avgD+D;
end
kdist=pdist2(KEYFRAME_TRANSLATION,curTranslation,'euclidean');
keyF=kdist/(avgD/lenD);
isKF=false;
    if keyF<MAGIC_KEYFRAME_THRESHOLD
        isKF=true;
        KEYFRAME_TRANSLATION=curTranslation;
    end
end 

function pxDeg=px2Deg(K,degAdd=5)
degAdd=degtorad(degAdd);
pxDeg=floor(degAdd/(2*atan(K(1,3)/(2*K(1,1))))/K(1,3));
end


function nvec=UVtoRotVec(UV,K,pxDeg)
nx=(UV(1)-(K(1,3)/2))*pxDeg;
ny=(UV(2)-(K(2,3)/2))*pxDeg;
nvec=[nx ny 0];
end