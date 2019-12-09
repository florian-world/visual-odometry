%degAdd=5;

function pxDeg=px2Deg(K,degAdd=5)
degAdd=degtorad(degAdd);
pxDeg=floor(degAdd/(2*atan(K(1,3)/(2*K(1,1))))/K(1,3));
end

function [keyF,Tresh]=magicKeyF(p_W_landmarks,t_C_W,tresh=0.1,degAdd=0.0873)
keyF=0;
avgD=0
lenD=length(p_W_landmarks);
for i=1:lenD
    D=norm(p_W_landmarks(:,i)+t_C_W');
    avgD=avgD+D;
end
keyF=norm(t_C_W)/(avgD/lenD);
Tresh=false

    if keyF<tresh
        Tresh=true
    end
end 
function nvec=UVtoRotVec(px,py,K,pxDegX,pxDegY=pxDegX)
nx=(px-(K(1,3)/2))*pxDegX;
ny=(py-(K(2,3)/2))*pxDegY;
nvec=[nx ny 0];
end
%%fristRcw rcw at first obeservation
%nrot=firstRcw*rotationVectorToMatrix(nvec);
%to here only first observation

%%dist from dists from maatcDescriptors.m
%dist=dist+dist;
%%candidate array
%candArr={firstRcw,nvec,nrot,ppos,dist};
%%alternatively only iter number instead of first R
%candArr{firstRCW, nvec, ppos}
%to execute at keyframe
function newP=triangNewKPoint(firstRcw,nvec,R_C_W,keypoints)
comp=zeros(length(firstRcw);
for i=length(firstRcw)
    %nrot=firstRcw*rotationVectorToMatrix(nvec);
    nrot=firstRcw[i]*rotationVectorToMatrix(nvec);
if dist(quaternion(nrot,'rotmat','frame'),quaternion(R_C_W,'rotmat','frame'))>degAdd
    comp[i]=1;
   %add to totriangulate
end
end
correspondCandidate=keypoints(:,comp>0);
validCand=firstRcw(:,comp>0);
pOld=nvec;
pOld(3,:)=1;
correspondCandidate = [correspondCandidate;ones(1,length(correspondCandidate))];
newP=linearTriangulationMVar(pOld,correspondCandidate,validCand,R_C_W);
end
%p_W_landmarks=union(p_W_landmarks,newP);
    