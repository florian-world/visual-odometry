degAdd=5;
degAdd=degtorad(degAdd);
pxDeg=floor(degAdd/(2*atan(K(1,3)/(2*K(1,1))))/K(1,3));
keyF=0;
avgD=0

lenD=length(p_W_landmarks);
for i=1:lenD
    D=norm(p_W_landmarks(:,i)+t_C_W');
    avgD=avgD+D;
end
keyF=norm(t_C_W)/(avgD/lenD);

%last pixel position
ppos=[px,py];
%from here
nx=(px-(K(1,3)/2))*pxDeg;
ny=(py-(K(2,3)/2))*pxDeg;
nvec=[nx ny 0];
%fristRcw rcw at first obeservation
nrot=firstRcw*rotationVectorToMatrix(nvec);
%to here only first observation

%dist from dists from maatcDescriptors.m
dist=dist+dist;
%candidate array
candArr={firstRcw,nvec,nrot,ppos,dist};

%to execute at keyframe
for i=length(candArr)
    
if dist(quaternion(nrot,'rotmat','frame'),quaternion(R_C_W,'rotmat','frame'))>degAdd
   %add to totriangulate
end
end