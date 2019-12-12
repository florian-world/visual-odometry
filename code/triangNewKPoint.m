function comp=triangNewKPoint(curState)
%Determine valid candidates and returns a mask
global MAGIC_KEYFRAME_ANGLE_RAD
le=length(curState.CandidateKeypoints);
comp=zeros(1,le);
for i=length(le)
    nvec=UVtoRotVec(curState.CandidateKeypoints(i,:));
    %nrot=firstRcw*rotationVectorToMatrix(nvec);
    nrot=curState.InitCandidatePoses(i)*rotationVectorToMatrix(nvec);
if dist(quaternion(nrot,'rotmat','frame'),quaternion(R_C_W,'rotmat','frame'))>MAGIC_KEYFRAME_ANGLE_RAD
    comp(i)=1;
   %add to totriangulate
end
end
end
%p_W_landmarks=union(p_W_landmarks,newP);
    