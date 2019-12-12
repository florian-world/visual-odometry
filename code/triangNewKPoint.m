function [comp, nvec] =triangNewKPoint(curState,R)
    %Determine valid candidates and returns a mask
    global MAGIC_KEYFRAME_ANGLE_RAD
    le=length(curState.CandidateKeypoints);
    comp=zeros(1,le);
    nvec=zeros(3,le);
    for i=1:le
        nvec(:,i)=uvToRotVec(curState.CandidateKeypoints(:,i))';
        %nrot=firstRcw*rotationVectorToMatrix(nvec);
        candPose=reshape(curState.InitCandidatePoses(:,i),[3,4]);
        candRot=candPose(:,1:3);
        nrot=candRot*rotationVectorToMatrix(nvec(:,i));
        if dist(quaternion(nrot,'rotmat','frame'),quaternion(R,'rotmat','frame'))>MAGIC_KEYFRAME_ANGLE_RAD
            comp(i)=1;
           %add to totriangulate
        end
    end
end
    