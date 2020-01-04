function [comp, nvec] =triangNewKPoint(curState,R)
    %Determine valid candidates and returns a mask
    global MAGIC_KEYFRAME_ANGLE_RAD
    minNewKeyPoints=50;
    le=length(curState.CandidateKeypoints);
    comp=false(1,le);
    nvec=zeros(3,le);
    %nP=0;
    distK=zeros(1,le);
    for i=1:le
        nvec(:,i)=uvToRotVec(curState.CandidateKeypoints(:,i))';
        %nrot=firstRcw*rotationVectorToMatrix(nvec);
        candPose=reshape(curState.InitCandidatePoses(:,i),[3,4]);
        initNvec=uvToRotVec(curState.InitCandidateKeypoints(:,i))';
        candRot=candPose(:,1:3);
        initNrot=candRot*rotationVectorToMatrix(initNvec);
        nrot=R*rotationVectorToMatrix(nvec(:,i));
        distK(i)=dist(quaternion(nrot,'rotmat','frame'),quaternion(initNrot,'rotmat','frame'));
        %if dist(quaternion(nrot,'rotmat','frame'),quaternion(initNrot,'rotmat','frame'))>MAGIC_KEYFRAME_ANGLE_RAD
            %comp(i)=true;
            %nP=nP+1;
           %add to totriangulate
        %end
    end
    co=distK(distK>MAGIC_KEYFRAME_ANGLE_RAD);
    if(length(co)>minNewKeyPoints)
        comp(distK>MAGIC_KEYFRAME_ANGLE_RAD)=true;
    else
        [~,sI]=sort(distK);
        comp(sI<=minNewKeyPoints)=true;
        comp(distK==0)=false;
    end
end
