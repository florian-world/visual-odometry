function [candidateMask] = filterGoodCandidates(initCandidateKPs, curCandidateKPs, initCandidatePoses, curPose)
%FILTERGOODCANDIDATES Filters candidates satisfying the angle condition
%   Detailed explanation goes here

global MAGIC_KEYFRAME_ANGLE_RAD

curR = curPose(:,1:3);

N = size(initCandidateKPs, 2);

candidateMask = false(1, N);

for i=1:N
    initKP = initCandidateKPs(:,i);
    curKP = curCandidateKPs(:,i);
    initPose = reshape(initCandidatePoses(:,i),[3,4]);
    
    isSamePose = all(abs(initPose - curPose) <= eps,'all');
    if isSamePose
        continue;
    end
    
    initR = initPose(:,1:3);
    
    v1_hom = [initKP(1) initKP(2) 1]';
    v2_hom = [curKP(1) curKP(2) 1]';
    
    % rotate back to world frame and check angle
    v1_W = initR * v1_hom;
    v2_W = curR * v2_hom;
    
    % angle btw. two vectors see https://www.mathworks.com/matlabcentral/answers/16243-angle-between-two-vectors-in-3d
    angle = atan2(norm(cross(v1_W,v2_W)), dot(v1_W,v2_W));
    
    
    candidateMask(i) = angle >= MAGIC_KEYFRAME_ANGLE_RAD;
    
    %% DEBUG OUTPUT
    
%     fprintf("Observed candidate [%2.1f %2.1f] first in \n", initKP(1), initKP(2));
%     initPose
%     fprintf("and it is now at [%2.1f %2.1f] in \n", curKP(1), curKP(2));
%     curPose
%     
%     fprintf("Showing an angle of %.1f degree, ", rad2deg(angle));
%     if candidateMask(i)
%         fprintf("therefore selected.\n");
%     else
%         fprintf("not selected.\n");
%     end
end

fprintf("Selected %d out of %d candidates using the %.0f degree angle condition\n", ...
    nnz(candidateMask), length(candidateMask), rad2deg(MAGIC_KEYFRAME_ANGLE_RAD));

end

