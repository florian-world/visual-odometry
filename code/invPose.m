function [inverse] = invPose(pose)

assert(all(size(pose) == [3 4]));

R = pose(:,1:3);
t = pose(:,4);

inverse = [R', -t];

end

