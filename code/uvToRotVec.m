function nvec = uvToRotVec(UV)

global PIX_TO_RAD K
    nx=(UV(1)-(K(1,3)/2))*PIX_TO_RAD;
    ny=(UV(2)-(K(2,3)/2))*PIX_TO_RAD;
    nvec=[nx ny 0];
end

