function nvec = uvToRotVec(UV)

global DEG_TO_PIX K
    nx=(UV(1)-(K(1,3)/2))*DEG_TO_PIX;
    ny=(UV(2)-(K(2,3)/2))*DEG_TO_PIX;
    nvec=[nx ny 0];
end

