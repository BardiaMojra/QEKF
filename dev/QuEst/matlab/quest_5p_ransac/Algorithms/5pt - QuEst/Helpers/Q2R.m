%% Convert quternion q = [q0 q1 q2 q3]^T into the equivalent rotation matrix R
function R = Q2R(Q)


numInp = size(Q,2);
R = zeros(3,3,numInp);

for i = 1 : numInp
    q = Q(:,i);

    if q(1) < 0
        q = -q;
    end

    R(:,:,i) = [ 1-2*q(3)^2-2*q(4)^2    2*q(2)*q(3)-2*q(4)*q(1)    2*q(2)*q(4)+2*q(3)*q(1);
             2*q(2)*q(3)+2*q(4)*q(1)        1-2*q(2)^2-2*q(4)^2    2*q(3)*q(4)-2*q(2)*q(1);
             2*q(2)*q(4)-2*q(3)*q(1)    2*q(3)*q(4)+2*q(2)*q(1)        1-2*q(2)^2-2*q(3)^2];
end

end




















































