function [qbin,idx] = QuatMult(qbin1,qbin2)

siz1 = size(qbin1,2);
siz2 = size(qbin2,2);
idx = zeros(2,siz1*siz2);

qbin = zeros(4,siz1*siz2);

counter = 0;
for i = 1 : siz1
    q1 = qbin1(:,i);
    for j = 1 : siz2
        q2 = qbin2(:,j);
        q = Mult(q1,q2);
        counter = counter +1;
        qbin(:,counter) = q;
        idx(1,counter) = i;
        idx(2,counter) = j;
    end
end

end


function q = Mult(q1,q2)
% q = q1 * q2

Q = [q1(1) -q1(2) -q1(3) -q1(4)
     q1(2)  q1(1) -q1(4)  q1(3)
     q1(3)  q1(4)  q1(1) -q1(2)
     q1(4) -q1(3)  q1(2)  q1(1)];
 
P = [q2(1);q2(2);q2(3);q2(4)];


q = Q * P;

end