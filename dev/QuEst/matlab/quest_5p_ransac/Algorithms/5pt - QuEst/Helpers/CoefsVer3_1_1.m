%% Five Point Pose Estimation using Quaternion 
%
% Fast way of generating the coefficient matrix C from the image points.
%
%
% Inputs:
%        m1:   Matrix containing the homogeneous coordinates of  
%              feature points in the 1st camera frame.
% 
%        m2:   Matrix containing the homogeneous coordinates of 
%              feature points in the 2nd camera frame.
%
%
% Output:
%        C: The 11*35 coefficient matrix
%
%
% Copyright (C) 2013-2017, by Kaveh Fathian.
%
% ------------------------------------------------------------------------
%
% Ver 3_1_1:
%           - Based on Ver3_1, whithout the unit norm coefficient vector
%             appended at the end. 
%
%%
function C = CoefsVer3_1_1(m1,m2) % #codegen

% Number of feature points
numPts = size(m1,2);

idxBin1 = zeros(2,nchoosek(numPts,2)-1);
counter = 0;
for i = 1 : numPts-2
   for j = i+1 : numPts
       counter = counter + 1;
       idxBin1(:,counter) = [i;j];     
   end
end


mx1 = m1(1,idxBin1(1,:)).';    my1 = m1(2,idxBin1(1,:)).';
nx1 = m2(1,idxBin1(1,:)).';    ny1 = m2(2,idxBin1(1,:)).';
mx2 = m1(1,idxBin1(2,:)).';    my2 = m1(2,idxBin1(2,:)).';
nx2 = m2(1,idxBin1(2,:)).';    ny2 = m2(2,idxBin1(2,:)).';

s1  = m1(3,idxBin1(1,:)).';      r1  = m2(3,idxBin1(1,:)).'; 
s2  = m1(3,idxBin1(2,:)).';      r2  = m2(3,idxBin1(2,:)).';


% coefsN = num1
coefsN = coefsNumVer2_0(mx1,mx2,my1,my2,nx2,ny2,r2,s1,s2);
% coefsD = den1
coefsD = coefsDenVer2_0(mx2,my2,nx1,nx2,ny1,ny2,r1,r2,s2);


%%
% Total number of equations
numEq = nchoosek(numPts,3);

idxBin2 = zeros(2,numEq);
counter = 0;
counter2 = 0;
for i = numPts-1 : -1 : 2
    
    for j = (1+counter2) : (i-1+counter2)
        for k = (j+1) : (i+counter2)
            counter = counter + 1;
            idxBin2(:,counter) = [j;k]; 
        end
    end
    counter2 = i  + counter2;
 
end

% ai = [num1;
%       den1];
a1  = [coefsN(idxBin2(1,:),1); coefsD(idxBin2(1,:),1)];    
a2  = [coefsN(idxBin2(1,:),2); coefsD(idxBin2(1,:),2)];
a3  = [coefsN(idxBin2(1,:),3); coefsD(idxBin2(1,:),3)];
a4  = [coefsN(idxBin2(1,:),4); coefsD(idxBin2(1,:),4)];
a5  = [coefsN(idxBin2(1,:),5); coefsD(idxBin2(1,:),5)];
a6  = [coefsN(idxBin2(1,:),6); coefsD(idxBin2(1,:),6)];
a7  = [coefsN(idxBin2(1,:),7); coefsD(idxBin2(1,:),7)];
a8  = [coefsN(idxBin2(1,:),8); coefsD(idxBin2(1,:),8)];
a9  = [coefsN(idxBin2(1,:),9); coefsD(idxBin2(1,:),9)];
a10 = [coefsN(idxBin2(1,:),10); coefsD(idxBin2(1,:),10)];

% bi = [num2;
%       den2];
b1  = [coefsD(idxBin2(2,:),1); coefsN(idxBin2(2,:),1)];
b2  = [coefsD(idxBin2(2,:),2); coefsN(idxBin2(2,:),2)];
b3  = [coefsD(idxBin2(2,:),3); coefsN(idxBin2(2,:),3)];
b4  = [coefsD(idxBin2(2,:),4); coefsN(idxBin2(2,:),4)];
b5  = [coefsD(idxBin2(2,:),5); coefsN(idxBin2(2,:),5)];
b6  = [coefsD(idxBin2(2,:),6); coefsN(idxBin2(2,:),6)];
b7  = [coefsD(idxBin2(2,:),7); coefsN(idxBin2(2,:),7)];
b8  = [coefsD(idxBin2(2,:),8); coefsN(idxBin2(2,:),8)];
b9  = [coefsD(idxBin2(2,:),9); coefsN(idxBin2(2,:),9)];
b10 = [coefsD(idxBin2(2,:),10); coefsN(idxBin2(2,:),10)];

% coefsND = [num1 * den2;
%            den1 * num2];
coefsND = coefsNumDen(a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,...
    b1,b2,b3,b4,b5,b6,b7,b8,b9,b10);

%% Matrix of all coefficients

% coefs = (num1 * den2)  -  (den1 * num2)
C = coefsND(1:numEq,:) - coefsND(numEq+1:2*numEq,:);


end



%%
function coefsN = coefsNumVer2_0(mx1,mx2,my1,my2,nx2,ny2,r2,s1,s2)


t2 = mx1.*my2.*r2;
t3 = mx2.*ny2.*s1;
t4 = my1.*nx2.*s2;
t5 = mx1.*nx2.*s2.*2.0;
t6 = my1.*ny2.*s2.*2.0;
t7 = mx1.*my2.*nx2.*2.0;
t8 = my2.*r2.*s1.*2.0;
t9 = mx2.*my1.*r2;
t10 = mx1.*ny2.*s2;
t11 = mx2.*my1.*ny2.*2.0;
t12 = mx2.*r2.*s1.*2.0;
t13 = my2.*nx2.*s1;
coefsN = [t2+t3+t4-mx2.*my1.*r2-mx1.*ny2.*s2-my2.*nx2.*s1,t11+t12-mx1.*my2.*ny2.*2.0-mx1.*r2.*s2.*2.0,t7+t8-mx2.*my1.*nx2.*2.0-my1.*r2.*s2.*2.0,t5+t6-mx2.*nx2.*s1.*2.0-my2.*ny2.*s1.*2.0,-t2-t3+t4+t9+t10-my2.*nx2.*s1,-t5+t6+mx2.*nx2.*s1.*2.0-my2.*ny2.*s1.*2.0,t7-t8-mx2.*my1.*nx2.*2.0+my1.*r2.*s2.*2.0,-t2+t3-t4+t9-t10+t13,-t11+t12+mx1.*my2.*ny2.*2.0-mx1.*r2.*s2.*2.0,t2-t3-t4-t9+t10+t13];


end


%%
function coefsD = coefsDenVer2_0(mx2,my2,nx1,nx2,ny1,ny2,r1,r2,s2)


t2 = mx2.*ny1.*r2;
t3 = my2.*nx2.*r1;
t4 = nx1.*ny2.*s2;
t5 = mx2.*nx2.*r1.*2.0;
t6 = my2.*ny2.*r1.*2.0;
t7 = mx2.*nx2.*ny1.*2.0;
t8 = ny1.*r2.*s2.*2.0;
t9 = my2.*nx1.*r2;
t10 = nx2.*ny1.*s2;
t11 = my2.*nx1.*ny2.*2.0;
t12 = nx1.*r2.*s2.*2.0;
t13 = mx2.*ny2.*r1;
coefsD = [t2+t3+t4-mx2.*ny2.*r1-my2.*nx1.*r2-nx2.*ny1.*s2,t11+t12-my2.*nx2.*ny1.*2.0-nx2.*r1.*s2.*2.0,t7+t8-mx2.*nx1.*ny2.*2.0-ny2.*r1.*s2.*2.0,t5+t6-mx2.*nx1.*r2.*2.0-my2.*ny1.*r2.*2.0,t2-t3-t4+t9+t10-mx2.*ny2.*r1,t5-t6-mx2.*nx1.*r2.*2.0+my2.*ny1.*r2.*2.0,-t7+t8+mx2.*nx1.*ny2.*2.0-ny2.*r1.*s2.*2.0,-t2+t3-t4-t9+t10+t13,t11-t12-my2.*nx2.*ny1.*2.0+nx2.*r1.*s2.*2.0,-t2-t3+t4+t9-t10+t13];

end


%%
function coefsND = coefsNumDen(a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,b1,b2,b3,b4,b5,b6,b7,b8,b9,b10)



coefsND = [a1.*b1,a1.*b2+a2.*b1,a2.*b2+a1.*b5+a5.*b1,a2.*b5+a5.*b2,a5.*b5,a1.*b3+a3.*b1,a2.*b3+a3.*b2+a1.*b6+a6.*b1,a2.*b6+a3.*b5+a5.*b3+a6.*b2,a5.*b6+a6.*b5,a3.*b3+a1.*b8+a8.*b1,a3.*b6+a6.*b3+a2.*b8+a8.*b2,a6.*b6+a5.*b8+a8.*b5,a3.*b8+a8.*b3,a6.*b8+a8.*b6,a8.*b8,a1.*b4+a4.*b1,a2.*b4+a4.*b2+a1.*b7+a7.*b1,a2.*b7+a4.*b5+a5.*b4+a7.*b2,a5.*b7+a7.*b5,a3.*b4+a4.*b3+a1.*b9+a9.*b1,a3.*b7+a4.*b6+a6.*b4+a7.*b3+a2.*b9+a9.*b2,a6.*b7+a7.*b6+a5.*b9+a9.*b5,a3.*b9+a4.*b8+a8.*b4+a9.*b3,a6.*b9+a7.*b8+a8.*b7+a9.*b6,a8.*b9+a9.*b8,a4.*b4+a1.*b10+a10.*b1,a4.*b7+a7.*b4+a2.*b10+a10.*b2,a7.*b7+a5.*b10+a10.*b5,a3.*b10+a4.*b9+a9.*b4+a10.*b3,a6.*b10+a7.*b9+a9.*b7+a10.*b6,a8.*b10+a9.*b9+a10.*b8,a4.*b10+a10.*b4,a7.*b10+a10.*b7,a9.*b10+a10.*b9,a10.*b10];


end










