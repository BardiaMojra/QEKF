%% Recover translation and depths given the rotation and feature point coordinates
%
% NOTE: Include the folder "Helpers" in Matlab path before execution.
%
%
% Inputs:
%
% m, n:    Homogeneous coordinates of matched feature points in the first  
%          and second coordinate frames. Each column of m or n has the 
%          format [x, y, 1]^T, where x and y are coordinates of the  
%          feature point on the image plane. 
%
% Q:       Recovered solutions for rotation, either in quaternion form or
%          rotation matrix form. For quaternions, Q is a 4*k matrix, where
%          k is the number of solutions. For rotations, Q is a 3*3*k
%          structure. 
%
%
% Outputs:
%
%
% T   :  Associated translation vectors. 
%
% Z1  :  Depths in the first camera frame.
%
% Z2  :  Depths in the second camera frame.
%
% R   :  Rotation matrix representation.
%
%
% NOTE: T, Z1, and Z2 are recovered up to a common scale factor.
% Copyright (C) 2017, by Kaveh Fathian.
%
function [T, Z1, Z2, R, Res] = FindTransDepth_Ver1_0(m,n, Q)

% If input is quaternion transform it into the rotation matrix
if size(Q,1) ~= 3
    R = Q2R(Q);
else 
    R = Q;
end

% Preallocate variables
I       = eye(3);           % Identity matrix
numPts  = size(m,2);        % Number of feature points
numInp  = size(R,3);        % Number of rotation matrices in input
T       = zeros(3,numInp);       % Translation vector
Z1      = zeros(numPts,numInp);  % Depth of points in the 1st camera frame
Z2      = zeros(numPts,numInp);  % Depth of points in the 2nd camera frame
Res     = zeros(1,numInp);       % Residue from SVD


for k = 1 : numInp
    % Stack rigid motion constraints into matrix-vector form C * Y = 0
    C = zeros(3*numPts, 2*numPts+3);
    for i = 1 : numPts
      a1 = (i-1)*3+1;
      a2 = i*3;
      C(a1:a2, 1:3) = I;
      a1 = (i-1)*3+1;
      a2 = i*3;
      b1 = (i-1)*2+4;
      b2 = (i-1)*2+5;
      A = R(:,:,k)*m(:,i);
      B = -n(:,i);
      C(a1:a2,b1:b2) = [A,B];
    end

    % Use SVD to find singular vectors
    [~,S,N] = svd(C,0);
    
    % Singular values
    Sd = diag(S);
    
    % The right singular vector corresponding to the zero singular value of C.
    Y = N(:,end);    

    t = Y(1:3)  ;  % Translation vector
    z = Y(4:end);  % Depths in both camera frames

    % Adjust the sign s.t. the recovered depths are positive
    numPos = sum(z > 0);
    numNeg = sum(z < 0);
    if numPos < numNeg
        t = -t;
        z = -z;
    end

    z1 =  z(1 : 2 : end-1); % Depths in camera frame 1
    z2 =  z(2 : 2 : end);   % Depths in camera frame 2

    % Store the results
    T(:,k) = t;
    Z1(:,k) = z1;
    Z2(:,k) = z2;
    Res(:,k) = Sd(end);
end










































































