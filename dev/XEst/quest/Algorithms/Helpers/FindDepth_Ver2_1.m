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
% QPval: Residual value from optimization
%
%
% NOTE: T, Z1, and Z2 are recovered up to a common scale factor.
% Copyright (C) 2017, by Kaveh Fathian.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Ver 2_1:
%           - Based on Ver2_0, but use a different ordering for C
%
% Ver 2_0:
%           - Uses power method to recover depth vector
%             and enforce positive depth constraint
%
%
function [Z1, Z2, R, Res, Z] = FindDepth_Ver2_1(m,n, Q)

% If input is quaternion transform it into the rotation matrix
if size(Q,1) ~= 3
    R = Q2R(Q);
else 
    R = Q;
end

% Preallocate variables
numPts  = size(m,2);        % Number of feature points
numInp  = size(R,3);        % Number of rotation matrices in input
Z1      = zeros(numPts,numInp);    % Depth of points in the 1st camera frame
Z2      = zeros(numPts,numInp);    % Depth of points in the 2nd camera frame
Z       = zeros(2*numPts,numInp);  % All depths 
Res     = zeros(1,numInp);         % Residue of the eigenvector

for k = 1 : numInp
    % Stack rigid motion constraints into matrix-vector form C * Y = 0
    C = zeros(3*(numPts-1), 2*numPts);
    for i = 1 : numPts-1
        C((i-1)*3+1:i*3, (i-1)*2+1:(i-1)*2+2) = [R(:,:,k)*m(:,i), -n(:,i)];
        C((i-1)*3+1:i*3, (i-1)*2+3:(i-1)*2+4) = -[R(:,:,k)*m(:,i+1), -n(:,i+1)];
    end

    % Use power method to find depths (s.t. they are positive)
    [z, res] = PowerMethod(C.'*C);
    
    z1 =  z(1 : 2 : end-1); % Depths in camera frame 1
    z2 =  z(2 : 2 : end);   % Depths in camera frame 2        

    % Save the results
    Z1(:,k) = z1;
    Z2(:,k) = z2;
    Z(:,k) = z;
    Res(k) = res;
end

end

%%
% Power method to recover eigenvector associated with the zero eigenvalue
function [xn, res] = PowerMethod(A)

N = size(A,2);                  
xn = rand(N,1);                 % Random initial guess
thresh = 1e-4;                  % Threshold to stop
dif = thresh + 1;
posVal = 0.1;                   % Positive value used to enforce positive depth constraint
maxItr = 10;                    % Maximum number of iterations
itr = 0;

I = eye(N);
c = 1e-13;

B = A + c * I;                  % To avoid inverting a singular matrix
% Bi = pinv(B);

while (dif > thresh) && (itr < maxItr)
    
    xnp = B \ xn; 
%     xnp = Bi * xn;
        
    xnp = xnp ./ max(abs(xnp)); % Normalize to have one L_inf norm
    xnp(xnp<0) = posVal;        % Force negative elements to be positive
    
    dif = sum(abs(xn - xnp));   % See how much xn has changed from previous step
    xn = xnp;                   % Update xn
    itr = itr + 1;
    
end

res = sum(abs(A * xn));

end

















