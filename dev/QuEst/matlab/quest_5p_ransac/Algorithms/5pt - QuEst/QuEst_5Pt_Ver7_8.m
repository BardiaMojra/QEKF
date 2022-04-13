%% QuEst (Quaternion Estimation) algorithm for 5 feature points
%
% NOTE: Include the folder "Helpers" in Matlab path before execution.
%
%
% Inputs:
%
% m, n:    Homogeneous coordinates of N feature points in the first  
%          and second coordinate frames. Each column of m or n has the 
%          format [x, y, 1]^T, where x and y are coordinates of the  
%          feature point on the image plane. Thus, m and n are 3*N matrices, 
%          with one entries in the 3rd row.
%
%
% Outputs:
%
% Q   :  The recovered rotation in quaternion. 
%
%
% Copyright (C) 2013-2017, by Kaveh Fathian.
%
% This program is a free software: you can redistribute it and/or modify it
% under the terms of the GNU lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or any 
% later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details  
% <http://www.gnu.org/licenses/>.
%
% ------------------------------------------------------------------------
%
% Ver 7_8:
%           -  Uses all matrices B1,B2,B3 to find solutions
%           -  Slightly slower, but has better accuracy 
%
%%
function [Q] = QuEst_5Pt_Ver7_8(m,n,i)
%% Preallocate variables

Idx = [ 1     2     5    11    21     3     6    12    22     8    14    24    17    27    31     4     7    13    23     9    15    25    18    28    32    10    16    26    19    29    33    20    30    34    35;
        2     5    11    21    36     6    12    22    37    14    24    39    27    42    46     7    13    23    38    15    25    40    28    43    47    16    26    41    29    44    48    30    45    49    50;
        3     6    12    22    37     8    14    24    39    17    27    42    31    46    51     9    15    25    40    18    28    43    32    47    52    19    29    44    33    48    53    34    49    54    55;
        4     7    13    23    38     9    15    25    40    18    28    43    32    47    52    10    16    26    41    19    29    44    33    48    53    20    30    45    34    49    54    35    50    55    56];


%% Construct coefficient matrix
% Coefficinet matrix in the linearized system of multinomials (Cf * V = 0)
% Cf = CoefsVer3_1_1(kp1, kp2);
Cf = CoefsVer3_1_1(m,n);
% Cf = CoefsVer3_1_1_mex(m,n);

numEq = size(Cf,1);
% A is the coefficient matrix such that A * X = 0
A = zeros(4*numEq,56);
for i = 1 : 4
    idx = Idx(i,:);
    A((i-1)*numEq+1 : i*numEq, idx) = Cf;
end

path = '../../mout/nbug_QuEst_A_matlab.txt';
writematrix(A,path,'Delimiter',' ');

% Find bases for the null space of A
[~,~,V] = svd(A,0);
N = V(:,37:56);

idx = Idx(1,:);   A0 = N(idx,:);
idx = Idx(2,:);   A1 = N(idx,:);
idx = Idx(3,:);   A2 = N(idx,:);
idx = Idx(4,:);   A3 = N(idx,:);

B = A0 \ [A1, A2, A3];

B1 = B(:,1:20);
B2 = B(:,21:40);
B3 = B(:,41:60);


%% Find eigenvectors

% Initial guess for the common eigenvectors
[V1, ~] = eig(B1); 
[V2, ~] = eig(B2);
[V3, ~] = eig(B3);

Ve = [V1, V2, V3];
% #todo for now remove all the imaginary solutions 
% Remove duplicate complex eigenvectors
Vy = imag(Ve);
imagIdx = sum(abs(Vy),1) > 10*eps;
Viall = Ve(:,imagIdx);
[~,srtIdx] = sort(real(Viall(1,:)),'ascend');
Vi = Viall(:,srtIdx(1:2:end)); % Keep only one complex eigenvector
Vr = Ve(:,~imagIdx);
V0 = real([Vi, Vr]);           % Use only the real parts


%% Extract quaternion elements

% Degree 5 monomial solution vectors
X5 = N * V0;

% Correct the sign of each column s.t. the first element (i.e., w^5) is always positive
X5 = bsxfun(@times, sign(X5(1,:)), X5);

% Recover quaternion elements  
w = nthroot(X5(1,:),5);
w4 = w.^4;
x = X5(2,:) ./ w4;
y = X5(3,:) ./ w4;
z = X5(4,:) ./ w4;

Q = [w;
     x;
     y;
     z];


% Normalize s.t. each column of Q has norm 1
QNrm = sqrt(sum(Q.^2,1));
Q = bsxfun(@rdivide, Q, QNrm);


