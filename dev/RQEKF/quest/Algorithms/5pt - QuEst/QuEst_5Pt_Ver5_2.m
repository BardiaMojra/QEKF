%% QuEst (Quaternion Estimation) algorithm for 5 feature points
%
% NOTE: Include the folder "Helpers" in Matlab path before execution.
%
%
% Inputs:
%
% m, n:    Homogeneous coordinates of 5 matched feature points in the first  
%          and second coordinate frames. Each column of m or n has the 
%          format [x, y, 1]^T, where x and y are coordinates of the  
%          feature point on the image plane. Thus, m and n are 3*5 matrices, 
%          with one entries in the 3rd row.
%
%
% Outputs:
%
% q   :  The recovered rotation in quaternion. 
%
% t   :  Associated translation vectors. 
%
% z1  :  Depths in the first camera frame.
%
% z2  :  Depths in the second camera frame.
%
% r   :  Rotation matrix representation of the recovered rotation.
%
%
% NOTE: t, z1, and z2 are recovered up to a common scale factor.
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
% Ver 5_2:
%           Exact implementation of the algorithm explained in paper 
%           "QuEst: A Quaternion-Based Approach for Camera Motion 
%           Estimation From Minimal Feature Points", IEEE ROBOTICS AND 
%           AUTOMATION LETTERS
%
function [Q] = QuEst_5Pt_Ver5_2(m,n)

% Preallocate variables
% 
% Let 
%       V:   vector of all degree 4 monomials (with 35 entries)
%       X:   vector of all degree 5 monomials (with 56 entries)
% in variables w,x,y,z. 
% Rows of Idx respectively show the index of the entries of vectors w*V, 
% x*V, y*V, and z*V, in the vector X.
%
Idx = [ 1     2     5    11    21     3     6    12    22     8    14    24    17    27    31     4     7    13    23     9    15    25    18    28    32    10    16    26    19    29    33    20    30    34    35;
        2     5    11    21    36     6    12    22    37    14    24    39    27    42    46     7    13    23    38    15    25    40    28    43    47    16    26    41    29    44    48    30    45    49    50;
        3     6    12    22    37     8    14    24    39    17    27    42    31    46    51     9    15    25    40    18    28    43    32    47    52    19    29    44    33    48    53    34    49    54    55;
        4     7    13    23    38     9    15    25    40    18    28    43    32    47    52    10    16    26    41    19    29    44    33    48    53    20    30    45    34    49    54    35    50    55    56];

    
% Index of columns of A corresponding to all monomials with a least one power of w
idx_w = (1 : 35);
% Index of the rest of the columns (monomials with no power of w)
idx_w0 = (36 : 56);

% First column of Idx1 shows the row index of matrix B. The second, 
% third, and fourth columns indicate the column index of B which should  
% contain a 1 element respectively for xV, yV, and zV.
% V = [w^4, w^3*x, w^3*y, w^3*z, w^2*x^2, w^2*x*y, w^2*x*z, w^2*y^2, w^2*y*z, w^2*z^2, w*x^3, w*x^2*y, w*x^2*z, w*x*y^2, w*x*y*z, w*x*z^2, w*y^3, w*y^2*z, w*y*z^2, w*z^3, x^4, x^3*y, x^3*z, x^2*y^2, x^2*y*z, x^2*z^2, x*y^3, x*y^2*z, x*y*z^2, x*z^3, y^4, y^3*z, y^2*z^2, y*z^3, z^4]^T
Idx1 = [ 1     2     3     4
         2     5     6     7
         3     6     8     9
         4     7     9    10
         5    11    12    13
         6    12    14    15
         7    13    15    16
         8    14    17    18
         9    15    18    19
        10    16    19    20
        11    21    22    23
        12    22    24    25
        13    23    25    26
        14    24    27    28
        15    25    28    29
        16    26    29    30
        17    27    31    32
        18    28    32    33
        19    29    33    34
        20    30    34    35];

% First column of Idx2 shows the row index of matrix B. The second,
% third, and fourth columns indicate the row index of Bbar which should be
% used respectively xV, yV, and zV.
Idx2 = [ 21     1     2     3
         22     2     4     5
         23     3     5     6
         24     4     7     8
         25     5     8     9
         26     6     9    10
         27     7    11    12
         28     8    12    13
         29     9    13    14
         30    10    14    15
         31    11    16    17
         32    12    17    18
         33    13    18    19
         34    14    19    20
         35    15    20    21];     

Bx = zeros(35);      
% By = zeros(35);
% Bz = zeros(35);

% Define Ve as a complex 35*35 matrices (needed for C++ code generation)
Ve = complex(zeros(35));

% Set 'svd' and 'eig' functions as extrinsic when compiling the mex (needed for Mex generation)
coder.extrinsic('svd');
coder.extrinsic('eig'); 


%%

% Coefficinet matrix in the linearized system of multinomials (Cf * V = 0)
Cf =  CoefsVer3_1_1(m,n);

numEq = size(Cf,1);
% A is the coefficient matrix such that A * X = 0
A = zeros(4*numEq,56);
for i = 1 : 4
    idx = Idx(i,:);
    A((i-1)*numEq+1 : i*numEq, idx) = Cf;
end

% Split A into matrices A1 and A2. A1 corresponds to terms that contain w, 
% and A2 corresponds to the rest of the terms.
A1 = A(:,idx_w);
A2 = A(:,idx_w0);

Bbar = - A2 \ A1;

% Let 
% V = [w^4, w^3*x, w^3*y, w^3*z, w^2*x^2, w^2*x*y, w^2*x*z, w^2*y^2, w^2*y*z, w^2*z^2, w*x^3, w*x^2*y, w*x^2*z, w*x*y^2, w*x*y*z, w*x*z^2, w*y^3, w*y^2*z, w*y*z^2, w*z^3, x^4, x^3*y, x^3*z, x^2*y^2, x^2*y*z, x^2*z^2, x*y^3, x*y^2*z, x*y*z^2, x*z^3, y^4, y^3*z, y^2*z^2, y*z^3, z^4]^T
% then we have
% x V = w Bx V   ,   y V = w By V   ,   z V = w Bz V
for i = 1 : 20
    Bx(Idx1(i,1), Idx1(i,2)) = 1;
end
Bx(Idx2(:,1),:) = Bbar(Idx2(:,2),:);

% for i = 1 : 20
%     By(Idx1(i,1), Idx1(i,3)) = 1;
% end
% By(Idx2(:,1),:) = Bbar(Idx2(:,3),:);
% 
% for i = 1 : 20
%     Bz(Idx1(i,1), Idx1(i,4)) = 1;
% end
% Bz(Idx2(:,1),:) = Bbar(Idx2(:,4),:);


%%

% Find eigenvectors
[Ve, ~] = eig(Bx);

% Use only the real parts
V = real(Ve);

% Correct the sign of each column s.t. the first element (i.e., w) is always positive
V = bsxfun(@times, sign(V(1,:)), V);

% Recover quaternion elements  
w = sqrt(sqrt(V(1,:))); % NOTE: "sqrt(sqrt(.))" is 10 times faster than "nthroot(.,4)"
w3 = w.^3;
x = V(2,:) ./ w3;
y = V(3,:) ./ w3;
z = V(4,:) ./ w3;

Q = [w;
     x;
     y;
     z];

% Normalize s.t. each column of Q has norm 1
QNrm = sqrt(sum(Q.^2,1));
Q = bsxfun(@rdivide, Q, QNrm);






























































































