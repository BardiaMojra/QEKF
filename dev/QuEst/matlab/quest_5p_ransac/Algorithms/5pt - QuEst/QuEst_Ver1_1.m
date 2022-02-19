%% QuEst (Quaternion pose Estimation) algorithm 
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
% sol.Q   :  The recovered rotation in quaternions. 
%
% sol.R   :  The recovered rotation in rotation matrix format. 
%
% sol.T   :  Associated translation vectors. 
%
% sol.Z1  :  Depths in the first camera frame.
%
% sol.Z2  :  Depths in the second camera frame.
%
%
% NOTE: T, Z1, and Z2 are recovered up to a common scale factor.
%
%
% Copyright (C) 2013-2017, by Kaveh Fathian.
%
% This program is a free software: you can redistribute it and/or modify it
% under the terms of the GNU lesser General Public License version 3  or
% (at your option) any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details: 
% <http://www.gnu.org/licenses/>
%
%
%%

function sol = QuEst_Ver1_1(m,n)
%% Find rotation in the form of quaternions

% Each column of 'Q' represent a quaternion solution candidate
%
% 
% Q = QuEst_5Pt_Ver5_2(m,n);          % The exact implementation of the algorithm in the paper
% Q = QuEst_5Pt_Ver7_8(m,n);          % For more accuracy use this 
Q = QuEst_5Pt_Ver7_8_spd(m,n);        % For faster execution use this 
% Q = QuEst_5Pt_Ver7_8_spd_mex(m,n);  % For fastest performace run this .mex version 


%% Find translation & depths 

% Each column of 'T' is the (unit norm) translation recovered for the
% corresponding quaternion solution candidate.
%
% 'Z1' and 'Z2' are depths of the feature points in the first and second 
% camera frames, respectively. 'R' is the rotation matrix associated to 
% each quaternion. 'res' is the residue value after enforcing the positive
% depth constraint and can be used to eliminate pose solutions with negative depths.
% negative depths.
[T, Z1, Z2, R, Res] = FindTransDepth_Ver1_0(m,n, Q);


%% Store results

sol.Q = Q;
sol.R = R;
sol.T = T;
sol.Z1 = Z1;
sol.Z2 = Z2;






































































































































































