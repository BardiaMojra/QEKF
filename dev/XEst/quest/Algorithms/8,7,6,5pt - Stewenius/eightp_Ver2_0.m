% 
% Ver 2_0:
%           - Points are preconditioned to have the best results
%           - Based on 8pt by Peter Kovesi (Copyright (c) 2002-2005 Peter
%             Kovesi, http://www.csse.uwa.edu.au/)
%
function E = eightp_Ver2_0(x1,x2)

% Number of points
npts = size(x1,2);

% Normalise each set of points so that the origin 
% is at centroid and mean distance from origin is sqrt(2). 
% normalise2dpts also ensures the scale parameter is 1.
[x1, T1] = normalise2dpts(x1);
[x2, T2] = normalise2dpts(x2);

% Build the constraint matrix
A = [x2(1,:)'.*x1(1,:)'   x2(1,:)'.*x1(2,:)'  x2(1,:)' ...
     x2(2,:)'.*x1(1,:)'   x2(2,:)'.*x1(2,:)'  x2(2,:)' ...
     x1(1,:)'             x1(2,:)'            ones(npts,1) ];       


[U,D,V] = svd(A,0); % Use MATLAB economy SVD

% Extract fundamental matrix from the column of V corresponding to
% smallest singular value.
F = reshape(V(:,9),3,3)';

% Enforce constraint that fundamental matrix has rank 2 by performing
% a svd and then reconstructing with the two largest singular values.
[U,D,V] = svd(F,0);
F = U*diag([D(1,1) D(2,2) 0])*V';

% Denormalise
E = T2' * F * T1;


return
    








