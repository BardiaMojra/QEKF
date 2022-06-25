% get_relPos Robustly estimate relative camera pose
%  [orien, loc, inlierIdx] = get_relPos(
%    matchedPoints1, matchedPoints2, cameraParams) returns the pose of
%  camera 2 in camera 1's coordinate system. The function calls
%  estimateEssentialMatrix and cameraPose functions in a loop, until
%  a reliable camera pose is obtained.
%
%  Inputs:
%  -------
%  matchedPoints1 - points from image 1 specified as an M-by-2 matrix of
%                   [x,y] coordinates, or as any of the point feature types
%  matchedPoints2 - points from image 2 specified as an M-by-2 matrix of
%                   [x,y] coordinates, or as any of the point feature types
%  cameraParams   - cameraParameters object
%
%  Outputs:
%  --------
%  orien - the orien of camera 2 relative to camera 1
%                specified as a 3-by-3 rotation matrix
%  loc    - the loc of camera 2 in camera 1's coordinate system
%                specified as a 3-element vector
%  inlierIdx   - the indices of the inlier points from estimating the
%                fundamental matrix
%
%  See also estimateEssentialmatrix, estimateFundamentalMatrix, 
%  relativeCameraPose

% Copyright 2016 The MathWorks, Inc. 

function [Q, T, inlierIdx] = get_relPos(m1, m2, cam, pos_alg)
  if ~isnumeric(m1)
    m1 = m1.Location;
  end
  if ~isnumeric(m2)
    m2 = m2.Location;
  end
 
  if strcmp(pos_alg, "default")
    [Q, T, inlierIdx] = relPos_SfM_default(m1, m2, cam);
  elseif strcmp(pos_alg, "QuEst")
    [M, inliers] = relPos_QuEst_RANSAC(m1, m2, 200);   
    Q = M.Q;
    T = M.t;  
  else 
    assert(false, "[get_relPos]--> unknown pos est alg!");
  end
  disp("Q"); disp(Q);
  disp("T"); disp(T);

end
