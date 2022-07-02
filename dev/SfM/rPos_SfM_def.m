% get_relPos Robustly estimate relative camera pose
%  [orientation, location, inlierIdx] = get_relPos(
%    matchedPoints1, matchedPoints2, cameraParams) returns the pose of
%  camera 2 in camera 1's coordinate system. The function calls
%  estimateEssentialMatrix and cameraPose functions in a loop, until
%  a reliable camera pose is obtained.
%
%  Inputs:
%  -------
%  m1       - points from image 1 specified as an M-by-2 matrix of
%                   [x,y] coordinates, or as any of the point feature types
%  m2       - points from image 2 specified as an M-by-2 matrix of
%                   [x,y] coordinates, or as any of the point feature types
%  camIntrs - cameraParameters object
%
%  Outputs:
%  --------
%  R - the orientation of camera 2 relative to camera 1
%                specified as a 3-by-3 rotation matrix
%  T    - the location of camera 2 in camera 1's coordinate system
%                specified as a 3-element vector
%  inLIdx   - the indices of the inlier points from estimating the
%                fundamental matrix
%
%  See also estimateEssentialmatrix, estimateFundamentalMatrix,
%  relativeCameraPose
% Copyright 2016 The MathWorks, Inc.
function [R, T, inLIdx] = rPos_SfM_def(m1, m2, camIntrs, trials, inLThresh)
  if ~isnumeric(m1)
    m1 = m1.Location;
  end
  if ~isnumeric(m2)
    m2 = m2.Location;
  end
  for i = 1:trials
    [E, inLIdx] = estimateEssentialMatrix(m1, m2, camIntrs);
    if sum(inLIdx) / numel(inLIdx) < .3 % Make sure we get enough inliers
      continue;
    end
    inLPts1 = m1(inLIdx, :); % get the epipolar inliers.
    inLPts2 = m2(inLIdx, :);
    % Compute cam pos from fundamental matrix. Use half of pts to reduce comp
    [R, T, inLFract] = relativeCameraPose(E, camIntrs, inLPts1(1:2:end,:),...
      inLPts2(1:2:end, :));
    disp("[rPos_SfM_def]->> inLFract"); disp(inLFract);
    if inLFract > inLThresh % -->> must have hi frac of inliers or F-mat would be wrong
      return;
    end
  end % for
  error('[rPos_SfM_def]->> after 100 iters, unable to compute the Essential matrix!');
end
