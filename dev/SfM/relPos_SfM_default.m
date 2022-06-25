function [Q, T, inlierIdx, valid_PtFrac] = relPos_SfM_default(m1, m2, cam)
  for i = 1:100   
    % Estimate the essential matrix.    
    [E, inlierIdx, valid_PtFrac] = estimateEssentialMatrix(m1, m2, cam);
    % Make sure we get enough inliers
    if sum(inlierIdx) / numel(inlierIdx) < .3
      return;
    end    
    % Get the epipolar inliers.
    inlierPts1 = m1(inlierIdx, :);
    inlierPts2 = m2(inlierIdx, :);    
    % Compute the camera pose from the fundamental matrix. Use half of the
    % points to reduce computation.
    [Q, T, valid_PtFrac] = relativeCameraPose(E, cam, inlierPts1(1:2:end,:),...
      inlierPts2(1:2:end, :));
      % valid_PtFrac is the fraction of inlier points that project in
    % front of both cameras. If the this fraction is too small, then the
    % fundamental matrix is likely to be incorrect.
    if valid_PtFrac > .8
      return;
    end
  end % for
  % After 100 attempts valid_PtFrac is still too low.
  error('Unable to compute the Essential matrix');
end 