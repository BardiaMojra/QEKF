function [R, T, inLIdx, inLFract] = rPos_RQuEst(m1, m2, camIntrs)

  [E, inLIdx, inLFract] = estimateEssentialMatrix(m1, m2, camIntrs);


  
  if sum(inLIdx) / numel(inLIdx) < .3 % Make sure we get enough inliers
    return;
  end    
  inLPts1 = m1(inLIdx, :); % get the epipolar inliers.
  inLPts2 = m2(inLIdx, :);    
  % Compute cam pos from fundamental matrix. Use half of pts to reduce comp
  [R, T, inLFract] = relativeCameraPose(E, camIntrs, inLPts1(1:2:end,:),...
    inLPts2(1:2:end, :));
end 