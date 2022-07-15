function [R, T, Es, inLIdx, inLFract] = rPos_RQuEst(m1, m2, intrns, trials, thresh)
  %% cfg
  if ~isnumeric(m1)
    m1 = m1.Location;
  end
  if ~isnumeric(m2)
    m2 = m2.Location;
  end
  for i = 1:trials
    [Es, inLIdx] = RQuEst(m1, m2, intrns);
    disp("[rPos_RQuEst]->> M: "); disp(M);
    if sum(inLIdx) / numel(inLIdx) < .3 % Make sure we get enough inliers
      continue;
    end
    iP1 = m1(inLIdx, :); % get the epipolar inliers.
    iP2 = m2(inLIdx, :);
    % find correct solutions
    [R, T, inLFract] = RQuEst_relCamPose(Es, intrns, iP1(1:2:end,:), iP2(1:2:end,:));
    if inLFract > thresh % should be .8-.9
      return;
    end
  end % for
  error('[rPos_RQuEst]->> after 100 iters, unable to compute the Essential matrix!');
end
