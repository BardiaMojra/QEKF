function [R, T, inLIdx] = rPos_RQuEst(m1, m2, camIns, trials, thresh)
  %% cfg

  if ~isnumeric(m1)
    m1 = m1.Location;
  end
  if ~isnumeric(m2)
    m2 = m2.Location;
  end
  for i = 1:trials
    %[E, inLIdx] = estimateEssentialMatrix(m1, m2, camIntrs);
    [M, E, inLIdx] = RQuEst(m1, m2, camIns);
    disp("[rPos_RQuEst]->> M: "); disp(M);
    if sum(inLIdx) / numel(inLIdx) < .3 % Make sure we get enough inliers
      continue;
    end
    inLPts1 = m1(inLIdx, :); % get the epipolar inliers.
    inLPts2 = m2(inLIdx, :);

    % find correct solutions
    [R, T, inLFract] = RQuEst_relCamPose(E, camIns, inLPts1(1:2:end,:),...
      inLPts2(1:2:end, :));

    disp("[rPos_RQuEst]->> inLFract:"); disp(inLFract);
    if inLFract > thresh % should be .8-.9
      return;
    end
  end % for
  error('[rPos_RQuEst]->> after 100 iters, unable to compute the Essential matrix!');
end
