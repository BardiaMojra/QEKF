function [Q, T, inLIdx] = get_relPos(m1, m2, camIntrs, pos_alg)
  %% cfg
  inLThresh = .8; % RANSAC inlier threshold
  trials = 100;
  if strcmp(pos_alg, "default_5Pt")
    [Q, T, inLIdx] = rPos_SfM_def(m1, m2, camIntrs, trials, inLThresh);
    Q = rotm2quat(R);
  elseif strcmp(pos_alg, "RQuEst")
    [Q, T, inLIdx] = rPos_RQuEst(m1, m2, camIntrs, trials, inLThresh);
    R = quat2rotm(Q);
  else
    assert(false, "[get_relPos]--> unknown pos est alg!");
  end
  disp("R:"); disp(R);
  disp("Q:"); disp(Q);
  disp("T:"); disp(T);
  disp("inLFract:"); disp(inLFract);
end
