function [R, T, inLIdx] = get_rPos(m1, m2, camIns, pos_alg)
  %% cfg
  inLThresh = .8; % RANSAC inlier threshold
  trials = 100;
  %% pose alg
  if strcmp(pos_alg, "default_5Pt")
    [R, T, inLIdx] = rPos_SfM_def(m1, m2, camIns, trials, inLThresh);
    Q = rotm2quat(R);
  elseif strcmp(pos_alg, "RQuEst")
    [Q, T, inLIdx] = rPos_RQuEst(m1, m2, camIns, trials, inLThresh);
    R = quat2rotm(Q);
  else
    assert(false, "[get_rPos]--> unknown pos est alg!");
  end
  disp("R:"); disp(R);
  disp("Q:"); disp(Q);
  disp("T:"); disp(T);
  disp("inLFract:"); disp(inLFract);
end
