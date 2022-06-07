function [tr,qr,t2,q2] = get_relGT(kfi, btype, tTru, qTru, t1, q1)
  % get current pose
  if strcmp(btype, 'KITTI') || strcmp(btype, 'ICL') || strcmp(btype, 'NAIST')
    q2 = qTru(:, kfi);
    t2 = tTru(:, kfi); 
  elseif strcmp(btype, 'TUM')  
    fname = dat.dataset.fnames{f};
    ftime = str2double( fname(1:end-4) ); % Time at the current frame       
    [q2, t2] = InterpPoseVer1_1(ftime, dat.dataset.times, qTru, tTru); % Interpolate data to find the pose of the current camera frame  
  else
    error('Undefined dataset.')
  end
  % compute incrementation pose wrt prev frame a.k.a. relative pose 
  if strcmp(btype, 'KITTI')  || strcmp(btype, 'ICL')  || strcmp(btype, 'TUM')
    % Relative rotation between two frames (^2R_1 : rotation of frame 1 given in frame 2)
    qr = QuatMult(QuatConj(q2), q1); 
    % Relative trans. vec. in current coord. frame (^2t_21 : relative trans given in frame 2)
    tr  = Q2R(QuatConj(q2)) * (t2 - t1); 
  elseif strcmp(btype, 'NAIST') 
    % In this dataset the absolute pose is given in the camera coordinate frame, i.e.,  
    % c^R_w, c^t_w.(This is opposite of what is claimed in their website, unfortunately!)
    % Relative rotation between two frames (^2R_1 : rotation of frame 1 given in frame 2)
    qr = QuatMult(q2, QuatConj(q1)); 
    % Relative trans. vec. in current coord. frame (^2t_21 : relative trans given in frame 2)
    tr = t2 - Q2R(q2) * Q2R(QuatConj(q1)) * t1;  
  end
end % function [tr,qr,t2,q2] = get_groundTruth(benchtype, tTru, qTru, t1, q1)