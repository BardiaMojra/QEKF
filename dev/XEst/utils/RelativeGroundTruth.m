function [relPos, posp] = RelativeGroundTruth(i, posp, dataset)

q1 = posp.q1;
t1 = posp.t1;

benchtype = dataset.benchtype;
qTru = dataset.qTru;
tTru = dataset.tTru;

if strcmp(benchtype, 'KITTI') || strcmp(benchtype, 'ICL') || strcmp(benchtype, 'NAIST')
    q2 = qTru(:,i);
    t2 = tTru(:,i);
elseif strcmp(benchtype, 'TUM')
    fname = dataset.fnames{i};
    ftime = str2double( fname(1:end-4) ); % Time at the current frame       
    [q2, t2] = InterpPoseVer1_1(ftime, dataset.times, qTru, tTru); % Interpolate data to find the pose of the current camera frame  
else
    error('Undefined dataset.')
end
% Relative pose
if strcmp(benchtype, 'KITTI')  || strcmp(benchtype, 'ICL')  || strcmp(benchtype, 'TUM')
    
    % Relative rotation between two frames (^2R_1 : rotation of frame 1 
    % given in frame 2)
    qr = QuatMult(QuatConj(q2),q1); 
    % Relative trans. vec. in current coord. frame (^2t_21 : relative 
    % translation given in frame 2)
    tr  = Q2R(QuatConj(q2)) * (t2 - t1); 
    
elseif strcmp(benchtype, 'NAIST') 
    % In this dataset the absolute pose is given in the camera coordinate
    % frame, i.e., c^R_w, c^t_w.(This is opposite of what is claimed in 
    % their website, unfortunately!)
    
    % Relative rotation between two frames (^2R_1 : rotation of frame 1 
    % given in frame 2)
    qr = QuatMult(q2,QuatConj(q1)); 
    % Relative trans. vec. in current coord. frame (^2t_21 : relative 
    % translation given in frame 2)
    tr = t2 - Q2R(q2)*Q2R(QuatConj(q1)) * t1;  
end

% Store the current pose for the next iteration
posp.q1 = normalizeVec(q2);
posp.t1 = t2;

% Relative pose
relPos.qr = normalizeVec(qr);
relPos.tr = tr;