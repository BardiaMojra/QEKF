%% XEst main 
% Summary of example objective
%

%% init
% warning('off','all'); % turn off all warnings!
addpath(genpath('./'));

%% config
cfg = config_class();
cfg.test_ID       = 'quest_unit_test';
cfg.seq           = 3; % aux config, used in KITTI 
cfg.skipFrame     = 0; % num of frames skipped bwt two keyframes 
% cfg.st_frame      = 650; % start frame index
% cfg.end_frame     = 680; 
cfg.benchtype     = "KITTI";
cfg.QuEst_method  = "QuEst";

cfg = init(cfg);


quest = quest_class();
% quest = copyprops(cfg, quest);

quest = quest.load_cfg(cfg.test_ID, ...
                       cfg.outDir, ...
                       cfg.QuEst_method, ...
                       cfg.st_frame, ...
                       cfg.end_frame, ...
                       cfg.datDir, ...
                       cfg.benchtype, ...
                       cfg.seq, ...
                       cfg.skipFrame, ...
                       cfg.ransac_thresh, ...
                       cfg.surf_thresh, ...
                       cfg.maxPts, ...
                       cfg.minPts);

% needed for VEst module 
cfg.keyFrames = quest.keyFrames;

vest = vest_class();
% vest.config(cfg);

% rqekf = rqekf_class();
% rqekf.config(cfg);

%% run 
for i = cfg.keyFrames
  cfg.cntr = cfg.cntr+1;

%   ppoints = quest.ppoints; 
  [quest,T,Q] = quest.get_pose(i, dat, cfg);
%   matches = quest.matches; % get current matches features
%   npoints = quest.ppoints; % get current feature points 
  
%   [vest, V, W] = vest.get_vel(i, quest.matches); 

  % get velocity
  [m, m_dot] = vest.prep_matches(quest.matches);
  [v_est, w_est] = vest.PoCo(m,m_dot); % call algorithm 
  v_est_norm = vest.normalize(v_est);
  v_err = get_v_err(v_est_norm, v_true_norm); 
  w_err = get_w_err(w_est, w_true); %
        
  %  error_v = sqrt( (v_est_norm(1)-v_true_norm(1))^2 + (v_est_norm(2)-v_true_norm(2))^2 + (v_est_norm(3)-v_true_norm(3))^2 ); 
      error_v =1/pi*acos((v_est_norm'*v_true_norm)); 
        error_w = sqrt( (w_est(1)-w_true(1))^2 + (w_est(2)-w_true(2))^2 + (w_est(3)-w_true(3))^2 ); 
        
        v_error_buffer(1, j) = error_v; 
        w_error_buffer(1, j) = error_w; 
  
  %   Q_VEst = exp_map(W);
%    if isequal(Q,Q_VEst)
%     disp('QuEst and VEst rotation estimates match!');
%    end 


%   x_TVWQ = QEKF(i,TQ,VW);



end 

%% post-processing 
res = quest.get_res();
