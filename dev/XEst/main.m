%% XEst main 
% Summary of example objective
%

%% init
% warning('off','all'); % turn off all warnings!
close all; clear; clc 
addpath(genpath('./'));

% config - datasets handled in cfg 
cfg = config_class(test_ID   =   'quest_unit_test');

dlogger = dlogger_class();
dlogger.load_cfg(cfg); % overwrite default settings 

quest = quest_class();
quest.load_cfg(cfg); 

vest = vest_class();
% vest.load_cfg(vest, cfg); 

% qekf = qekf_class();
% qekf.load_cfg(qekf, cfg); 


%% run 
benchName = [];
for bnch = length(cfg.benchmarks)
  benchName = cfg.benchmarks(bnch);
  int idx  = 0; % test index
  for frame_idx = cfg.dats.keyFrames
    idx = idx+1;
  
    [T,Q]   = quest.get_pose(frame_idx, dat, cfg);  
    [V, W] = vest.get_vel(frame_idx, quest.matches); % get velocity
    vest.check_w_QuEst(W, Q);
  %   x_TVWQ = QEKF(i,TQ,V,W);
  
    dlogger.log_state(bnch, benchName, idx, frame_idx, T, Q, V, W);
    
    dlogger.datidx_hist(idx,1) = idx;
    dlogger.kF_hist(idx,:) =  frame_idx;
    dlogger.T_hist(idx,:)  =  T;
    dlogger.Q_hist(idx,:)  =  Q;
    dlogger.V_hist(idx,:)  =  V;
    dlogger.W_hist(idx,:) =  W;
      
    %   Q_VEst = exp_map(W);
  %    if isequal(Q,Q_VEst)
  %     disp('QuEst and VEst rotation estimates match!');
  %    end 
  
  end % end of for frame_idx = cfg.dats.keyFrames
end % bnch = length(cfg.benchmarks)

%% post-processing 

quest_res = quest.get_res(dlogger);
vest_res = vest.get_res(dlogger);
qekf_res = qekf.get_res(dlogger);

