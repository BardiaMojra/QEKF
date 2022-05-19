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
for bnch = length(cfg.benchmarks) % --->> iter benchmarks
  idx  = 0; % test index
  for frame_idx = cfg.dats{bnch}.keyFrames % --->> iter keyframes 
    idx = idx+1;

    [TQ_sols, Q] = quest.get_pose(frame_idx, cfg.dats{bnch}, cfg); 
    
    [V, W] = vest.get_vel(cfg.dats{bnch}.matches); % get velocity
    
    vest.check_w_QuEst(W, Q);
  
    % x_TVWQ = QEKF(i,TQ,V,W);
  
    dlogger.log_state(bnch, idx, frame_idx, TQ_sols, V, W);

  end % for frame_idx = cfg.dats.keyFrames
end % bnch = length(cfg.benchmarks)

%% post-processing 

quest_res = quest.get_res(dlogger);
vest_res = vest.get_res(dlogger);
qekf_res = qekf.get_res(dlogger);

