%% XEst main 
% Summary of example objective
%

%% init
% warning('off','all'); % turn off all warnings!
close all; clear; clc 
addpath(genpath('./'));

% config - datasets handled in cfg 
cfg = config_class(test_ID   =   'quest_unit_test');

dlog = dlogger_class();
dlog.load_cfg(cfg); % overwrite default settings 

quest = quest_class();
quest.load_cfg(cfg); 

vest = vest_class();
% vest.load_cfg(vest, cfg); 

% qekf = qekf_class();
% qekf.load_cfg(qekf, cfg); 


%% run 
for b = 1: length(cfg.benchmarks) % --->> iter benchmarks
  cntr  = 0;
  for frame_idx = cfg.dats{b}.keyFrames % --->> iter keyframes 
    cntr = cntr+1;

    [TQ_sols] = quest.get_pose(frame_idx, cfg.dats{b}, cfg); 
    
    [V, W] = vest.get_vel(cfg.dats{b}.matches); % get velocity
    
%     vest.check_w_QuEst(W, Q);
  
    % x_TVWQ = QEKF(i,TQ,V,W);
  
    dlog.log_state(b, cfg.benchmarks{b}, cntr, frame_idx, TQ_sols, V, W);

  end % for frame_idx = cfg.dats.keyFrames
end % for b = 1:length(cfg.benchmarks)

%% post-processing 

quest_res   = quest.get_res(cfg, dlog);
vest_res     = vest.get_res(cfg, dlog);
qekf_res     = qekf.get_res(cfg, dlog);

