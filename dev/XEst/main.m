%% XEst main 

%% init
close all; clear; clc 
addpath(genpath('./'));

% config - datasets handled by cfg object
cfg = config_class(test_ID   =   'XEst_unit_test');

dlog = dlogger_class();
dlog.load_cfg(cfg); % overwrite default settings 

quest = quest_class();
quest.load_cfg(cfg); 

vest = vest_class();
vest.load_cfg(cfg); 

% qekf = qekf_class();
% qekf.load_cfg(cfg); 


%% run 
for b = 1: length(cfg.benchmarks) % --->> iter benchmarks
  cntr  = 0;
  for frame_idx = cfg.dats{b}.keyFrames % --->> iter keyframes 
    cntr = cntr+1;

    TQ_sols = quest.get_pose(frame_idx, cfg.dats{b}, cfg); % get pose
    
    [V, W] = vest.get_vel(cfg.dats{b}.matches); % get velocity

%     state  = qekf.get_state(TQ_sols, V, W); % get state
  
    dlog.log_state(b, cfg.benchmarks{b}, cntr, frame_idx, TQ_sols, V, W); %, state);

  end % for frame_idx = cfg.dats.keyFrames
end % for b = 1:length(cfg.benchmarks)

%% post-processing 

quest_res   = quest.get_res(cfg, dlog);
vest_res     = vest.get_res(cfg, dlog);
% qekf_res     = qekf.get_res(cfg, dlog);

