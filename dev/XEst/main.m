%% XEst main 

%% init
close all; clear; clc 
addpath(genpath('./'));

% config - datasets handled by cfg object
cfg = config_class( test_ID     = 'test_001', ...
                    benchmark   = 'KITTI');
                    %st_frame   = 650, ... % start frame index
                    %end_frame  = 680);
                    
dlog = dlogger_class(); 
dlog.load_cfg(cfg); 
quest = quest_class();
quest.load_cfg(cfg); 
vest = vest_class();
vest.load_cfg(cfg); 
qekf = qekf_handler_class();
qekf.load_cfg(cfg); 

%% todo: keep track features for x number of frames


%% run 
cntr  = 0;
for frame_idx = cfg.dat.keyFrames % --->> iter keyframes 
  cntr = cntr+1;
  TQVW_sols = quest.get_pose(frame_idx, cfg.dat); % get pose
  TQVW_sols = vest.get_vel(cfg.dat.matches, TQVW_sols); % get velocity
  st_sols   = qekf.run_filter(TQVW_sols); % run filter

  dlog.log_state(cntr, frame_idx, TQVW_sols, st_sols);
end % for frame_idx = cfg.dats.keyFrames

%% results
quest_res   = quest.get_res(cfg, dlog);
vest_res    = vest.get_res(cfg, dlog);
qekf_res    = qekf.get_res(cfg, dlog);

