%% XEst main 

%% init
close all; clear; clc; addpath(genpath('./'));

cfg   = config_class(test_ID    = 'test_001', ... % --->> config on the fly
                     benchmark  = 'KITTI');
dlog  = dlogger_class(); dlog.load_cfg(cfg); 
quest = quest_class(); quest.load_cfg(cfg); 
vest  = vest_class(); vest.load_cfg(cfg); 
qekf  = qekf_handler_class(); qekf.load_cfg(cfg); 
rpt   = report_class(); rpt.load_cfg(cfg);
%% todo: keep track features for x number of frames

%% run 
cntr  = 0;  
for frame_idx = cfg.dat.keyFrames % --->> iter keyframes 
  cntr      = cntr+1;
  TQVW_sols = quest.get_pose(frame_idx, cfg.dat); % get pose
  TQVW_sols = vest.get_vel(cfg.dat.matches, TQVW_sols); % get velocity
  st_sols   = qekf.run_filter(TQVW_sols); % run filter
  dlog.log_state(cntr, frame_idx, TQVW_sols, st_sols);
end 

%% results
quest.get_res(cfg, dlog);
vest.get_res(cfg, dlog);
qekf.get_res(cfg, dlog);

%% report
rpt.gen_plots(dlog);
rpt.gen_report(quest, vest, qekf);
disp("end of process...");