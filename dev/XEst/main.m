%% XEst main 

%% init
close all; clear; clc; addpath(genpath('./'));
addpath(genpath('/home/smerx/DATA')); 
cfg   = config_class(TID  = 'T00001_0-40fr_150surf', benchmark  = 'KITTI');
pfeat = pfeature_class(); pfeat.load_cfg(cfg);
dlog  = dlogger_class(); dlog.load_cfg(cfg); 
quest = quest_class(); quest.load_cfg(cfg); 
vest  = vest_class(); vest.load_cfg(cfg); 
qekf  = qekf_handler_class(); qekf.load_cfg(cfg); 
rpt   = report_class(); rpt.load_cfg(cfg);
%% todo: keep track features for x number of frames

%% run 
cntr  = 0;  
for kf = cfg.kframes % --->> iter keyframes 
  cntr = cntr+1;
  pfeat.get_pmats(kf, cfg.dat);
  TQVW_sols = quest.get_pose(kf, cfg.dat); % get pose
  TQVW_sols = vest.get_vel(cfg.dat.matches, TQVW_sols); % get velocity
  st_sols   = qekf.run_filter(TQVW_sols); % run filter

  dlog.log_state(cntr, kf, TQVW_sols, st_sols);
end 

%% results
dlog.get_logs();
dlog.plot_logs();
quest.get_res(cfg, dlog);
vest.get_res(cfg, dlog);
qekf.get_res(cfg, dlog);
dlog.save_logs();
%% report
%rpt.gen_plots(cfg.dat, dlog, quest, vest, qekf);
rpt.gen_report(quest, vest, qekf);
disp("end of process...");