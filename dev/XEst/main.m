%% RQEKF main 
% Summary of example objective
%
%




%% init
% warning('off','all'); % turn off all warnings!
addpath(genpath('./'));

cfg = config_class();

%% config
% common
cfg.outDir        = 'out/';
cfg.srcDir        = 'data/'; 
% quest
cfg.algs          = {'QuEst_RANSAC_v0102'};
cfg.benchmarks    = {'KITTI'}; 
cfg.seq           = 3; % aux config, used in KITTI 
% cfg.st_frame      = 650; % start frame index
% cfg.end_frame     = 680; 
cfg.skip_frame    = 0; % num of frames skipped bwt two keyframes 
cfg.ransac_thresh = 1e-6; % ransac, sampson dist thresh
cfg.surf_thresh   = 200; % surf detector thresh
cfg.maxPts        = 30; % max features used in pose est (fewer features, faster compute)
cfg.minPts        = 6; % min feature required (6 to estimate a unique pose from RANSAC)


cfg = cfg.init();

quest = quest_class();
quest = quest.load_cfg(cfg);

vest = vest_class();
vest.config(cfg);

rqekf = rqekf_class();
rqekf.config(cfg);

%% run 
for i = cfg.keyframes

  TQ = quest.get_pose(i);
  VW = vest.get_vel(i); 
  x_TVWQ = QEKF(i,TQ,VW);

end 

%% post-processing 

