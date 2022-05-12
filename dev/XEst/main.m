%% XEst main 
% Summary of example objective
%
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
cfg = init(cfg);
disp("output dir:");
disp(cfg.outDir);
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
% obj.numImag       = length(obj.dataset.fnames); 
% obj.keyFrames     = 2+obj.skipFrame:1+obj.skipFrame:obj.numImag; 
% obj.numKeyFrames  = length(obj.keyFrames); 
% obj.numMethods    = length(obj.algorithms); 
% obj.rotErr        = NaN(obj.numKeyFrames,obj.numMethods); 
% obj.tranErr       = NaN(obj.numKeyFrames,obj.numMethods); 
% obj.Q             = cell(obj.numKeyFrames,obj.numMethods); 
% obj.T             = cell(obj.numKeyFrames,obj.numMethods);

% vest = vest_class();
% vest.config(cfg);

% rqekf = rqekf_class();
% rqekf.config(cfg);

%% run 
for i = cfg.keyFrames

  [quest,T,Q] = quest.get_pose(i);
  cfg.cntr = quest.cntr;
  [vest,V,W] = vest.get_vel(i,T,Q); 
%   x_TVWQ = QEKF(i,TQ,VW);

end 

%% post-processing 
res = quest.get_res();
