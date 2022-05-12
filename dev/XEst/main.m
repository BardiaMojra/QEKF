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
cfg.skipFrame    = 0; % num of frames skipped bwt two keyframes 
% cfg.st_frame      = 650; % start frame index
% cfg.end_frame     = 680; 
cfg = cfg.init();

quest = quest_class();
quest = quest.load_cfg(cfg);

% vest = vest_class();
% vest.config(cfg);

% rqekf = rqekf_class();
% rqekf.config(cfg);

%% run 
for i = cfg.keyframes

  [quest, TQ] = quest.get_pose(i);
%   VW = vest.get_vel(i); 
%   x_TVWQ = QEKF(i,TQ,VW);

end 

%% post-processing 

