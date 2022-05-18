%% XEst main 
% Summary of example objective
%

%% init

% warning('off','all'); % turn off all warnings!
close all 
clear 
clc 
addpath(genpath('./'));

%% datasets 






%% config
cfg = config_class(... 
                                 test_ID               = 'quest_unit_test'  );
cfg = init(cfg);

dlog = dlog_class();
dlog = dlog.load_cfg(cfg);% overwrite default settings 

quest = quest_class();
quest = quest.load_cfg(cfg); 

vest = vest_class();
% vest = vest.load_cfg(vest, cfg); 

% qekf = qekf_class();
% qekf = qekf.load_cfg(qekf, cfg); 



%% run 
for i = cfg.keyFrames
  cfg.cntr = cfg.cntr+1;

%   ppoints = quest.ppoints; 
  [quest,T,Q] = quest.get_pose(i, dat, cfg);
  
  dlog.T_hist(i,:) =  v;
  dlog.Q_hist(i,:) =  w;
  
% matches = quest.matches; % get current matches features
%   npoints = quest.ppoints; % get current feature points 
  
  [vest, V, W] = vest.get_vel(i, quest.matches); % get velocity
  
%   x_TVWQ = QEKF(i,TQ,V,W);



  dlog.kF_hist(i,:) =  i;
  dlog.T_hist(i,:)  =  T;
  dlog.Q_hist(i,:)  =  Q;
  dlog.V_hist(i,:)  =  V;
  dlog.W_hist(i,:) =  W;
    
  %   Q_VEst = exp_map(W);
%    if isequal(Q,Q_VEst)
%     disp('QuEst and VEst rotation estimates match!');
%    end 



end 

%% post-processing 

res = quest.get_res();
