%% RQKF main 
% Summary of example objective
%
%



%% config
outDir = 'out/';
srcDir = 'data/'; 
dataset = 'KITTI'; 
seq = 3;


quest = quest_class('outDir',outDir,...
                    'srcDir',srcDir,...
                    'dat_name',dataset,...
                    'dat_seq',seq);
quest.config();

vest = vest_class();
vest.config();

rqekf = rqekf_class();
rqekf.config();

%% run 
for i = keyframes

  TQ = quest.get_pose(i);
  VW = vest.get_vel(i); 
  x_TVWQ = QEKF(i,TQ,VW);

end 

%% post-processing 

