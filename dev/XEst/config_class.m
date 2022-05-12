classdef config_class
  %UNTITLED Summary of this class goes here
  %   Detailed explanation goes here

  properties
    %% common config
    test_ID   
    outDir    = [pwd '/out/'];
    datDir    = [pwd '/data/']; % data dir     
    st_frame    = 1; % start frame index
    end_frame   = nan;% end frame index
    % QuEst config
    QuEst_method    = 'QuEst';
    benchtype       = 'KITTI'; 
    seq             = 3% aux config, used in KITTI     
    skipFrame       = 0; % num of frames skipped bwt two keyframes         
    ransac_thresh   = 1e-6; % ransac, sampson dist thresh
    surf_thresh     = 200; % surf detector thresh
    maxPts          = 30; % max features used in pose est (fewer features, faster compute)
    minPts          = 8; % min feature required (6 to estimate a unique pose from RANSAC, or at least 8 for 8-pt algorithm)
  end
  methods
    function obj = init(obj)
      obj.outDir = strcat(pwd,obj.outDir,'out_',obj.test_ID,'/');
      if ~exist(obj.outDir,'dir')
        disp('outDir does NOT exist: ');
        disp(obj.outDir);
        exit;
      end 
    end

  end
end