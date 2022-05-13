classdef config_class < matlab.System
  properties
    %% common config
    test_ID     %= [];
    outDir      %= [pwd '/out/'];
    datDir      %= [pwd '/Datasets/']; % data dir     
    st_frame    %= 1; % start frame index
    end_frame   %= nan;% end frame index
    % %QuEst config
    QuEst_method    %= 'QuEst';
    benchtype       %= 'KITTI'; 
    seq             %= 3% aux config, used in KITTI     
    skipFrame       %= 0; % num of frames skipped bwt two keyframes         
    ransac_thresh   %= 1e-6; % ransac, sampson dist thresh
    surf_thresh     %= 200; % surf detector thresh
    maxPts          %= 30; % max features used in pose est (fewer features, faster compute)
    minPts          %= 8; % min feature required (6 to estimate a unique pose from RANSAC, or at least 8 for 8-pt algorithm)
    % QuEest runtime var
    keyFrames;
    cntr;
  end
  methods
    %% constructor
    function obj = config_class(namedArgs)
      arguments
        namedArgs.test_ID       = [];
        namedArgs.outDir        = [pwd '/out/'];
        namedArgs.datDir        = [pwd '/data/']; 
        namedArgs.st_frame      = 1; 
        namedArgs.end_frame     = nan;
        namedArgs.QuEst_method    = 'QuEst';
        namedArgs.benchtype       = 'KITTI'; 
        namedArgs.seq             = 3% aux config, used in KITTI     
        namedArgs.skipFrame       = 0; % num of frames skipped bwt two keyframes         
        namedArgs.ransac_thresh   = 1e-6; % ransac, sampson dist thresh
        namedArgs.surf_thresh     = 200; % surf detector thresh
        namedArgs.maxPts          = 30; % max features used in pose est (fewer features, faster compute)
        namedArgs.minPts          = 8; 
      end 
      obj.test_ID       = namedArgs.test_ID   ;    
      obj.outDir        = namedArgs.outDir    ;    
      obj.datDir        = namedArgs.datDir    ;    
      obj.st_frame      = namedArgs.st_frame  ;    
      obj.end_frame     = namedArgs.end_frame ;    
      obj.QuEst_method    = namedArgs.QuEst_method;    
      obj.benchtype       = namedArgs.benchtype   ;    
      obj.seq             = namedArgs.seq         ;    
      obj.skipFrame       = namedArgs.skipFrame   ;    
      obj.ransac_thresh   = namedArgs.ransac_thresh;  
      obj.surf_thresh     = namedArgs.surf_thresh ;    
      obj.maxPts          = namedArgs.maxPts      ;    
      obj.minPts          = namedArgs.minPts      ;    

    end    
    function obj = init(obj)
      obj.outDir = [obj.outDir 'out_' obj.test_ID '/'];
      if not(isfolder(obj.outDir))
        disp('outDir does NOT exist: ');
        disp(obj.outDir);
        pause(5);
        mkdir(obj.outDir);
      end 
    end

  end
end