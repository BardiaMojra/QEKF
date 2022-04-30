classdef config_class
  %UNTITLED Summary of this class goes here
  %   Detailed explanation goes here

  properties
    % settings
    test_ID       
    outDir = 'out/'; % out dir        
    srcDir = 'data/'; % data dir 
    benchmarks % benchmark datasets e.g. KITTI  
    seq % aux config, used in KITTI     
    skip_frame % num of frames skipped bwt two keyframes   
    st_frame = 1; % start frame index   
    end_frame = nan;      
    % quest 
    algs = {'QuEst_RANSAC_v0102'}; % pose estimation algorithms
    ransac_thresh = 1e-6; % ransac, sampson dist thresh
    surf_thresh = 200; % surf detector thresh
    maxPts = 30; % max features used in pose est (fewer features, faster compute)
    minPts = 8; % min feature required (6 to estimate a unique pose from RANSAC, or at least 8 for 8-pt algorithm)
  end

  methods
    function obj = init(obj)
      obj.outDir = strcat(obj.outDir,'out_',obj.test_ID,'/');
      obj.srcDir = strcat(obj.srcDir,name,'_df.csv');
    
    end

%     function outputArg = method1(obj,inputArg)
%       %METHOD1 Summary of this method goes here
%       %   Detailed explanation goes here
%       outputArg = obj.Property1 + inputArg;
%     end
  end
end