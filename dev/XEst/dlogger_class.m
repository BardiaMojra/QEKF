classdef dlogger_class < matlab.System 
  properties
    res_prt_en  = true;
    res_sav_en = true;
    % config (inarg)
    test_ID
    test_outDir
    numMethods 
    pose_algorithms  
    benchmark
    log
  end

  methods  % constructor
    function obj = dlogger_class(varargin)
      setProperties(obj,nargin,varargin{:}) % init obj w name-value args
    end
  end % methods % constructor
  
  methods (Access = public) 
    function load_cfg(obj, cfg)
      obj.test_ID           = cfg.test_ID;
      obj.test_outDir       = cfg.test_outDir;
      obj.numMethods        = cfg.numMethods;
      obj.pose_algorithms   = cfg.pose_algorithms;
      obj.benchmark         = cfg.benchmark;      
      obj.log  = log_class(benchtype   =  cfg.dat.benchtype, ... 
                           keyFrames   =  cfg.dat.keyFrames, ...
                           algorithms  =  cfg.pose_algorithms );
      obj.log.load_cfg(cfg); 
    end
    
    function log_state(obj, cntr, frame_idx, TQVW_sols, state_sols)
      obj.log.log_state(cntr, frame_idx, TQVW_sols, state_sols);
    end
  end % methods (Access = public) 
end
