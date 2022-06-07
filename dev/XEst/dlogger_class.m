classdef dlogger_class < matlab.System 
  properties
    %% features
    res_prt_en  = true;
    res_sav_en  = true;
    % config (argin)
    test_ID
    test_outDir
    benchmark
    pos_algs
    vel_algs
    pos_numMethods 
    vel_numMethods 
    log
  
  end
  methods  % constructor

    function obj = dlogger_class(varargin)
      setProperties(obj,nargin,varargin{:})
    end
  
  end % methods % constructor
  methods (Access = public) 

    function load_cfg(obj, cfg)
      obj.test_ID           = cfg.test_ID;
      obj.test_outDir       = cfg.test_outDir;
      obj.benchmark         = cfg.benchmark;  
      obj.pos_numMethods    = cfg.pos_numMethods;
      obj.pos_algs          = cfg.pos_algs;  
      obj.vel_numMethods    = cfg.vel_numMethods;
      obj.vel_algs          = cfg.vel_algs;
      obj.log               = log_class(benchtype =  cfg.dat.benchtype, ... 
                                        kframes   =  cfg.dat.kframes, ...
                                        pos_algs  =  cfg.pos_algs, ...
                                        vel_algs  =  cfg.vel_algs );
      obj.log.load_cfg(cfg); 
    end
    
    function log_state(obj, cntr, frame_idx, TQVW_sols, state_sols)
      obj.log.log_state(cntr, frame_idx, TQVW_sols, state_sols);
    end

  end % methods (Access = public) 
end
