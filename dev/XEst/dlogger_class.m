classdef dlogger_class < matlab.System 
  properties
    %% features
    %log_prt_en        = false;
    %log_sav_en        = true;
    %log_csv_sav_en    = true;
    % config (argin)
    TID
    ttag
    toutDir
    benchmark
    pos_algs
    vel_algs
    pos_numMethods 
    vel_numMethods 
    log
    pfeat_logs % pfeat mod
    pos_logs
    vel_logs
    qekf_logs
  end
  methods  % constructor

    function obj = dlogger_class(varargin)
      setProperties(obj,nargin,varargin{:})
    end
  
  end % methods % constructor
  methods (Access = public) 

    function load_cfg(obj, cfg)
      obj.TID               = cfg.TID;
      obj.ttag              = cfg.ttag;
      obj.toutDir           = cfg.toutDir;
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

    function get_logs(obj)
      %obj.pfeat_logs = obj.log.get_qekf_logs();
      obj.pos_logs = obj.log.get_pos_logs();
      obj.vel_logs = obj.log.get_vel_logs();
      obj.qekf_logs = obj.log.get_qekf_logs();
    end 

  end % methods (Access = public) 
end
