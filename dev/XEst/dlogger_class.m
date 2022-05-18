classdef dlogger_class < matlab.System 
  properties
    res_prt_en  = true;
    res_sav_en = true;
    % config (inarg)
    test_ID
    test_outDir
    numMethods 
    numBenchmarks 
    pose_algorithms  
    benchmarks 
    % private 
    logs % cell array log_class objs corresponding to each benchmark
  end

  methods  % constructor
    function obj = dlogger_class(varargin)
      setProperties(obj,nargin,varargin{:}) % init obj w name-value args
    end
  end % methods % constructor
  
  methods (Access = public) 
    function load_cfg(~, cfg)
      obj.test_ID                    = cfg.test_ID;
      obj.test_outDir             = cfg.test_outDir;
      obj.numMethods          = cfg.numMethods;
      obj.numBenchmarks   = cfg.numBenchmarks;
      obj.pose_algorithms   = cfg.pose_algorithms;
      obj.benchmarks           = cfg.benchmarks;      
      
      obj.logs                      = cell(obj.numBenchmarks,0);
      for b = 1:obj.numBenchmarks
        log = log_class();
        btype =  cfg.dats{b}.benchtype;
        kframes = cfg.dats{b}.keyFrames;
        disp(log);
        
%         disp(btype );
%         disp(kframes );
        log.benchtype = btype;
        log.keyFrames  = keyFrames;
%         load_cfg2log(cfg, btype, kframes); 
        obj.logs{b} = log;
      end



    end
    
    function log_state(obj, bnch, benchName, idx, frame_idx, T, Q, V, W)
      assert(strcmp(obj.logs{bnch},benchName), "dlog.log{%d}: %s  doesnt match benchName: %s", ...
        bench, obj.logs{bnch}, benchName);
      obj.log{bnch}.log_state(idx, frame_idx, T, Q, V, W);
    end
  end % methods (Access = public) 
 
%   methods (Access = private) 
%     function obj = init(obj)    
%     end
%   end % methods (Access = private) 

end
