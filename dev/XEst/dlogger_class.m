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
    function load_cfg(obj, cfg)
      obj.test_ID                    = cfg.test_ID;
      obj.test_outDir             = cfg.test_outDir;
      obj.numMethods          = cfg.numMethods;
      obj.numBenchmarks   = cfg.numBenchmarks;
      obj.pose_algorithms   = cfg.pose_algorithms;
      obj.benchmarks           = cfg.benchmarks;      
      obj.logs                      = cell(obj.numBenchmarks,0);
      for b = 1:obj.numBenchmarks
        obj.logs{b} = log_class( benchtype   =  cfg.dats{b}.benchtype, ... 
                                                  keyFrames  =  cfg.dats{b}.keyFrames, ...
                                                  algorithms   =  cfg.pose_algorithms );
        obj.logs{b}.load_cfg(cfg); 
      end
    end
    
    function log_state(obj, bnch, benchName, idx, frame_idx, TQ_sols, V, W)
      % check log BenchName and current benchtype

      msg = sprintf("dlog.log{%d}: %s  DOES NOT match benchName: %s", bnch, ...
        obj.logs{bnch}.benchtype{1}, benchName);
      %strcmp(obj.logs{bnch}.benchtype{1}, benchName)
      %disp(class(obj.logs{bnch}.benchtype{1}));
      %disp(class(benchName));
      assert(strcmp(obj.logs{bnch}.benchtype{1}, benchName), msg);
        

      obj.logs{bnch}.log_state(idx, frame_idx, TQ_sols, V, W);
    end
  end % methods (Access = public) 
 
%   methods (Access = private) 
%     function obj = init(obj)    
%     end
%   end % methods (Access = private) 

end
