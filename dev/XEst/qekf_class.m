classdef qekf_class < matlab.System 
  properties % public vars
    % features 
    
    % cfg argin
    test_ID
    test_outDir
    benchmarks
    algorithms
    
    %% private 
    numMethods
    numBenchmarks
    % run-time variables 
    
    
    %% private constants 
    RowNames  = {'ang vel err mean     ';
                              'ang vel err std         ';
                              'ang vel err median  '; 
                              'ang vel err Q_1        ';
                              'ang vel err Q_3        ';
                              };
   
  end
  methods % constructor
    function obj = vest_class(varargin) 
      setProperties(obj,nargin,varargin{:}) % init obj w name-value args
    end  
  end % methods % constructor
  methods (Access = public) 
    function load_cfg(obj, cfg)
      obj.test_ID                   =  cfg.test_ID;                   
      obj.test_outDir            =  cfg.test_outDir;       
      obj.benchmarks          = cfg.benchmarks;
      obj.algorithms             =  cfg.pose_algorithms;
      obj.init();
    end
    function state = get_state(obj, TQ_sols, V, W)
  
    end 

  end % methods (Access = public) 
end
