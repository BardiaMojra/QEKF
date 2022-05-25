classdef qekf_handler_class < matlab.System 
  properties % public vars
    % features 
    
    % cfg argin    
    test_ID
    test_outDir
    benchmark
    pose_algorithms
    numMethods

  end

  methods % constructor
    function obj = qekf_handler_class(varargin) 
      setProperties(obj, nargin, varargin{:}) % init obj w name-value args
    end  
  end % methods % constructor
  
  methods (Access = public) 
    function load_cfg(obj, cfg)
      obj.test_ID           = cfg.test_ID;                   
      obj.test_outDir       = cfg.test_outDir;       
      obj.benchmark         = cfg.benchmark;
      obj.pose_algorithms   = cfg.pose_algorithms;
      obj.numMethods        = length(obj.pose_algorithms);
      obj.trackers          = cell(obj.numMethods, 0);
      obj.st_sols           = cell(7,obj.numMethods); % alg,Z,U,X,Y,P,K 
      for alg = 1:obj.numMethods
        obj.trackers{alg} = state_class( alg_idx = alg );
        obj.trackers{alg}.load_cfg(cfg);
      end
    end

    function st_sols = run_filter(obj, TQVW_sols)
      for alg = 1:obj.numMethods
        obj.st_sols{alg} = obj.trackers{alg}.run_qekf(TQVW_sols, alg);
      end 
      st_sols = obj.st_sols;
    end 



  end % methods (Access = public) 
  
  methods (Access = private)      
    function stats = get_stats(~, errs) 
      stats     =  zeros(5, size(errs,2));
      stats(1, :) = mean(errs,1);  % Mean
      stats(2, :) = std(errs,1);  % Standard deviation
      stats(3, :) = median(errs,1); % Median
      stats(4, :) = quantile(errs, 0.25, 1); % 25% quartile
      stats(5, :) = quantile(errs, 0.75, 1); % 75% quartile
    end
    
    function res_table = get_log_res(obj, log, dat) % get per benchmark log errs 
      cntr = 0;
      for f = dat.keyFrames
        cntr = cntr + 1;
        for alg = 1:length(log.algorithms) % calc and save errs per method
          q    = log.Q_hist{cntr, alg};
          w    = log.W_hist{cntr, alg};
          if isequal(size(w), [3,1]) 
            log.W_errs(cntr, alg)     = obj.cmp_exp_map_w_Q(w, q);
          end
        end % for alg = length(log.algorithms)
      end % for f = kframes
      W_stats = obj.get_stats(log.W_errs);
      res_table  = obj.get_res_table(W_stats);
    end % function get_log_errs(log, dat) 
  end % methods ( Access = private)
end
