classdef qekf_class < matlab.System 
  properties % public vars
    % features 
    
    % cfg argin
    test_ID
    test_outDir
    benchmarks
    algorithms

    %% QEKF config
    dim_x = 9 % Txyz, Vxyz, Qxyz - linPos, linVel, rotVec (quat)
    dim_z = 9 % Txyz, Vxyz, Qxyz - linPos, linVel, rotVec (quat)
    dim_u = 6 % Axyz, Wrpy
    deltaT 
    Q_T_xyz         = 1.0e-5  % process noise covar
    Q_V_xyz         = 1.5e-2
    Q_quat_xyz    = 0.5e-3
    R_noise          = 1e-6 % measurement noise covar
    P_est_0           = 1e-4
    IC  %  init conditions 
    K_scale           = 1.0 % kalman gain factor

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
    
      for alg = 1: length(obj.algorithms)
        method = obj.algorithms{alg};
        u_Wrpy, obj.get_method_est(Q, T, alg);
    end 

  end % methods (Access = public) 

  methods (Access = private)
      
    function init(obj)
      obj.numBenchmarks    = length(obj.benchmarks);
      obj.numMethods           = length(obj.algorithms);
    end 
    
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
