classdef qekf_class < matlab.System 
  properties % public vars
    % features 
    
    % cfg argin    

    test_ID
    test_outDir
    benchmark
    pose_algorithms
    numMethods
    
    %% QEKF config
    dim_x = 9 % Txyz, Vxyz, Qxyz - linPos, linVel, rotVec (quat)
    dim_z = 9 % Txyz, Vxyz, Qxyz - linPos, linVel, rotVec (quat)
    dim_u = 6 % Axyz, Wrpy
    T_    = 0.1 % time period 
    Q_T_xyz     = 1.0e-5  % process noise covar
    Q_V_xyz     = 1.5e-2
    Q_quat_xyz  = 0.5e-3
    R_noise     = 1e-6 % measurement noise covar
    P_est_0     = 1e-4
    K_scale     = 1.0 % kalman gain factor    
    %% private 
    % run-time variables 
    %x_TVQxyz % state est vec
    %y_TVQxyz % residual vec
    %P % posterior noise covariance
    %F % state transition matrix
    %K % kalman gain vec
    %S % system uncertainty
    %L % state jacobian matrix
    %I_ % state-vec-sized eye matrix
    %C % state rotation matrix
    %H % observation jacobian matrix
    %Q_c % process noise covar matrix
    %R % measurement noise covar matrix
    %% other private constants 
    RowNames  = {'Ang vel err mean';
                 'Ang vel err std';
                 'Ang vel err median'; 
                 'Ang vel err Q_1';
                 'Ang vel err Q_3';
                                    };
    ylabels = {'Tx','Ty','Tz','vx','vy','vz','qx','qy','qz','qw'};
  end
  methods % constructor
    function obj = vest_class(varargin) 
      setProperties(obj, nargin, varargin{:}) % init obj w name-value args
    end  
  end % methods % constructor
  methods (Access = public) 
    function load_cfg(obj, cfg)
      obj.test_ID               = cfg.test_ID;                   
      obj.test_outDir           = cfg.test_outDir;       
      
      obj.benchmark             = cfg.benchmark;
      obj.pose_algorithms       = cfg.pose_algorithms;
      obj.numMethods            = cfg.numMethods;
      
      for alg = 1:obj.numMethods
        obj.state{} = state_class( alg_idx = alg );
        obj.state
      end
      
      obj.init();
    end

    function state_sols = get_state(obj, TQVW_sols, state_sols)
      for alg = 1: length(obj.algorithms)
        [u_Wrpy, x_TVQxyz] = obj.get_method_est(TQVW_sols, state_sols, alg);
      end 
    end 

  end % methods (Access = public) 

  methods (Access = private)      
    function init(obj)
      obj.numMethods       = length(obj.algorithms);

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
