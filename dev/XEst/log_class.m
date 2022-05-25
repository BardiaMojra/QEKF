classdef log_class < matlab.System
  properties
    % config (argin)
    benchtype  % single benchmark    
    keyFrames  % corresponding keyframes 
    algorithms  

    % private
    numKeyFrames
    numMethods 

    %% datalog arrays for each module
%     log_labels = { ...
    cntr_hist
    frame_hist
    % quest 
    T_hist % recovered translations
    Q_hist % recovered quaternions 
    % vest 
    V_hist
    W_hist 
    % qekf Z,U,X,Y,P,K 
    Z_hist
    U_hist
    X_hist
    Y_hist
    P_hist
    K_hist

    %% Log Errs
    % init in init() but always compute and fill at post-processing 
    T_errs % trans err for each method
    Q_errs % rot err for each method
    V_errs
    W_errs

    %% qekf
    % state - gt
    x_t_errs % st trans est errs
    x_q_errs % st rot est errs
    x_v_errs % st vel est errs
    % meas - gt
    z_t_errs % meas trans errs
    z_v_errs % meas rot errs
    z_q_errs % meas vel errs
    % L1 residual
    y_t_L1 % st trans L1 res
    y_q_L1 % st rot L1 res
    y_v_L1 % st vel L1 res
    % L2 residual
    y_t_L2 % st trans L2 res
    y_q_L2 % st rot L2 res
    y_v_L2 % st vel L2 res
  
  end

  methods  % constructor
    function obj = log_class(varargin)
      setProperties(obj,nargin,varargin{:}) % init obj w name-value args
    end
    function load_cfg(obj, cfg)
      % load config from cfg
      obj.algorithms       = cfg.pose_algorithms;
      obj.init();
    end 
  end 

  methods (Access = public) 

    function log_state(obj, cntr, frame_idx, TQVW_sols, st_sols)
      obj.cntr_hist(cntr, 1)     = cntr;
      obj.frame_hist(cntr, 1)    = frame_idx;

      for alg = 1:length(obj.algorithms)
        assert(strcmp(obj.algorithms{alg}, TQVW_sols{1, alg}{1}), ... 
          "[log_class.log_state()]--> alg mismatch"); 

        obj.T_hist{cntr, alg}      = TQVW_sols{2, alg}; % quest
        obj.Q_hist{cntr, alg}      = TQVW_sols{3, alg};

        obj.V_hist{cntr, alg}      = TQVW_sols{4, alg}; % vest
        obj.W_hist{cntr, alg}      = TQVW_sols{5, alg};

        obj.Z_hist{cntr, alg}      = st_sols{2, alg}; % qekf
        obj.U_hist{cntr, alg}      = st_sols{3, alg};
        obj.X_hist{cntr, alg}      = st_sols{4, alg};
        obj.Y_hist{cntr, alg}      = st_sols{5, alg};
        obj.P_hist{cntr, alg}      = st_sols{6, alg};
        obj.K_hist{cntr, alg}      = st_sols{7, alg};

      end
    end % function log_state(obj, benchName, cntr, frame_idx, T, Q, V, W)
  end % methods (Access = public) % public functions
  methods (Access = private) % private functions
    
    function init(obj) 
      % consider changing to matrices instead of cells, (matrix runs
      % faster)
      obj.numMethods      = length(obj.algorithms);
      obj.numKeyFrames    = length(obj.keyFrames);
      obj.cntr_hist       = NaN(obj.numKeyFrames,1);
      obj.frame_hist      = NaN(obj.numKeyFrames,1);
      %% log 
      % quest
      obj.T_hist          = cell(obj.numKeyFrames,obj.numMethods);
      obj.Q_hist          = cell(obj.numKeyFrames,obj.numMethods); 
      % vest
      obj.V_hist          = cell(obj.numKeyFrames,obj.numMethods); 
      obj.W_hist          = cell(obj.numKeyFrames,obj.numMethods);
      % qekf
      obj.Z_hist          = cell(obj.numKeyFrames,obj.numMethods); 
      obj.U_hist          = cell(obj.numKeyFrames,obj.numMethods); 
      obj.X_hist          = cell(obj.numKeyFrames,obj.numMethods); 
      obj.Y_hist          = cell(obj.numKeyFrames,obj.numMethods); 
      obj.P_hist          = cell(obj.numKeyFrames,obj.numMethods); 
      obj.K_hist          = cell(obj.numKeyFrames,obj.numMethods); 


      % quest
      obj.T_errs          = NaN(obj.numKeyFrames,obj.numMethods); 
      obj.Q_errs          = NaN(obj.numKeyFrames,obj.numMethods); 
      % vest
      obj.V_errs          = NaN(obj.numKeyFrames,obj.numMethods); 
      obj.W_errs          = NaN(obj.numKeyFrames,obj.numMethods); 
      % qekf

    end % init(obj)
  end % methods (Access = private) % private functions
end