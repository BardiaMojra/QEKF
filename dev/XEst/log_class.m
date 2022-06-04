classdef log_class < matlab.System
  properties
    %% config (argin)
    benchtype  % single benchmark    
    kFrames  % corresponding keyframes 
    pos_algs
    vel_algs
    %% config
    d_T = 3
    d_Q = 4
    d_V = 3
    d_W = 3
    d_Z = 10
    d_U = 3
    d_X = 10
    d_Y = 10
    %d_P 
    %d_K
    %% private vars
    numKF
    pos_numAlgs
    vel_numAlgs
    cntr_hist
    kf_hist 
    T_hist % quest
    Q_hist 
    V_hist % vest
    W_hist 
    Z_hist % qekf
    U_hist
    X_hist
    Y_hist
    P_hist
    K_hist
    %% Log Errs
    % init in init() but always compute and fill at post-processing 
    T_errs % trans err for each method
    Q_errs % rot err for each method
    VEst_T_errs % based on VEst V .* delta_t
    VEst_Q_errs % based on VEst exp_map(W)
    % qekf
    % gt - x
    x_t_errs % st trans est errs
    x_q_errs % st rot est errs
    x_v_errs % st vel est errs
    % gt - z 
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
    
    function obj = log_class(varargin) % init obj w name-value args
      setProperties(obj,nargin,varargin{:}) 
    end
    
    function load_cfg(obj, cfg) % load config from cfg
      obj.pos_algs       = cfg.pos_algs;
      obj.vel_algs       = cfg.vel_algs;
      obj.init();
    end 
  
  end % methods  % constructor
  methods (Access = public) 

    function log_state(obj, cntr, frame_idx, TQVW_sols, st_sols)
      obj.cntr_hist(cntr, 1)     = cntr;
      obj.kf_hist(cntr, 1)    = frame_idx;
      for a = 1:length(obj.pos_algs) % log pose algs
        assert(strcmp(obj.pos_algs{a}, TQVW_sols{1, a}{1}), ... 
          "[log_class.log_state()]--> alg mismatch"); 
        obj.T_hist(cntr,((a-1)*obj.d_T)+1:((a-1)*obj.d_T)+obj.d_T)=TQVW_sols{2,a}; % quest
        obj.Q_hist(cntr,((a-1)*obj.d_Q)+1:((a-1)*obj.d_Q)+obj.d_Q)=TQVW_sols{3,a};
        obj.V_hist(cntr,1:obj.d_V)                                =TQVW_sols{4,end}; % vest
        obj.W_hist(cntr,1:obj.d_W)                                =TQVW_sols{5,end};
        obj.Z_hist(cntr,((a-1)*obj.d_Z)+1:((a-1)*obj.d_Z)+obj.d_Z)=st_sols{2,a}; % qekf
        obj.U_hist(cntr,((a-1)*obj.d_U)+1:((a-1)*obj.d_U)+obj.d_U)=st_sols{3,a};
        obj.X_hist(cntr,((a-1)*obj.d_X)+1:((a-1)*obj.d_X)+obj.d_X)=st_sols{4,a};
        obj.Y_hist(cntr,((a-1)*obj.d_Y)+1:((a-1)*obj.d_Y)+obj.d_Y)=st_sols{5,a};
        obj.P_hist{cntr, a}      = st_sols{6, a};
        obj.K_hist{cntr, a}      = st_sols{7, a};
      end
    end % function log_state(obj, cntr, frame_idx, TQVW_sols, st_sols)
  
  end % methods (Access = public) % public functions
  methods (Access = private) % private functions
    
    function init(obj) 
      % consider changing to matrices instead of cells, (matrix runs faster)
      obj.pos_numAlgs         = length(obj.pos_algs);
      obj.vel_numAlgs         = length(obj.vel_algs);
      obj.numKF               = length(obj.kFrames);
      obj.cntr_hist           = NaN(obj.numKF,1);
      obj.kf_hist             = NaN(obj.numKF,1);
      %% --->> logs
      obj.T_hist              = NaN(obj.numKF,obj.pos_numAlgs * obj.d_T); % quest
      obj.Q_hist              = NaN(obj.numKF,obj.pos_numAlgs * obj.d_Q); 
      obj.V_hist              = NaN(obj.numKF,obj.vel_numAlgs * obj.d_V); % vest
      obj.W_hist              = NaN(obj.numKF,obj.vel_numAlgs * obj.d_W);
      obj.Z_hist              = NaN(obj.numKF,obj.pos_numAlgs * obj.d_Z); % qekf
      obj.U_hist              = NaN(obj.numKF,obj.pos_numAlgs * obj.d_U); 
      obj.X_hist              = NaN(obj.numKF,obj.pos_numAlgs * obj.d_X); 
      obj.Y_hist              = NaN(obj.numKF,obj.pos_numAlgs * obj.d_Y); 
      obj.P_hist              = cell(obj.numKF,obj.pos_numAlgs); 
      obj.K_hist              = cell(obj.numKF,obj.pos_numAlgs); 
      %% --->> errs    
      obj.T_errs              = NaN(obj.numKF,obj.pos_numAlgs); % quest
      obj.Q_errs              = NaN(obj.numKF,obj.pos_numAlgs); 
      obj.VEst_T_errs         = NaN(obj.numKF,obj.vel_numAlgs); % vest
      obj.VEst_Q_errs         = NaN(obj.numKF,obj.vel_numAlgs); 
      % qekf
      % not needed???

    end % function init(obj)
  end % methods (Access = private) % private functions
end