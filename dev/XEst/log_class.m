classdef log_class < matlab.System
  properties
    % features 
    logs_sav_en     = true;
    % config (argin)
    TID     
    ttag    
    toutDir 
    benchtype  % single benchmark    
    kframes  % corresponding keyframes 
    pos_algs
    vel_algs
    % cfg (private constants)
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
    pfeat_logs % pfeat mod
    pos_logs
    vel_logs
    qekf_logs
    %% logs ------->> log objects 
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
    % Log Errs
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
      obj.TID            = cfg.TID;
      obj.ttag           = cfg.ttag;
      obj.toutDir        = cfg.toutDir;
      obj.pos_algs       = cfg.pos_algs;
      obj.vel_algs       = cfg.vel_algs;
      obj.init();
    end 
  
  end % methods  % constructor
  methods (Access = public) 

    function log_state(obj, cntr, kfi, TQVW_sols, st_sols)
      obj.cntr_hist(cntr, 1)    = cntr;
      obj.kf_hist(cntr, 1)      = kfi;
      for a = 1:length(obj.pos_algs) % log pose algs
        assert(strcmp(obj.pos_algs{a}, TQVW_sols{1, a}{1}), ... 
          "[log_class.log_state()]--> alg mismatch"); 
        obj.T_hist{cntr,a}  = TQVW_sols{2,a}; % quest
        obj.Q_hist{cntr,a}  = TQVW_sols{3,a};
        obj.V_hist{cntr,1}  = TQVW_sols{4,1}; % vest
        obj.W_hist{cntr,1}  = TQVW_sols{5,1};
        obj.Z_hist{cntr,a}  = st_sols{2,a}; % qekf
        obj.U_hist{cntr,a}  = st_sols{3,a};
        obj.X_hist{cntr,a}  = st_sols{4,a};
        obj.Y_hist{cntr,a}  = st_sols{5,a};
        obj.P_hist{cntr,a}  = st_sols{6,a};
        obj.K_hist{cntr,a}  = st_sols{7,a};
      end
    end % function log_state(obj, cntr, frame_idx, TQVW_sols, st_sols)

    function pos_logs = get_pos_logs(obj)
      pos_logs = cell(2, obj.pos_numAlgs); 
      cnts = obj.cntr_hist;
      kfs = obj.kf_hist;
      assert(isequal(length(cnts),length(kfs)), ...
        "[log.get_pos_logs]-->> cnts n kfs not equal in length!");
      for a = 1:obj.pos_numAlgs % logs per pos alg
        pos_logs{1, a} = obj.pos_algs{a};
        T = obj.T_hist{cntr,a};
        Q = obj.Q_hist{cntr,a};
        TQ = nan(obj.numKF, 2 + obj.d_T+obj.d_Q); % cntr, kfi, Txyz, Qwxyz
        TQ(:,1) = cnts(:,1);
        TQ(:,2) = kfs(:,1);
        for i = 1:length(cnts)
          TQ(i,3:5) = T(1:3,1)';
          TQ(i,6:8) = Q(1:3,1)';
        end
        if obj.logs_sav_en
          fname = strcat(obj.toutDir,"log_pos_",obj.pos_algs{a},".csv");
          writematrix(TQ, fname); 
        end
      end % for pos_numAlgs
      obj.pos_logs = pos_logs;
    end % 

    
    function get_qekf_logs(obj)
      %obj.get_qekf_X_logs();
      %obj.get_qekf_Y_logs();
    end 

    function get_qekf_X_logs(obj)
      X = nan(obj.numKF, obj.d_X); % Txyz Vxyz Qxyzw
      for a = 1:obj.pos_numAlgs % create logs per pos alg
        Vcols   = get_cols(a, obj.d_V); % --->> get var cols
        X(:,1) = obj.V_hist(:, Vcols(1)); % Vxyz
        X(:,2) = obj.V_hist(:, Vcols(2));
        X(:,3) = obj.V_hist(:, Vcols(3));
        X(:,4) = obj.W_hist(:, Wcols(1)); % Wrpy
        X(:,5) = obj.W_hist(:, Wcols(2));
        X(:,6) = obj.W_hist(:, Wcols(3));
        %disp(VW);
        fname = strcat(obj.toutDir,"log_vel_",obj.vel_algs{a},".csv");
        writematrix(X, fname);  
      end % for
    end % 

  end % methods (Access = public) % public functions
  methods (Access = private) % private functions
    
    function init(obj) 
      % consider changing to matrices instead of cells, (matrix runs faster)
      obj.pos_numAlgs         = length(obj.pos_algs);
      obj.vel_numAlgs         = length(obj.vel_algs);
      obj.numKF               = length(obj.kframes);
      obj.cntr_hist           = NaN(obj.numKF,1);
      obj.kf_hist             = NaN(obj.numKF,1);
      %% --->> logs
      obj.T_hist              = cell(obj.numKF,obj.pos_numAlgs); % quest
      obj.Q_hist              = cell(obj.numKF,obj.pos_numAlgs); 
      obj.V_hist              = cell(obj.numKF,obj.vel_numAlgs); % vest
      obj.W_hist              = cell(obj.numKF,obj.vel_numAlgs);
      obj.Z_hist              = cell(obj.numKF,obj.pos_numAlgs); % qekf
      obj.U_hist              = cell(obj.numKF,obj.pos_numAlgs); 
      obj.X_hist              = cell(obj.numKF,obj.pos_numAlgs); 
      obj.Y_hist              = cell(obj.numKF,obj.pos_numAlgs); 
      obj.P_hist              = cell(obj.numKF,obj.pos_numAlgs); 
      obj.K_hist              = cell(obj.numKF,obj.pos_numAlgs); 
      %% --->> errs    
      obj.T_errs              = NaN(obj.numKF,obj.pos_numAlgs); % quest
      obj.Q_errs              = NaN(obj.numKF,obj.pos_numAlgs); 
      obj.VEst_T_errs         = NaN(obj.numKF,obj.vel_numAlgs); % vest
      obj.VEst_Q_errs         = NaN(obj.numKF,obj.vel_numAlgs); 
    end % function init(obj)


  end % methods (Access = private) % private functions
end