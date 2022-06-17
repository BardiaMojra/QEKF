classdef log_class < matlab.System
  properties
    % features 
    logs_sav_en     = true;
    sliding_ref_en  % cfg argin
    % config (argin)
    TID     
    ttag    
    toutDir 
    btype  % single benchmark    
    kframes  % corresponding keyframes 
    pos_algs
    vel_algs
    dat
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
      obj.btype          = cfg.benchmark;
      obj.pos_algs       = cfg.pos_algs;
      obj.vel_algs       = cfg.vel_algs;
      obj.sliding_ref_en = cfg.sliding_ref_en;
      obj.init();
    end 
  
  end % methods  % constructor
  methods (Access = public) 

    function log_state(obj, cntr, kfi, TQVW_sols, st_sols)
      obj.cntr_hist(cntr, 1)    = cntr;
      obj.kf_hist(cntr, 1)      = kfi;
      for a = 1:obj.pos_numAlgs % log pose algs
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
      pos_logs = cell(2, obj.pos_numAlgs+1); % poses + gt
      cnts = obj.cntr_hist;
      kfs = obj.kf_hist;
      assert(isequal(length(cnts),length(kfs)), ...
        "[log.get_pos_logs]-->> cnts n kfs not equal in length!");
      for a = 1:obj.pos_numAlgs+1% logs for pos_alg + gt
        if ~isequal(a, size(pos_logs, 2)) % -->> pos_algs
          pos_logs{1,a} = obj.pos_algs{a};
          TQ = nan(obj.numKF, 2 + obj.d_T+obj.d_Q); % cntr, kfi, Txyz, Qwxyz
          TQ(:,1) = cnts(:,1);
          TQ(:,2) = kfs(:,1);
          for c = 1:length(kfs)
            T = obj.T_hist{c,a};
            Q = obj.Q_hist{c,a};
            TQ(c,3:5) = T(1:3,1);
            TQ(c,6:9) = Q(1:4,1); 
          end
          pos_logs{2,a} = TQ;
          if obj.logs_sav_en % --->> save logs to file
            fname = strcat(obj.toutDir,"log_pos_",obj.pos_algs{a},".csv");
            writematrix(TQ, fname); 
          end
        else % -->> groundtruth 
          pos_logs{1,a} = "groundtruth";
          TQ = nan(obj.numKF, 2 + obj.d_T+obj.d_Q); % cntr, kfi, Txyz, Qwxyz          TQ(:,1) = cnts(:,1);
          TQ(:,1) = cnts(:,1);
          TQ(:,2) = kfs(:,1);
          qTru    = obj.dat.dataset.qTru;
          tTru    = obj.dat.dataset.tTru;
          q1      = obj.dat.posp_i.q1;
          t1      = obj.dat.posp_i.t1;
          for c = 1:length(kfs)
            f = kfs(c);
            [tr,qr,t2,q2] = get_relGT(f, obj.btype, tTru, qTru, t1, q1);
            TQ(c,3:5) = tr(:,1)';
            TQ(c,6:9) = qr(:,1)';
            if obj.sliding_ref_en % sliding window reference 
              q1 = q2; t1 = t2; 
            end
          end % kfs
          pos_logs{2,a} = TQ;
          if obj.logs_sav_en % --->> save logs to file
            fname = strcat(obj.toutDir,"log_pos_groundtruth.csv");
            writematrix(TQ, fname); 
          end
        end % get log table 
        %disp(pos_logs{1,a});
        %disp(TQ);
      end % for pos_alg + gt
      obj.pos_logs = pos_logs; 
    end % 

    function vel_logs = get_vel_logs(obj)
      vel_logs = cell(2, obj.vel_numAlgs); % vels (no groundtruth)
      cnts = obj.cntr_hist;
      kfs = obj.kf_hist;
      assert(isequal(length(cnts),length(kfs)), ...
        "[log.get_vel_logs]-->> cnts n kfs not equal in length!");
      for a = 1:obj.vel_numAlgs% logs for vel_alg
        vel_logs{1,a} = obj.vel_algs{a};
        VW = nan(obj.numKF, 2+obj.d_V+obj.d_W); % cntr, kfi, Vxyz, Wrpy
        VW(:,1) = cnts(:,1);
        VW(:,2) = kfs(:,1);
        for c = 1:length(kfs)
          V = obj.V_hist{c,a};
          W = obj.W_hist{c,a};
          %disp('W'); disp(W);
          VW(c,3:5) = V(:,1);
          VW(c,6:8) = W(:,1); 
        end
        vel_logs{2,a} = VW;
        if obj.logs_sav_en % --->> save logs to file
          fname = strcat(obj.toutDir,"log_vel_",obj.vel_algs{a},".csv");
          writematrix(VW, fname); 
        end
        %disp(vel_logs{1,a});
        %disp(VW);
      end % for vel_algs
      obj.vel_logs = vel_logs;
    end % 
    
    function qekf_logs = get_qekf_logs(obj)
      qekf_logs = cell(7, obj.pos_numAlgs+1); % rowNames + posAlgs
      qekf_logs{1,1} = "logs";
      qekf_logs{2,1} = "Z_log";
      qekf_logs{3,1} = "U_log";
      qekf_logs{4,1} = "X_log";
      qekf_logs{5,1} = "Y_log";
      qekf_logs{6,1} = "P_log";
      qekf_logs{7,1} = "K_log";
      for a = 1:obj.pos_numAlgs
        qekf_logs{1,a+1} = obj.pos_algs{a}; % --->> pos_alg
        qekf_logs{2,a+1} = obj.get_qekf_log(a,obj.Z_hist,"Z_hist");
        qekf_logs{3,a+1} = obj.get_qekf_log(a,obj.U_hist,"U_hist");
        qekf_logs{4,a+1} = obj.get_qekf_log(a,obj.X_hist,"X_hist"); 
        qekf_logs{5,a+1} = obj.get_qekf_log(a,obj.Y_hist,"Y_hist"); 
        %qekf_logs{6,a+1} = obj.get_qekf_log(a,obj.P_hist,"P_hist"); 
        %qekf_logs{7,a+1} = obj.get_qekf_log(a,obj.K_hist,"K_hist"); 
      end
      %disp(qekf_logs);
      obj.qekf_logs = qekf_logs;
    end 

    function  log = get_qekf_log(obj, a, log_hist, logName)
      cnts = obj.cntr_hist;
      kfs = obj.kf_hist;
      assert(isequal(length(cnts),length(kfs)), ...
        "[log.get_qekf_log]-->> cnts n kfs not equal in length!");
      %disp("size(log_hist{1,a},1)"); disp(size(log_hist{1,a},1));
      numRows = size(log_hist{1,a},1)+2;
      log = nan(obj.numKF, numRows); % cntr, kfi, log vec length
      log(:,1) = cnts(:,1);
      log(:,2) = kfs(:,1);
      for c = 1:obj.numKF
        log(c,3:end) = log_hist{c,a}';
      end
      %disp(log);
      fname = strcat(obj.toutDir,"log_qekf_",logName,"_",obj.pos_algs{a},".csv");
      writematrix(log, fname);  
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