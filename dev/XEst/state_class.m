classdef state_class < matlab.System
  % STATE_CLASS defines a class of QEKF trackers that are to be
  % instantiated both per dataset/benchmark  and per pose est method.
  properties
    %% module config (argin)
    alg_idx % unique QEKF filter per pose_alg
    
    test_ID
    test_outDir
    benchtype    
    benchnum
    pose_algorithm
    T_i 
    Q_i 
    V_i 
    W_i 
    

    %% QEKF config (argin)
 
  %end
  %properties (Access = private)
    %% state (private)
    
    st_Z
    st_U
    st_X % state est vec x_TVQxyz 
     
    % run-time variables 
    x_TVQxyz % state est vec
    y_TVQxyz % residual vec
    P % posterior noise covariance
    F % state transition matrix
    K % kalman gain vec
    S % system uncertainty
    L % state jacobian matrix
    I_ % state-vec-sized eye matrix
    C % state rotation matrix
    H % observation jacobian matrix
    Q_c % process noise covar matrix
    R % measurement noise covar matrix
  end
  methods  % constructor
    function obj = state_class(varargin)
      setProperties(obj, nargin, varargin{:}) 
    end

    function load_cfg(obj, qekf, cfg) % load config from cfg
      obj.test_ID           = qekf.test_ID;
      obj.test_outDir       = qekf.test_outDir;
      obj.benchmark         = qekf.benchmark;
      obj.pose_algorithms   = qekf.pose_algorithms;
      obj.numMethods        = qekf.numMethods;
      %  init conditions 
      obj.T_i = cfg.dat.posp_i.t1;
      obj.Q_i = cfg.dat.posp_i.q1;
      obj.V_i = zeros(3, 1);
      obj.W_i = zeros(3, 1);
      obj.init();
    end 

  end % methods % constructor 
  methods (Access = public) 
    function update_state(obj, cntr, frame_idx, QT_sols, V, W)
     
    
    end % function log_state(obj, benchName, cntr, frame_idx, T, Q, V, W)
  
  end % methods (Access = public) % public functions
  
  methods (Access = private) % private functions
    function init(obj)
      disp(obj.T_i);
      disp(obj.Q_i);
      disp(obj.V_i);
      disp(obj.W_i);


     obj.x_TVQxyz  = vertcat(obj.T_i, ... 
                             obj.V_i, ...
                             obj.Q_i)'; 
      obj.P         = eye(obj.dim_x) .* obj.P_est_0;
      obj.F         = eye(obj.dim_x);
      obj.K         = zeros(obj.dim_x, 1);
      obj.S         = zeros(obj.dim_z, obj.dim_z); 
      obj.L         = eye(obj.dim_x);
      obj.I_        = eye(obj.dim_x);
      obj.C         = zeros(3,3); 
      obj.H         = zeros(obj.dim_z, obj.dim_x); 
      obj.Q_c       = diag([obj.Q_T_xyz, ...
                            obj.Q_T_xyz, ...
                            obj.Q_T_xyz, ...
                            obj.Q_V_xyz, ...
                            obj.Q_V_xyz, ...
                            obj.Q_V_xyz, ...
                            obj.Q_quat_xyz, ...
                            obj.Q_quat_xyz, ...
                            obj.Q_quat_xyz]);
      obj.R         = diag([obj.R_noise, ...
                            obj.R_noise, ...
                            obj.R_noise, ...
                            obj.R_noise, ...
                            obj.R_noise, ...
                            obj.R_noise, ...
                            obj.R_noise, ...
                            obj.R_noise, ...
                            obj.R_noise]);
    end % init(obj)
  end % methods (Access = private) % private functions
end