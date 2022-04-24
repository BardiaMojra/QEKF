classdef qekf_class
  %qekf_class qekf class holds related configs and data structures 
  %   detailed explanation goes here
  properties
    % config (pass in to init internal configs)
    dset % dataset obj 
    dim_x % state est/bel vec dim
    dim_z % state meas/obs vec dim
    dim_u % state input/comt vec dim
    T_ = 0.1; % time period 
    Q_T_xyz % process noise covar
    Q_V_xyz %
    Q_quat_xyz %
    R_noise % measurement noise covar
    P_est_0 % init condition posterior noise covar
    IC % init condition state vec 
    k_scale = 1.0; % linear gain factor for manual tuning 
    % internal data structs 
    x_TVQxyz % state est vec
    p % posterior noise covariance
    F % state transition matrix
    % y_TVQxyz % residual vec
    K % kalman gain vec
    S % system uncertainty
    L % state jacobian matrix
    I_ % state-vec-sized eye matrix
    C % state rotation matrix
    H % observation jacobian matrix
    Q_c % process noise covar matrix
    R % measurement noise covar matrix
    log_en = true; % data logger flag
    log % datalog obj
    
  end
  methods
    function obj = qekf_class(dset, ...
                              dlog, ...
                              dim_x, ...
                              dim_z, ...
                              dim_u, ...
                              Q_T_xyz, ...
                              Q_V_xyz, ...
                              Q_quat_xyz, ...
                              R_noise, ...
                              P_est_0 ...
                              )
      if nargin > 0
        obj.dset       = dset;
        obj.log        = dlog;
        obj.dim_x      = dim_x;
        obj.dim_z      = dim_z;
        obj.dim_u      = dim_u;
        obj.T_         = T_;
        obj.Q_T_xyz    = Q_T_xyz;
        obj.Q_V_xyz    = Q_V_xyz;
        obj.Q_quat_xyz = Q_quat_xyz;
        obj.R_noise    = R_noise;
        obj.P_est_0    = P_est_0;
        % state vectors
        obj.x_TVQxyz   = obj.dset.dat(1,:); % set state initial conditions
        obj.P         = eye(dim_x) .* P_est_0;
        obj.F         = eye(dim_x);
        % obj.y_TVQxyz % residual vector
        obj.T_        = T_; 
        obj.K         = zeros(dim_x,1);
        obj.S         = zeros(dim_z,dim_z); 
        obj.L         = eye(dim_x);
        obj.I_        = eye(dim_x);
        obj.C         = zeros(3,3); 
        obj.H         = zeros(dim_z,dim_x); 
        obj.Q_c       = diag([Q_T_xyz, ...
                              Q_T_xyz, ...
                              Q_T_xyz, ...
                              Q_V_xyz, ...
                              Q_V_xyz, ...
                              Q_V_xyz, ...
                              Q_quat_xyz, ...
                              Q_quat_xyz, ...
                              Q_quat_xyz]);
        obj.R         = diag([R_noise, ...
                              R_noise, ...
                              R_noise, ...
                              R_noise, ...
                              R_noise, ...
                              R_noise, ...
                              R_noise, ...
                              R_noise, ...
                              R_noise]);
%         obj.log_en = True;
%         obj.log;
      end
    end
  end
end