classdef qekf_class < matlab.System
  % STATE_CLASS defines a class of QEKF trackers that are to be
  % instantiated both per dataset/benchmark  and per pose est method.
  properties
    %% config (argin)
    alg_idx % unique QEKF filter per pose_alg
    TID
    ttag
    toutDir
    benchmark    
    benchnum
    pos_algs
    pos_numAlgs
    vel_algs
    vel_numAlgs
    st_sol
    %% QEKF config (argin)
    dim_x   = 9 % Txyz, Vxyz, Qxyz - linPos, linVel, rotVec (quat)
    dim_z   = 9 % Txyz, Vxyz, Qxyz - linPos, linVel, rotVec (quat)
    dim_u   = 3 % Axyz, Wrpy
    T_      = 0.1 % time period 
    Q_T_xyz     = 1.0e-5  % process noise covar
    Q_V_xyz     = 1.5e-2
    Q_quat_xyz  = 0.5e-9
    R_noise     = 1e-6 % measurement noise covar
    P_est_0     = 1e-4
    K_scale     = 1.0 % kalman gain factor   
    x_T_lim     = 50.0
    %% local vars 
    T_i 
    Q_i 
    V_i 
    W_i 
    u_Wrpy % control input vec 
    z_TVQw % measurement vec
    x_TVQxyz % state est vec 
    x_TVQw % state est vec log buff
    y_TVQw % residual vec
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

    function obj = qekf_class(varargin)
      setProperties(obj, nargin, varargin{:}) 
    end

    function load_cfg(obj, cfg) % load config from cfg
      obj.TID               = cfg.TID;
      obj.ttag              = cfg.ttag;
      obj.toutDir           = cfg.toutDir;
      obj.benchmark         = cfg.benchmark;
      obj.pos_algs          = cfg.pos_algs;
      obj.pos_numAlgs       = cfg.pos_numAlgs;
      obj.vel_algs          = cfg.vel_algs;
      obj.vel_numAlgs       = cfg.vel_numAlgs;
      obj.T_                = cfg.del_T;
      obj.T_i               = cfg.dat.posp_i.t1; % init conditions 
      obj.Q_i(1:3,1)        = cfg.dat.posp_i.q1(2:end,1);      
      obj.Q_i(4,1)          = cfg.dat.posp_i.q1(1,1);      
      obj.V_i               = zeros(3, 1);
      obj.W_i               = zeros(3, 1);
      obj.st_sol            = cell(7,0);
      obj.init();
    end 

  end % methods % constructor 
  methods (Access = public) 

    function state = run_qekf(obj, TQVW_sols, a)
      assert(strcmp(TQVW_sols{1,a}{1}, obj.pos_algs{obj.alg_idx}), ... 
        'alg mismatch!!');
      mthd          = TQVW_sols{1, a}{1}; % load 
      T             = TQVW_sols{2, a}; 
      Q             = TQVW_sols{3, a}; 
      V             = TQVW_sols{4, a}; 
      W             = TQVW_sols{5, a}; 
      obj.u_Wrpy    = W;
      obj.z_TVQw    = vertcat(T, V, Q(2:end), Q(1)); % copy all except w term 
      obj.x_TVQxyz  = obj.predict(obj.x_TVQxyz, obj.u_Wrpy); % run 
      obj.x_TVQxyz  = obj.update(obj.x_TVQxyz, obj.z_TVQw);
      state         = obj.get_st_sol(mthd); % save
    end 

    function st_sol = get_st_sol(obj, mthd)
      % load state sols alg,Z,U,X,Y,P,K 
      obj.st_sol{1} = mthd; 
      obj.st_sol{2} = obj.z_TVQw;
      obj.st_sol{3} = obj.u_Wrpy;
      obj.st_sol{4} = obj.x_TVQw;
      obj.st_sol{5} = obj.y_TVQw;
      obj.st_sol{6} = obj.P;
      obj.st_sol{7} = obj.K;
      st_sol = obj.st_sol;
      %disp('[qekf.get_st_sol]--> obj.x_TVQw'); disp(obj.x_TVQw);
    end 

    function init(obj)
      obj.x_TVQxyz  = vertcat(obj.T_i, obj.V_i, obj.Q_i(1:end-1)); 
      obj.x_TVQw    = zeros(obj.dim_x+1, 1); 
      obj.u_Wrpy    = zeros(obj.dim_u, 1);
      obj.z_TVQw    = zeros(obj.dim_z+1, 1);
      obj.y_TVQw    = zeros(obj.dim_x+1, 1);
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
      
    function x_TVQxyz = update(obj, x_TVQxyz, z_TVQxyzw)
      z_TVQxyz  = z_TVQxyzw(1:end-1,1);
      % ''' compute Kalman gain '''
      PHT       = obj.P * obj.H';
      obj.S     = obj.H * PHT + obj.R;
      obj.K     = PHT \ obj.S; % instead of PHT * inv(obj.S);
      obj.K     = obj.K .* obj.K_scale;
      % ''' lin part '''
      hx          = obj.H * x_TVQxyz;
      y_TVQ       = z_TVQxyz - hx; % TVWQxyz
      x_TVQ_post  = zeros(obj.dim_x,1);
      Ky          = obj.K * y_TVQ ;
      x_TVQ_post(1:6) = x_TVQxyz(1:6) + Ky(1:6);
      % ''' quat part '''
      x_q       = Qxyz2Q(x_TVQxyz(7:9,1));
      z_q       = Qxyz2Q(z_TVQxyz(7:9,1));
      y_PHI_Qwxyz  = z_q * qinv(x_q); % get quaternion error
      y_PHI_lmap  = log_map(y_PHI_Qwxyz); % get incremental rotation error 
      ky_PHI_lmap = obj.K(7:9,7:9) * y_PHI_lmap;
      x_q_corr  = exp_map(obj.T_ .* ky_PHI_lmap(1:3)); % quaternion correction
      x_q_corr  = Qxyz2Q(x_q_corr(1:3)); % return Quat object
      % eq(6) update quaternion
      x_q_post = x_q_corr * x_q; % wxyz format
      x_q_post = x_q_post.normalize();
      % update x and P
      x_TVQxyz(1:6) = x_TVQ_post(1:6);
      x_Qwxyz = x_q_post.compact(); 
      x_TVQxyz(7:9) = x_Qwxyz(2:4); % load quat xyz to x_post
      obj.x_TVQxyz = x_TVQxyz;
      I_KH  = obj.I_ - obj.K * obj.H;
      obj.P = I_KH * obj.P * I_KH' + obj.K * obj.R * obj.K';
      % ''' update X and Y '''
      obj.x_TVQw(1:end-1,1) = x_TVQxyz(:,1);
      obj.x_TVQw(7:10,1)    = Qxyz2Qxyzw(x_TVQxyz(7:9,1));
      [w,x,y,z]           = y_PHI_Qwxyz.parts();
      obj.y_TVQw(1:6,1)   = y_TVQ(1:6,1);  
      obj.y_TVQw(7:10,1)  = [x,y,z,w];
    end

    function x_TVQxyz = predict_x(obj, x_TVQxyz, u_Wrpy)
      x_T = x_TVQxyz(1:3) + obj.T_ .* x_TVQxyz(4:6);
      %while sqrt(sum(x_T.^2))>obj.x_T_lim
       % x_T = x_T./2;
       % disp("[qekf_class.predict_x]->> x_T divided in half...");
      %end
      %assert( sqrt(sum(x_T.^2))<obj.x_T_lim, ...
      %  "[qekf_class.predict_x]->> x_T is too large!");
      x_TVQxyz(1:3) = x_T;

      % est linVel
      x_TVQxyz(4:6) = x_TVQxyz(4:6);
      % eq(18) est incremental rotation (in quat) based on input angVel (Wrpy)
      u_Qxyzw = exp_map(obj.T_ .* u_Wrpy);
      q_u_Qwxyz = Qxyz2Q(u_Qxyzw(1:3));
      q_x_Qwxyz = Qxyz2Q(x_TVQxyz(7:9,1));
      q_x_Qwxyz = q_u_Qwxyz * q_x_Qwxyz;
      q_x_Qwxyz = q_x_Qwxyz.normalize();
      x_TVQxyz(7:9,1) = Q2Qxyz(q_x_Qwxyz);
    end

    function x_TVQxyz = predict(obj, x_TVQxyz, u_Wrpy)
      Qxyz      = x_TVQxyz(7:9,1);
      obj.C     = obj.get_C(Qxyz);
      obj.H     = obj.get_H(obj.C);
      obj.L     = obj.get_L(obj.C,obj.dim_x);
      obj.F     = obj.get_F(u_Wrpy,obj.dim_x,obj.T_);
      obj.x_TVQxyz  = obj.predict_x(x_TVQxyz,u_Wrpy);
      obj.x_TVQxyz  = x_TVQxyz;
      Q_k       = obj.T_ .* obj.F * obj.L * obj.Q_c * obj.L' * obj.F';
      obj.P     = obj.F * obj.P * obj.F' + Q_k;
    end

    function F = get_F(~, u_Wrpy, dim_x, T)
      F           = eye(dim_x);
      F(1:3,4:6)  = T .* eye(3);
      u_skw       = skw_sym(u_Wrpy);
      F(7:9,7:9)  = eye(3) - T .* u_skw;
    end
    
    function H = get_H(~, C)
      H(1:9,1:9)  = eye(9);
      H(1:3,1:3)  = C;
    end 
    
    function L = get_L(~, C, dim_x)
      L           = eye(dim_x);
      L(4:6,4:6)  = -C';
    end 
    
    function C = get_C(~, Qxyz)
      q    = Qxyz2Q(Qxyz);
      C    = quat2rotm(q);
    end
  end % methods (Access = private) % private functions
end

%% local functions
function qvec = Qxyz2Qxyzw(Qxyz) 
  q = Qxyz2Q(Qxyz);
  qvec = q.compact';
  qvec = [qvec(2:end,1); qvec(1)]; % return a 4x1 vec --> Qxyzw
  %disp(qvec);
end

function qvec = Q2Qxyz(q)
  q = q.normalize();
  qvec = q.compact;
  qvec = qvec(1,2:end)'; % return a 3x1 vector
  %disp(qvec);
end

function q = Qxyz2Q(Qxyz)
  w = sqrt(1 - Qxyz(1)^2 - Qxyz(2)^2 - Qxyz(3)^2);
  q = quaternion(w,Qxyz(1),Qxyz(2),Qxyz(3));
end

function x_sym = skw_sym(x)
  x_sym       = zeros(3,3);
  x_sym(1,2)  = -x(3);
  x_sym(1,3)  =  x(2);
  x_sym(2,1)  =  x(3);
  x_sym(2,3)  = -x(1);
  x_sym(3,1)  = -x(2);
  x_sym(3,2)  =  x(1);
end

function Qxyzw = exp_map(x)
  if isequal(size(x),[3,1])
    norm_x = norm(x);
    if norm_x == 0
      Qxyzw = [0,0,0,1];
    else
      qxyz = sin(norm_x/2) .* x/norm_x;
      qw = cos(norm_x/2);
      Qxyzw = [qxyz(1);qxyz(2);qxyz(3);qw];
    end
  end
end

function qi = qinv(q)
  q   = q.normalize();
  qi  = q.conj ./ (q * q.conj);
end 

function logq  = log_map(q) 
  % logarithmic map of a Quaternion
  q = q.normalize();
  [w,x,y,z] = q.parts();
  v = [x;y;z];
  qnorm = norm(v,1);
  if w > 1
    w = 1;
  end
  if w < -1
    w = -1;
  end
  if (qnorm~=0 && w>=0)
    logq = 2*acos(w) .* v/qnorm;
  elseif (qnorm~=0 && w<0)
    logq = -2*acos(-w) .* v/qnorm;
  elseif qnorm==0
    logq = zeros(3,1);
  end
end


