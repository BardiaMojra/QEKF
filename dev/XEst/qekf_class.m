classdef qekf_class < matlab.System
  % STATE_CLASS defines a class of QEKF trackers that are to be
  % instantiated both per dataset/benchmark  and per pose est method.
  properties
    %% config (argin)
    alg_idx % unique QEKF filter per pose_alg
    test_ID
    test_outDir
    benchmark    
    benchnum
    pose_algorithms
    
    numMethods
    st_sol
    
   
    %% QEKF config (argin)
    dim_x = 9 % Txyz, Vxyz, Qxyz - linPos, linVel, rotVec (quat)
    dim_z = 9 % Txyz, Vxyz, Qxyz - linPos, linVel, rotVec (quat)
    dim_u = 3 % Axyz, Wrpy
    T_    = 0.1 % time period 
    Q_T_xyz     = 1.0e-5  % process noise covar
    Q_V_xyz     = 1.5e-2
    Q_quat_xyz  = 0.5e-3
    R_noise     = 1e-6 % measurement noise covar
    P_est_0     = 1e-4
    K_scale     = 1.0 % kalman gain factor   

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


    %ylabels = {'Tx','Ty','Tz','vx','vy','vz','qx','qy','qz','qw'};


     
  end
  methods  % constructor
    function obj = qekf_class(varargin)
      setProperties(obj, nargin, varargin{:}) 
    end

    function load_cfg(obj, cfg) % load config from cfg
      obj.test_ID           = cfg.test_ID;
      obj.test_outDir       = cfg.test_outDir;
      obj.benchmark         = cfg.benchmark;
      obj.pose_algorithms   = cfg.pose_algorithms;
      obj.numMethods        = cfg.numMethods;
      %  init conditions 
      obj.T_i       = cfg.dat.posp_i.t1;
      obj.Q_i       = cfg.dat.posp_i.q1;
      obj.V_i       = zeros(3, 1);
      obj.W_i       = zeros(3, 1);

      obj.st_sol    = cell(7,0);
      obj.init();
    end 
  end % methods % constructor 

  methods (Access = public) 

    function st_sol = run_qekf(obj, TQVW_sols, alg)
      assert(strcmp(TQVW_sols{1,alg}{1},obj.pose_algorithms{obj.alg_idx}), ... 
        'alg mismatch!!');
      mthd          = TQVW_sols{1, alg}{1}; % load 
      T             = TQVW_sols{2, alg}; 
      Q             = TQVW_sols{3, alg}; 
      V             = TQVW_sols{4, alg}; 
      W             = TQVW_sols{5, alg}; 
      obj.u_Wrpy    = W;
      obj.z_TVQw    = vertcat(T, V, Q(2:end), Q(1)); % copy all except w term 
      obj.x_TVQxyz  = obj.predict(obj.x_TVQxyz, obj.u_Wrpy); % run 
      obj.x_TVQxyz  = obj.update(obj.x_TVQxyz, obj.z_TVQw);
      st_sol        = obj.get_st_sol(mthd); % save
    end % function obj = run_qekf(obj, TQVW_sols, st_sols, alg)

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
    end 


    function init(obj)
      obj.x_TVQxyz  = vertcat(obj.T_i, obj.V_i, obj.Q_i(2:end)); 
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
      % eq(16-22) discrete form: \hat{x}_{k|k-1} = f(\hat{x}_{k-1|k-1}, u_{k}) 
      % est linPos
      x_TVQxyz(1:3) = x_TVQxyz(1:3) + obj.T_ .* x_TVQxyz(4:6);
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



    %function losses = get_losses(~, res)
    %  losses = zeros(height(res),2);
    %  for i = 1:height(res)
    %    state_l1 = 0.0;
    %    state_l2 = 0.0;
    %    for j = 1:width(res)-1 % ignore q.w term
    %      l1 = abs(res(i,j));
    %      l2 = res(i,j)^2;
    %      state_l1 = state_l1 + l1;
    %      state_l2 = state_l2 + l2;
    %    end
    %    losses(i,1) = state_l1;
    %    losses(i,2) = state_l2;
    %  end
    %  losses = array2table(losses,'VariableNames',{'L1','L2'});
    %  %head(losses);
    %  %res = [res, Losses];
    %  %if isstring(outDir)
    %  %  fname = strcat(outDir,'losses.txt');
    %  %  file = fopen(fname, 'a');
    %  %  mssg = strcat('---->> L1:',num2str(sum(L1)),'  L2:',num2str(sum(L2)),'\n');
    %  %  disp(mssg);
    %  %  fprintf(file,mssg);
    %  %end
    %end


  end % methods (Access = private) % private functions
end


%% local functions
%  
function [x,y,z] = Qxyz2Qxyzw(Qxyz)
  q = Qxyz2Q(Qxyz);
  [~,x,y,z] = q.parts();
end

function [x,y,z] = Q2Qxyz(q)
  q = q.normalize();
  [~,x,y,z] = q.parts();
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


