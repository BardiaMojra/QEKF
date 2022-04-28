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
    log_en = true; % data logger flag
    log % datalog obj
  end
  methods
    function obj = config(obj, dset, dlog, ...
                          dim_x,dim_z,dim_u, ...
                          Q_T_xyz, ...
                          Q_V_xyz, ...
                          Q_quat_xyz, ...
                          R_noise, ...
                          P_est_0 ...
                          )
      obj.dset       = dset;
      obj.log        = dlog;
      obj.dim_x      = dim_x;
      obj.dim_z      = dim_z;
      obj.dim_u      = dim_u;
      obj.Q_T_xyz    = Q_T_xyz;
      obj.Q_V_xyz    = Q_V_xyz;
      obj.Q_quat_xyz = Q_quat_xyz;
      obj.R_noise    = R_noise;
      obj.P_est_0    = P_est_0;
      % state vectors
      obj.x_TVQxyz   = horzcat(dset.Txyz(1,:),dset.Vxyz(1,:),dset.Qxyzw(1,1:end-1))'; % set state initial conditions
      obj.P          = eye(dim_x) .* P_est_0;
      obj.F          = eye(dim_x);
      obj.K          = zeros(dim_x,1);
      obj.S          = zeros(dim_z,dim_z); 
      obj.L          = eye(dim_x);
      obj.I_         = eye(dim_x);
      obj.C          = zeros(3,3); 
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
    end

    function [obj, x_TVQxyz] = update(obj,x_TVQxyz,z_TVQxyz)
      % ''' compute Kalman gain '''
      PHT       = dot(obj.P, obj.H');
      obj.S     = dot(obj.H, PHT) + obj.R;
      obj.K     = dot(PHT,inv(obj.S));
      obj.K     = obj.K .* obj.K_scale;
      % ''' lin part '''
      hx          = dot(obj.H,x_TVQxyz);
      y_TVQ       = z_TVQxyz - hx; % TVWQxyz
      x_TVQ_post  = zeros(obj.dim_x,1);
      Ky          = dot(obj.K,y_TVQ);
      x_TVQ_post(1:6) = x_TVQxyz(1:6) + Ky(1:6);
      % ''' quat part '''
      x_q       = Qxyz2Q(x_TVQxyz(7:9,1));
      z_q       = Qxyz2Q(z_TVQxyz(7:9,1));
      y_PHIxyz  = z_q * quatinv(x_q); % get quaternion error
      y_PHIxyz  = quatnormalize(y_PHIxyz);
      y_PHIxyz  = Q2Qxyz(y_PHIxyz);
      y_PHIrpy  = quatlog(y_PHIxyz); % get rotation error
      ky_PHIrpy = dot(obj.K(7:9,7:9),y_PHIrpy);
      x_q_corr  = quatexp(obj.T_ .* ky_PHIrpy(1:3)); % quaternion correction
      x_q_corr  = Qxyz2Q(x_q_corr(1:3)); % return Quat object
      % equation 6 from EKF2 paper # update quaternion
      x_q_post = x_q_corr * x_q; % wxyz format
      x_q_post = quatnormalize(x_q_post);
      % at last update x 
      x_TVQxyz(1:6) = x_TVQ_post(1:6);
      x_Qwxyz = parts(x_q_post); 
      x_TVQxyz(7:9) = x_Qwxyz(2:4); % load quat xyz to x_post
      I_KH = obj.I_ - dot(obj.K,obj.H);
      obj.P = dot(dot(I_KH,obj.P),I_KH') + dot(dot(obj.K,obj.R),obj.K');
      
      % ''' log state vectors '''
      x_TVQxyzw = zeros(obj.dim_x+1,1);
      x_TVQxyzw(7:10,1) = obj.get_Qxyzw(x_TVQxyz(7:9,1));
      y_TVQ(7:9,1) = y_PHIxyz;
      obj.log.log_update(y_TVQ, x_TVQxyzw, obj.P, obj.K)
    end
    function x_TVQxyz = predict_x(obj, x_TVQxyz, u_Wrpy)
      % estimation model
      %  - eq 16-22 of QEKF2
      %  - this routine is essentially discrete form of \hat{x}_{k|k-1} =\
      %    f(\hat{x}_{k-1|k-1}, u_{k}) 
      % est linPos

      x_TVQxyz(1:3) = x_TVQxyz(1:3) + obj.T_ .* x_TVQxyz(4:6);
      % est linVel
      x_TVQxyz(4:6) = x_TVQxyz(4:6);
      % est rotVec (quat) -- eq(18)
      % est incremental rotation (in quat) based on input angVel (Wrpy) and delta t
      
      u_Rrpy = exp_map(obj.T_ .* u_Wrpy);



      q_u_Qwxyz = Qxyz2Q(u_Rrpy(1:3))
      q_u_Qwxyz = quaternion(u_Rrpy, 'rotvec')
      
      q_x_Qwxyz = Qxyz2Q(x_TVQxyz(7:9,1));
      q_x_Qwxyz = q_u_Qwxyz * q_x_Qwxyz;
      q_x_Qwxyz = quatnormalize(q_x_Qwxyz);
      x_TVQxyz(7:9,1) = Q2Qxyz(q_x_Qwxyz);
    end
    function [obj, x_TVQxyz] = predict(obj,x_TVQxyz,u_Wrpy)
      Qxyz      = x_TVQxyz(7:9,1);
      obj.C     = obj.get_C(Qxyz);
      obj.H     = obj.get_H(obj.C);
      obj.L     = obj.get_L(obj.C,obj.dim_x);
      obj.F     = obj.get_F(u_Wrpy,obj.dim_x,obj.T_);
      x_TVQxyz  = obj.predict_x(x_TVQxyz,u_Wrpy);
      obj.x_TVQxyz  = x_TVQxyz;
      Q_k       = dot(dot(dot(dot(obj.T_ .* obj.F,obj.L),obj.Q_c),obj.L'),obj.F');
      obj.P     = dot(dot(obj.F,obj.P),obj.F') + Q_k;
    end
  end 
  methods (Static)
    function F = get_F(u_Wrpy,dim_x,T)
      F           = eye(dim_x);
      F(1:3,4:6)  = T .* eye(3);
      u_skw       = skw_sym(u_Wrpy);
      F(7:9,7:9)  = eye(3) - T .* u_skw;
    end
    function H = get_H(C)
      H(1:9,1:9)  = eye(9);
      H(1:3,1:3)  = C;
    end 
    function L = get_L(C,dim_x)
      L           = eye(dim_x);
      L(4:6,4:6)  = -C';
    end 
    function C = get_C(Qxyz)
      q    = Qxyz2Q(Qxyz);
      C    = quat2rotm(q);
    end
    function res = get_losses(res, outDir)
      L1 = zeros(height(res),1);
      L2 = zeros(height(res),1);
      for i = 1:height(res)
        state_l1 = 0.0;
        state_l2 = 0.0;
        for j = 1:width(res)
          l1 = abs(res(i,j));
          l2 = res(i,j)^2;
          state_l1 = state_l1 + l1;
          state_l2 = state_l2 + l2;
        end
        L1(i,1) = state_l1;
        L2(i,2) = state_l2;
      end
      L1 = array2table(L1,'VariableNames','L1');
      L2 = array2table(L2,'VariableNames','L2');
      Losses = [L1,L2];
      res = [res,Losses];
      if isstring(outDir)
        fname = strcat(outDir,'losses.txt');
        file = fopen(fname, 'a');
        mssg = strcat('---->> L1:',num2str(sum(L1)),'  L2:',num2str(sum(L2)),'\n');
        disp(mssg);
        fprintf(file,mssg);
      end
    end
  end
end

%% local functions
%  
function [x,y,z] = Q2Qxyz(q)
  q = quantnormalize(q);
  [~,x,y,z] = parts(q);
end
function q = Qxyz2Q(Qxyz)
  disp(Qxyz)
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
  %   x = array(x);
    if norm_x == 0
      Qxyzw = [0,0,0,1];
    else
      qxyz = sin(norm_x/2) .* x/norm_x;
      qw = cos(norm_x/2);
      Qxyzw = [qxyz(1),qxyz(2),qxyz(3),qw];
    end
  end
end
