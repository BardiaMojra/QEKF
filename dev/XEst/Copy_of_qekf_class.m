classdef qekf_class___sssss
  %qekf_class qekf class holds related configs and data structures 
  %   detailed explanation goes here
  properties
    %% config (pass in to init internal configs)
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
    K_scale = 1.0; % linear gain factor for manual tuning 
    
    %% internal data structs 
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
    %%  log
    log_en = true; % data logger flag
    log % datalog obj
    idx 
    z_hist % state measurement history
    x_hist % state estimate history
    y_hist % state residual history
    P_hist % state estimation covar history
    K_hist %Kalman gain matrix history 
    ylabels = {'Tx','Ty','Tz','vx','vy','vz','qx','qy','qz','qw'};
    name % dataset
    outDir % output dir 
  end
  methods
    function obj = load_cfg(obj, cfg)                    
      %obj.dset       = dset;
      %obj.log        = dlog;
     
      obj.name       = dname;
      obj.outDir     = outdir;
     
      
    end
    %dset, ... % dlog, ...
    function [obj, x_TVQxyz] = update(obj,x_TVQxyz,z_TVQxyzw,i)
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
      % update x 
      x_TVQxyz(1:6) = x_TVQ_post(1:6);
      x_Qwxyz = x_q_post.compact(); 
      x_TVQxyz(7:9) = x_Qwxyz(2:4); % load quat xyz to x_post
      I_KH = obj.I_ - obj.K * obj.H;
      obj.P = I_KH * obj.P * I_KH' + obj.K * obj.R * obj.K';
      % ''' log state vectors '''
      x_TVQxyzw = zeros(obj.dim_x+1,1);
      x_TVQxyzw(7:10,1) = Qxyz2Qxyzw(x_TVQxyz(7:9,1));
      [w,x,y,z] = y_PHI_Qwxyz.parts();
      y_TVQw = zeros(obj.dim_x+1,1);
      y_TVQw(1:6,1) = y_TVQ(1:6,1);  
      y_TVQw(7:10,1) = [x,y,z,w];
      obj = obj.log_update(y_TVQw, x_TVQxyzw,obj.P,obj.K);
      obj = obj.log_meas(z_TVQxyzw,i);
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
    function [obj, x_TVQxyz] = predict(obj,x_TVQxyz,u_Wrpy)
      Qxyz      = x_TVQxyz(7:9,1);
      obj.C     = obj.get_C(Qxyz);
      obj.H     = obj.get_H(obj.C);
      obj.L     = obj.get_L(obj.C,obj.dim_x);
      obj.F     = obj.get_F(u_Wrpy,obj.dim_x,obj.T_);
      x_TVQxyz  = obj.predict_x(x_TVQxyz,u_Wrpy);
      obj.x_TVQxyz  = x_TVQxyz;
      Q_k       = obj.T_ .* obj.F * obj.L * obj.Q_c * obj.L' * obj.F';
      obj.P     = obj.F * obj.P * obj.F' + Q_k;
    end
    function obj = log_meas(obj,z,idx)
      z = reshape(z,[],1);
      obj.z_hist = cat(1,obj.z_hist,z');
      obj.idx = cat(1,obj.idx,idx);
    end
    function obj = log_update(obj,y,x,P,K)
      y = reshape(y,[],1);
      x = reshape(x,[],1);
      P = reshape(P,[],1);
      K = reshape(K,[],1);
      obj.y_hist = cat(1,obj.y_hist,y');
      obj.x_hist = cat(1,obj.x_hist,x');
      obj.P_hist = cat(1,obj.P_hist,P');
      obj.K_hist = cat(1,obj.K_hist,K');   
    end
    function res = get_res(obj)
      losses  = obj.get_losses(obj.y_hist);
      res     = array2table(obj.y_hist,'VariableNames',obj.ylabels);
      res     = [res,losses];
      fname = strcat(obj.outDir,'res_',obj.name,'_tab.csv');
      writetable(res,fname);
%       head(res);
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
    function losses = get_losses(res)
      losses = zeros(height(res),2);
      for i = 1:height(res)
        state_l1 = 0.0;
        state_l2 = 0.0;
        for j = 1:width(res)-1 % ignore q.w term
          l1 = abs(res(i,j));
          l2 = res(i,j)^2;
          state_l1 = state_l1 + l1;
          state_l2 = state_l2 + l2;
        end
        losses(i,1) = state_l1;
        losses(i,2) = state_l2;
      end
      losses = array2table(losses,'VariableNames',{'L1','L2'});
%       head(losses);
      %       res = [res,Losses];
%       if isstring(outDir)
%         fname = strcat(outDir,'losses.txt');
%         file = fopen(fname, 'a');
%         mssg = strcat('---->> L1:',num2str(sum(L1)),'  L2:',num2str(sum(L2)),'\n');
%         disp(mssg);
%         fprintf(file,mssg);
%       end
    end
  end
  methods (Access = private)
    function init(obj)
      % state vectors
      obj.x_TVQxyz  = horzcat(obj.dset.Txyz(1,:), ... 
                              obj.dset.Vxyz(1,:), ...
                              obj.dset.Qxyzw(1,1:end-1))'; % set state initial conditions
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
    end 
  end % methods (Access = private)
end

%% local functions
%  
function head(dat)
  disp(dat(1:5,:));
end
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
