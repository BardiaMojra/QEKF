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
    p % posterior noise covariance
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
    function obj = config(obj, dset, ...
                          dim_x,dim_z,dim_u, ...
                          Q_T_xyz, ...
                          Q_V_xyz, ...
                          Q_quat_xyz, ...
                          R_noise, ...
                          P_est_0 ...
                          )
      obj.dset       = dset;
%         obj.log        = dlog;
      obj.dim_x      = dim_x;
      obj.dim_z      = dim_z;
      obj.dim_u      = dim_u;
      obj.Q_T_xyz    = Q_T_xyz;
      obj.Q_V_xyz    = Q_V_xyz;
      obj.Q_quat_xyz = Q_quat_xyz;
      obj.R_noise    = R_noise;
      obj.P_est_0    = P_est_0;
      % state vectors
      obj.x_TVQxyz   = obj.dset.dat(1,:); % set state initial conditions
      obj.P          = eye(dim_x) .* P_est_0;
      obj.F           = eye(dim_x);
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
    end

      def update(self, x_TVQxyz, z_TVQxyz):
    # compute Kalman gain
    PHT = dot(self.P, self.H.T)
    self.S = dot(self.H, PHT) + self.R
    self.K = PHT.dot(linalg.inv(self.S))
    self.K = self.K * self.K_scale
    ''' lin part '''
    hx = self.H @ x_TVQxyz
    y_TVQ = np.subtract(z_TVQxyz, hx) # TVWQxyz
    x_TVQ_post = np.zeros((self.dim_x,1))
    Ky = self.K @ y_TVQ
    x_TVQ_post[0:6] = x_TVQxyz[0:6] + Ky[0:6]
    ''' quat part '''
    x_q = get_npQ(x_TVQxyz[6:9,0])
    z_q = get_npQ(z_TVQxyz[6:9,0])
    y_PHIxyz = z_q * x_q.inverse() # get quaternion error
    y_PHIxyz = y_PHIxyz.normalized()
    y_PHIrpy = Q_log(get_Qwxyz(y_PHIxyz.imag)) # get rotation error
    ky_PHIrpy = self.K[6:9,6:9] @ y_PHIrpy
    x_q_corr = exp_map(self.T_*ky_PHIrpy[0:3,0]) # quaternion correction
    x_q_corr = get_npQ(x_q_corr[0:3])
    # equation 6 from EKF2 paper # update quaternion
    x_q_post = x_q_corr * x_q  ## wxyz format
    x_q_post = x_q_post.normalized()
    ''' at last update x '''
    x_TVQxyz[0:6] = x_TVQ_post[0:6]
    x_TVQxyz[6:9,0] = x_q_post.imag # load quat xyz to x_post
    I_KH = self._I -  (self.K @ self.H)
    self.P = (I_KH @ self.P) @ I_KH.T + (self.K @ self.R) @ self.K.T
    ''' log state vector '''
    x_TVQxyzw = np.ndarray((self.dim_x+1,1))
    x_TVQxyzw[:-4,0] = x_TVQxyz[:-3,0]
    x_TVQxyzw[6:10,0] = get_Qxyzw(x_TVQxyz[6:9,0])
    y_TVQ[6:9,0] = y_PHIxyz.imag
    self.log.log_update(y_TVQ, x_TVQxyzw, self.P, self.K)
    return x_TVQxyz

  def predict_x(self, x_TVQxyz, u_Wrpy):
    ''' estimation model
      - eq 16-22 of QEKF2
      - this routine is essentially discrete form of \hat{x}_{k|k-1} =\
        f(\hat{x}_{k-1|k-1}, u_{k}) '''
    # est linPos
    x_TVQxyz[0:3] = x_TVQxyz[0:3]+self.T_*x_TVQxyz[3:6]
    # est linVel
    x_TVQxyz[3:6] = x_TVQxyz[3:6]
    ''' est rotVec (quat) -- eq(18) '''
    # est incremental rotation (in quat) based on input angVel (Wrpy) and delta t
    u_Qxyzw = exp_map(self.T_ * u_Wrpy)
    q_u_Qwxyz = get_npQ(u_Qxyzw[0:3])
    q_x_Qwxyz = get_npQ(x_TVQxyz[6:9,0])
    q_x_Qwxyz = q_u_Qwxyz * q_x_Qwxyz
    q_x_Qwxyz = q_x_Qwxyz.normalized()
    x_TVQxyz[6:9,0] = q_x_Qwxyz.imag
    return x_TVQxyz

    

    function obj = set_F(obj,u_Wrpy)
      obj.F = eye(obj.dim_x);
      obj.F(1:3,4:6) = obj.T_ .* eye(3);
      obj.F(7:9,7:9) = eye(3) - obj.T_ .* obj.get_skew_symm_X(u_Wrpy);
    end
    function obj = set_H(obj)
      obj.H[1:9,1:9] = eye(9);
      obj.H(1:3,1:3) = obj.C;
    end 
    function obj = set_L(obj)
      obj.L = eye(obj.dim_x);
      obj.L(4:6,4:6) = -obj.C';
    end 
    function obj = set_C(obj,x_Qxyz)
      r = R.from_quat(x_Qxyz)
      obj.C = r.as_matrix();
    end
    def get_losses(res:pd.DataFrame, output_dir:str, save_en:bool=True, prt_en:bool=True):
  L1 = list()
  L2 = list()
  for i in range(len(res.index)):
    state_l1 = 0.0
    state_l2 = 0.0
    for j in range(len(res.columns)):
      l1 = abs(res.iloc[i,j])
      l2 = res.iloc[i,j] ** 2
      state_l1 += l1
      state_l2 += l2
    L1.append(state_l1);  L2.append(state_l2)
  L1_df = pd.DataFrame(L1, columns=['L1'])
  L2_df = pd.DataFrame(L2, columns=['L2'])
  res = pd.concat([res,L1_df, L2_df], axis=1)
  if save_en==True and  output_dir is not None:
    file_name = output_dir+'losses.txt'
    with open(file_name, 'a+') as f:
      L1_str = shorthead+f"L1 (total): {res['L1'].sum()}"
      L2_str = shorthead+f"L2 (total): {res['L2'].sum()}"
      f.write(L1_str)
      f.write(L2_str+'\n\n')
      f.close()
  return res

def print_losses(df: pd.DataFrame):
  print(shorthead+"L1 (total): ", df['L1'].sum())
  print(shorthead+"L2 (total): ", df['L2'].sum())
  print('\n\n')
  return
  end
end
