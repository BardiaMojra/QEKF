def get_skew_symm_X(x):
  X = np.zeros((3,3))
  X[0,1] = -x[2]
  X[0,2] =  x[1]
  X[1,0] =  x[2]
  X[1,2] = -x[0]
  X[2,0] = -x[1]
  X[2,1] =  x[0]
  return X

def get_omega(q):
  omega = np.array([  [-q[1], -q[2], -q[3] ],
                      [ q[0],  q[3], -q[2] ],
                      [-q[3],  q[0],  q[1] ],
                      [ q[2], -q[1],  q[0] ]
                      ])
  return np.reshape(omega,(4,3))

def check_symmetric(a, tol=1e-8):
  return np.all(np.abs(a-a.T) < tol)

### Exponential Map of Quaternion
def exp_map(x):
  if np.shape(x)[0] !=3:
    print("Vector size is not 3")
    return -1
  norm_x = norm(x)
  x = np.asarray(x)
  if norm_x ==0: return np.array([0,0,0,1])
  #nprint('norm_x', norm_x)
  temp_ = np.sin(norm_x/2)*x/norm_x
  temp_2 = np.cos(norm_x/2)
  return [ temp_[0],temp_[1],temp_[2], temp_2]

### Logarithmic map of Quaternion
def Q_log(q):
  q_v = q[0]
  q_n = np.array([q[1],q[2],q[3]])
  norm_q_n = np.linalg.norm(q_n)
  if q_v>1:
    q_v=1
  if q_v<-1:
    q_v=-1
  if (norm_q_n!=0 and q_v>=0):
    return 2*np.arccos(q_v)*q_n/norm_q_n
  elif (norm_q_n!=0 and q_v<0):
    return -2*np.arccos(-q_v)*q_n/norm_q_n
  elif norm_q_n==0:
    return zeros((3,1))

# # 3d plot
def set_axes_equal(ax):
  '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
  cubes as cubes, etc..  This is one possible solution to Matplotlib's
  ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

  Input
    ax: a matplotlib axis, e.g., as output from plt.gca().
  '''

  x_limits = ax.get_xlim3d()
  y_limits = ax.get_ylim3d()
  z_limits = ax.get_zlim3d()
  x_range = abs(x_limits[1] - x_limits[0])
  x_middle = np.mean(x_limits)
  y_range = abs(y_limits[1] - y_limits[0])
  y_middle = np.mean(y_limits)
  z_range = abs(z_limits[1] - z_limits[0])
  z_middle = np.mean(z_limits)

  # The plot bounding box is a sphere in the sense of the infinity
  # norm, hence I call half the max range the plot radius.
  plot_radius = 0.5*max([x_range, y_range, z_range])

  ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
  ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
  ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])
  return


class ExtendedKalmanFilter(object):
  ''' Simple Kalman Filter:
    - x is estimate of the state a process
      - x_k = A x_{k-1} + B u_{k-1} + w_{k-1}
    - z is measurement matrix
      - z_k = H x_k + v_k
    - w_k is a random variable that represents process noise
      - p(w) ~ Normal_Dist(0, Q)
    - v_k is a random variable that represents measurement noise
      - p(v) ~ Normal_Dist(0, R)
    - Q is the process noise covariance
    - R is the measurement noise covariance

  Extended Kalman Filter:

      \-->> New QEKF (var/dim):  ----------   z
      lin_pos: T_xyz (3)
      lin_vel: V_xyz (3)
      ang_vel: W_rpy (3)
      ang_vel_b:
      ang_rot: Q_wxyz (4)
            --->> 13 state variables

    \-->> New QEKF (var/dim):  ----------   x
      lin_acc:
      lin_acc_bias:
      lin_vel: V_xyz (3)
      lin_pos: T_xyz (3)
      ang_acc:
      ang_vel: W_rpy
      ang_vel_b:
      ang_rot: Q_xyz (3)
            --->> 9 (+1 for q_w) state variables

    \-->> New QEKF (var/dim):  ----------   x_prior
      lin_acc:
      lin_acc_bias:
      lin_vel: V_xyz (3)
      lin_pos: T_xyz (3)
      ang_acc:
      ang_vel: W_rpy
      ang_vel_b:
      ang_rot: Q_wxyz (3+1)
            --->> 9 (+1 for q_w) state variables


    #TODO:
      - scale factor
        lin_vel_Vest * data_time_period = delta_lin_pos_Quest * (some_scalar)
        equation must hold true per V * deltaT = deltaX and scalar factor
        should correspond to scale factor difference.
  '''
  def __init__(self,
               dim_x,
               dim_z,
               deltaT,
               Q_T_xyz,
               Q_V_xyz,
               Q_quat_xyz,
               R_noise,
               P_est_0,
               dim_u=0,
               log=True,
               adaWind=25,
               K_scale=1.0,
               x_check=None,
               P_check=None):

    self.K_scale = K_scale
    self.dim_x = dim_x
    self.dim_z = dim_z
    self.dim_u = dim_u
    self.x_post_TVQxyz = np.zeros((dim_x,1)) + .0001
    self.P_post = np.eye(dim_x) * P_est_0  # uncertainty covariance
    self.B = 0                 # control transition matrix
    self.F = np.eye(dim_x)     # state transition matrix
    self.R = np.eye(dim_z)        # state uncertainty
    self.Q = np.eye(dim_x)        # process uncertainty
    self.y_TVWQxyz = np.zeros((dim_z, 1)) # residual
    #self.G = None
    self.T_ = deltaT #time-period
    #self.w_= np.zeros((dim_x,1))
    # z = np.array([None]*self.dim_z)
    self.z_TVWQxyzw = np.zeros((dim_z,1))
    self.v = np.zeros((dim_z, adaWind))
    # gain and residual are computed during the innovation step. We
    # save them so that in case you want to inspect them for various
    # purposes
    self.K = np.zeros((dim_x,1)) # kalman gain
    self.S = np.zeros((dim_z, dim_z))   # system uncertainty
    self.SI = np.zeros((dim_z, dim_z))  # inverse system uncertainty
    self.L = None
    # identity matrix. Do not alter this.
    self._I = np.eye(dim_x)

    # these will always be a copy of x,P after predict() is called
    #self.x_prior_TVQwxyz = self.x.copy()
    #self.P_prior = self.P_post.copy()

    # these will always be a copy of x,P after update() is called
    #self.x_post = self.x.copy()
    #self.P_post = self.P_post.copy()
    self.C = None
    #self.x_post_Qwxyz= Quaternion([1,0,0,0])


    self.x_prior_TVQwxyz = zeros((dim_x+1,1)) # add 1 extra for Quat w term
    self.H = np.zeros((dim_z, dim_x))

    # quaternion measurement is 4D - so is for updating estimation model
    #self.z = np.zeros((dim_z+1,1)) # trans_xyz, vel_xyz, rot_wxyz, vel_rpy
    self.z_TVWQxyzw = np.zeros((dim_z+1,1))


    ''' init the process iid noise covar matrix Q -- 9x9 diagonal (dim_x by dim_x)
    '''
    self.Q = np.diag(np.array([Q_T_xyz,
                               Q_T_xyz,
                               Q_T_xyz,
                               Q_V_xyz,
                               Q_V_xyz,
                               Q_V_xyz,
                               Q_quat_xyz,
                               Q_quat_xyz,
                               Q_quat_xyz]))

    ''' init the measurement iid noise covar matrix R -- 12x12 diagonal (dim_z by dim_z)
    '''
    self.R = np.diag(np.array([R_noise, R_noise, R_noise, R_noise, R_noise, R_noise,
                            R_noise, R_noise, R_noise, R_noise, R_noise, R_noise]))

    # data logger
    self.log = dlm(enabled=log)
    self.plotter = dmm


    ## end of init


  def update(self, z_TVWQxyzw):
    if z_TVWQxyzw is None:
      st()
      self.z_TVWQxyzw = np.array([[None]*self.dim_z]).T
      eprint(longhead+'Err: in update(), z is None...'+longtail)
      return

    if np.isscalar(z_TVWQxyzw) and self.dim_z == 1:
      st()
      z_TVWQxyzw = np.asarray([z_TVWQxyzw], float)
      eprint(longhead+'Err: in update(), z is a scalar and z vect is created...'+longtail)

    # compute Kalman gain
    PHT = dot(self.P_prior, self.H.T)
    self.S = dot(self.H, PHT) + self.R
    self.K = PHT.dot(linalg.inv(self.S))
    #nprint('self.K', self.K)

    # compute Kalman gain check
    # H_check must be the Jacobian of observation function
    #K_check =  self.P_check @ H_check.T @ np.linalg.inv(H_check @ self.P_check @ H_check.T + Q_check)
    #print('K_check', K_check)

    # modified for qekf and quarternion states
    x_temp = zeros((self.dim_x,1))
    x_temp[0:6,0] = self.x_prior_TVQwxyz[0:6,0]
    x_temp[6:9,0] = self.x_prior_TVQwxyz[7:10,0]
    nprint('update: self.x_prior_TVQwxyz', self.x_prior_TVQwxyz)
    nprint('update: x_temp_TVQxyz', x_temp)
    # calc est
    hx = np.dot(self.H, x_temp)
    nprint ('update: self.z_TVWQxyzw[0:12,0]', self.z_TVWQxyzw[0:12,0])
    nprint ('update: hx', hx)
    ''' lin part '''
    self.y_TVWQxyz = np.subtract(self.z_TVWQxyzw[0:12,0], hx.T).T # TVWQxyz
    nprint ('update: error : self.y_TVWQxyz', self.y_TVWQxyz)

    #st()

    # lin part check
    #x_est_check = zeros((self.dim_x,1))
    #x_est_check[0:6,0] = self.x_prior_TVQwxyz[0:6,0]
    #x_est_check[6:9,0] = self.x_prior_TVQwxyz[7:10,0]
    #z_check =

    '''NBUG'''
    nprint ('update: self.x_prior_TVQwxyz[6:10,0]', self.x_prior_TVQwxyz[6:10,0])
    nprint ('update: self.z_TVWQxyzw[12,9,10,11]', self.z_TVWQxyzw[12],self.z_TVWQxyzw[9],self.z_TVWQxyzw[10],self.z_TVWQxyzw[11])


    ''' quat part '''
    q_estimate = Quaternion(self.x_prior_TVQwxyz[6:10,0]) ## w,x,y,z -- quat part
    q_measurement = Quaternion(self.z_TVWQxyzw[12],self.z_TVWQxyzw[9],self.z_TVWQxyzw[10],self.z_TVWQxyzw[11]) # w,x,y,z
    e__ = ((q_measurement*q_estimate.inverse)) ## check
    e__log = Q_log(e__.elements)#


    self.y_TVWQxyz[9:12,0] = [e__log[0],e__log[1],e__log[2]]

    #st()
    self.K = self.K * self.K_scale

    ky = dot(self.K, self.y_TVWQxyz)
    self.x_post_TVQxyz = x_temp + ky # dot(self.K, self.y)

    temp_exp_map = exp_map(self.T_*ky[6:9])

    # equation 6 from EKF2 paper
    exp_map_ = Quaternion([temp_exp_map[3],temp_exp_map[0],temp_exp_map[1],temp_exp_map[2]]) \
      * Quaternion(self.x_prior_TVQwxyz[6:10,0])  ## wxyz #TODO: should be quaternion object?

    #st()
    self.x_post_TVQxyz[6:9,0] = exp_map_.elements[1:4]
    #self.x_post_Qwxyz = exp_map_

    # P = (I-KH)P(I-KH)' + KRK' is more numerically stable
    # and works for non-optimal K vs the equation
    # P = (I-KH)P usually seen in the literature.
    I_KH = self._I - dot(self.K, self.H)
    self.P_post = dot(I_KH, self.P_prior).dot(I_KH.T) + dot(self.K, self.R).dot(self.K.T)

    # set to None to force recompute
    #self._log_likelihood = None
    #self._likelihood = None
    #self._mahalanobis = None
    #self.x = self.x_post.copy()
    #self.P = self.P_post.copy()
    self.log.log_update(self.y_TVWQxyz, self.x_post_TVQxyz, self.P_post, self.K)
    return

  def predict_x(self, u=0):
    # eq 16-22 of QEKF2
    self.x_prior_TVQwxyz[0:3,0] = self.x_post_TVQxyz[0:3,0]+\
                                  self.T_*self.x_post_TVQxyz[3:6,0]
    self.x_prior_TVQwxyz[3:6,0] = self.x_post_TVQxyz[3:6,0] #+ self.T_ #*temp_

    #temp = exp_map(self.T_/2*(self.z[6:9,0]-self.z[13:16,0]))
    #st()
    z_Wrpy = self.z_TVWQxyzw[6:9,0]
    nprint('predict_x: z_Wrpy', z_Wrpy)
    z_Wrpy = np.expand_dims(z_Wrpy, axis=1)
    #tmp = self.T_* self.C @ z_Wrpy
    #nprint('tmp', tmp)
    temp = exp_map(self.T_* self.C.T @ z_Wrpy)
    #temp = exp_map(z_Wrpy)

    #st()
    nprint('predict_x: temp _Qxyzw', temp)

    temp_q = Quaternion(temp[3],temp[0],temp[1],temp[2]) ##w,x,y,z
    nprint('predict_x: temp_q _wxyz_', temp_q) #

    nprint('predict_x: self.x_post', self.x_post_TVQxyz) #


    # equation 18
    self.x_prior_TVQwxyz[6:10,0] = (temp_q *\
      Quaternion(imaginary=self.x_post_TVQxyz[6:9])).elements ##wxyz
    #self.x_prior[10:13,0]= self.x[9:12,0]
    #self.x_prior[13:16,0]= self.x[12:15,0]
    #self.log.log_predicted_quat(self.x_q, self.x_prior[6:10,0])
    #nprint("self.x_post_Qwxyz", self.x_post_Qwxyz)
    nprint('self.x_prior_TVQwxyz[6:10,0]', self.x_prior_TVQwxyz[6:10,0])

    #st()
    return


  def predict(self, u=0):
    self.set_H()
    self.set_L()
    self.set_F() #
    self.predict_x()
    # Note: L could be taken out
    Q_ = self.T_ * self.F @ self.L @ self.Q @ self.L.T @ self.F.T
    self.P_prior = dot(self.F, self.P_post).dot(self.F.T) + Q_#self.Q
    self.log.log_prediction(self.x_prior_TVQwxyz, self.P_prior)
    return

  def partial_update(self,gamma,beta):
    for i in range(self.dim_x):
      self.x[i] = gamma[i]*self.x_post_TVQxyz[i] + (1-gamma[i])*self.x_prior_TVQwxyz[i]
      for j in range(self.dim_x):
        self.P_post[i,j] = gamma[i]*gamma[j]*self.P_post[i,j]+(1-gamma[i]*gamma[j])*self.P_prior[i,j]

  def set_F(self):
    self.F = np.eye(self.dim_x)
    self.F[0:3,3:6] = self.T_*np.eye(3)
    '''note: we no longer have f_k^x (skew symmetry) since we dont have lin force'''
    self.F[3:6,6:9] = np.zeros(3)# -self.T_* self.C.T @ get_skew_symm_X(self.z[0:3,0])
    #self.F[3:6,9:12] = -self.T_* self.C.T
    self.F[6:9,6:9] = np.eye(3) - self.T_*get_skew_symm_X(self.z_TVWQxyzw[6:9,0])
    #self.F[6:9,12:15] = - self.T_*np.eye(3)

  def set_G(self,W,Omega,V):
    self.G = np.zeros((self.dim_x,9))
    self.G[0:3,0:3] = ((self.T_**3)/6)*np.eye(3)
    self.G[3:6,0:3] = ((self.T_**2)/2)*np.eye(3)
    self.G[6:9,0:3] = self.T_*np.eye(3)
    self.G[6:9,3:6] = self.T_*V
    self.G[9:12,6:9] = self.T_*np.eye(3)
    self.G[12:16,3:6] = (self.T_/2)*(np.sin(norm(W)*self.T_/2)/norm(W)) * Omega
    self.G[16:19,3:6] = self.T_*np.eye(3)

  def get_z_TVWQxyzw(self, lin_vel, translation, ang_vel, quat, Vscale=1):
    self.z_TVWQxyzw[0:3,0] = translation
    self.z_TVWQxyzw[3:6,0] = lin_vel*Vscale
    self.z_TVWQxyzw[6:9,0] = ang_vel
    self.z_TVWQxyzw[9:13,0] = quat
    return

  def set_H(self):
    # set measurement transition function (H matrix)
    nprint('self.x_post[3:6,0]', self.x_post_TVQxyz[3:6,0])
    #st()
    self.H[0:3,0:3] = -self.C.T
    self.H[0:3,3:6] = self.C.T ## go away?
    self.H[0:3,6:9] = -self.C.T @ get_skew_symm_X(self.x_post_TVQxyz[3:6,0])
    # drk.H[3:6,0:3] = np.zeros(3)
    self.H[3:6,3:6] = -self.C.T
    # drk.H[3:6,6:9] = np.zeros(3)
    # drk.H[6:9,0:3] = np.zeros(3)
    self.H[6:9,6:9] = np.eye(3)
    return self

  def set_L(self):
    ## QEKF2 L matrix
    self.L = np.eye(self.dim_x)
    self.L[3:6,3:6] = -self.C.T # todo:
    self.L[0:3,0:3] = 0
    self.L[6:9,6:9] = -np.eye(3)
    return self

# EOF
