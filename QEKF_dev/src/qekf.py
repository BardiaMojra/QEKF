
from pyquaternion import Quaternion
import numpy as np
import scipy.linalg as linalg
from pdb import set_trace as st
from pprint import pprint as pp
from numpy import dot, zeros, eye
from scipy.linalg import norm
from dlm import dlm
import dmm

''' general config '''
longhead = '\n\n  \--->> '
shorthead = '\n  \--->> '
longtail = '\n\n'
attn = 'here ----------- <<<<<\n\n'  #print(longhead+attn)

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
  '''
  def __init__(self, dim_x, dim_z, deltaT, Q_T_xyz, Q_V_xyz, Q_quat_xyz, \
    R_noise, P_est_0, dim_u=0, log=True, adaWind=25, plotter=dmm):
    self.dim_x = dim_x
    self.dim_z = dim_z
    self.dim_u = dim_u
    self.x = np.zeros((dim_x,1)) + .0001
    self.P = eye(dim_x) * P_est_0  # uncertainty covariance
    self.B = 0                 # control transition matrix
    self.F = np.eye(dim_x)     # state transition matrix
    self.R = eye(dim_z)        # state uncertainty
    self.Q = eye(dim_x)        # process uncertainty
    self.y = zeros((dim_z, 1)) # residual
    self.G = None
    self.T_ = deltaT #time-period
    self.w_=zeros((9,1))
    # z = np.array([None]*self.dim_z)
    self.z_TVWrpyQxyzw = zeros((dim_z,1))
    self.v = zeros((dim_z, adaWind))
    # gain and residual are computed during the innovation step. We
    # save them so that in case you want to inspect them for various
    # purposes
    self.K = np.zeros(self.x.shape) # kalman gain
    self.S = np.zeros((dim_z, dim_z))   # system uncertainty
    self.SI = np.zeros((dim_z, dim_z))  # inverse system uncertainty
    self.L = None
    # identity matrix. Do not alter this.
    self._I = np.eye(dim_x)

    # self._log_likelihood = log(sys.float_info.min)
    # self._likelihood = sys.float_info.min
    # self._mahalanobis = None

    # these will always be a copy of x,P after predict() is called
    self.x_prior_TVQwxyz = self.x.copy()
    self.P_prior = self.P.copy()

    # these will always be a copy of x,P after update() is called
    self.x_post = self.x.copy()
    self.P_post = self.P.copy()
    self.C = None
    self.x_qwxyz= Quaternion([1,0,0,0])


    self.x_prior_TVQwxyz = zeros((dim_x+1,1)) # add 1 extra for Quat w term
    self.H = np.zeros((dim_z, dim_x))

    # quaternion measurement is 4D - so is for updating estimation model
    #self.z = np.zeros((dim_z+1,1)) # trans_xyz, vel_xyz, rot_wxyz, vel_rpy
    self.z_TVWrpyQxyzw = zeros((dim_z+1,1))


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


  def update(self, z, R=None, args=(), hx_args=(), residual=np.subtract):
    if z is None:
      self.z_TVWrpyQxyzw = np.array([[None]*self.dim_z]).T
      self.x_post = self.x.copy()
      self.P_post = self.P.copy()
      return

    if not isinstance(args, tuple): args = (args,)
    if not isinstance(hx_args, tuple): hx_args = (hx_args,)

    if R is None or R.shape != (self.dim_z,self.dim_z):
      R = self.R
    elif np.isscalar(R):
      R = eye(self.dim_z) * R

    if np.isscalar(z) and self.dim_z == 1:
      z = np.asarray([z], float)

    #H = HJacobian(self.x, *args)
    #H = self.Hself.z[12]
    PHT = dot(self.P_prior, self.H.T)
    self.S = dot(self.H, PHT) + R
    self.K = PHT.dot(linalg.inv(self.S))
    #hx = Hx(self.x, *hx_args)
    #hx = np.dot(self.H,self.x_prior)
    #self.y = residual(z, hx.T).T
    # modified for qekf and quarternion states
    x_temp = zeros((self.dim_x,1))
    x_temp[0:6,0] = self.x_prior_TVQwxyz[0:6,0] #TODO: BUG
    x_temp[6:9,0] = self.x_prior_TVQwxyz[7:10,0] #TODO: BUG
    print(shorthead+'x_prior_TVQwxyz[0:10,0]:_TVQWxyz _____BUG_____')
    print(x_temp)
    print(shorthead+'x_temp[0:9,0]:_TVQxyz _____BUG_____')
    print(x_temp)
    hx = np.dot(self.H,x_temp)

    # lin part
    self.y = residual(self.z_TVWrpyQxyzw[0:12,0], hx.T).T


    ''' quat part '''
    q_estimate = Quaternion(self.x_prior_TVQwxyz[6:10,0]) ## w,x,y,z -- quat part
    q_measurement = Quaternion(self.z_TVWrpyQxyzw[12],self.z_TVWrpyQxyzw[9],self.z_TVWrpyQxyzw[10],self.z_TVWrpyQxyzw[11]) # w,x,y,z

    print_q_est_and_meas = True
    if print_q_est_and_meas is True:
      print(shorthead+'q_estimate: _wxyz_ ')
      print(q_estimate)
      print(shorthead+'q_measurement: _wxyz_')
      print(q_measurement)

    e__ = ((q_measurement*q_estimate.inverse)) ## check
    e__log = Q_log(e__.elements)#

    print_e_and_elog = True
    if print_e_and_elog is True:
      print(shorthead+'e__:_wxyz_')
      pp(e__)
      print(shorthead+'e__log:_xyz_')
      pp(e__log)

    self.y[9:12,0] = [e__log[0],e__log[1],e__log[2]]
    ky = dot(self.K, self.y)

    print_k_and_y = True
    if print_k_and_y is True:
      #print(shorthead+'self.K: ')
      #print(self.K)
      print(shorthead+'self.y:')
      print(self.y)
      print(shorthead+'ky:')
      print(ky)


    self.x_post = x_temp + ky # dot(self.K, self.y)


    #print(longhead+'e__.elements:')
    #print('     \--->> shape:'+str(e__.elements.shape))
    #print(e__.elements)
    #print(longhead+'ky[6:9]:')
    #print(ky[6:9])

    temp_exp_map = exp_map(ky[6:9])
    #print('     \--->> len:'+str(len(temp_exp_map)))
    #print(longhead+'temp_exp_map:')
    #print(temp_exp_map)

    print_temp_exp_map = True
    if print_temp_exp_map is True:
      print(shorthead+'temp_exp_map: _xyzw_ ')
      print(temp_exp_map)

    #st()
    # equation 6 from EKF2 paper #TODO: change back to 3012
    exp_map_ = Quaternion([temp_exp_map[3],temp_exp_map[0],temp_exp_map[1],temp_exp_map[2]]) * self.x_prior_TVQwxyz[6:10,0]  ## wxyz

    print(shorthead+"x_prior_TVQwxyz[6:10,0]:"+str(self.x_prior_TVQwxyz.shape)+'wxyz') # the prediction
    print(self.x_prior_TVQwxyz[6:10,0])
    print(shorthead+"exp_map_: _wxyz_")
    print(exp_map_)

    self.x_post[6:9,0] = exp_map_.elements[1:4]
    self.x_qwxyz = exp_map_

    # P = (I-KH)P(I-KH)' + KRK' is more numerically stable
    # and works for non-optimal K vs the equation
    # P = (I-KH)P usually seen in the literature.
    I_KH = self._I - dot(self.K, self.H)
    self.P_post = dot(I_KH, self.P_prior).dot(I_KH.T) + dot(self.K, R).dot(self.K.T)

    # set to None to force recompute
    self._log_likelihood = None
    self._likelihood = None
    self._mahalanobis = None
    self.x = self.x_post.copy()
    self.P = self.P_post.copy()
    self.log.log_update(self.y, self.x_post, self.P_post, self.K)
    return

  def predict_x(self, u=0): # eq 16-22 of QEKF2
    ## gravity added -- check again
    #temp_ = self.C.T@ (self.z[0:3,0]-self.x[9:12,0])+self.z[3:6,0]

    self.x_prior_TVQwxyz[0:3,0] = self.x[0:3,0] + self.T_*self.x[3:6,0]
    self.x_prior_TVQwxyz[3:6,0] = self.x[3:6,0] #+ self.T_ #*temp_
    #temp = exp_map(self.T_/2*(self.z[6:9,0]-self.z[13:16,0]))
    #st()
    temp = exp_map(self.T_/2*(self.z_TVWrpyQxyzw[9:12,0]))
    temp_q = Quaternion(temp[3],temp[1],temp[1],temp[2]) ##w,x,y,z

    print(shorthead+"z_TVWrpyQxyzw[9:12,0]:_xyzw_")
    print(self.z_TVWrpyQxyzw[9:12,0])
    print(shorthead+"temp:_xyzw_")
    print(temp)
    print(shorthead+"temp_q:_wxyz_") #
    print(temp_q)

    # equation 18
    self.x_prior_TVQwxyz[6:10,0] = (temp_q * self.x_qwxyz).elements ##wxyz
    #self.x_prior[10:13,0]= self.x[9:12,0]
    #self.x_prior[13:16,0]= self.x[12:15,0]
    #self.log.log_predicted_quat(self.x_q, self.x_prior[6:10,0])
    print(shorthead+"x_q_wxyz_:")
    print(self.x_qwxyz)
    print(shorthead+"x_prior_q[6:10,0]:_wxyz_") # the prediction
    print(self.x_prior_TVQwxyz[6:10,0])


  def predict(self, u=0):
    self.set_F() #
    self.predict_x()
    # Note: L could be taken out
    Q_ = self.T_ * self.F @ self.L @ self.Q @ self.L.T @ self.F.T
    self.P_prior = dot(self.F, self.P).dot(self.F.T) + Q_#self.Q
    self.log.log_prediction(self.x_prior_TVQwxyz, self.P_prior)
    return

  def partial_update(self,gamma,beta):
    for i in range(self.dim_x):
      self.x[i] = gamma[i]*self.x_post[i] + (1-gamma[i])*self.x_prior_TVQwxyz[i]
      for j in range(self.dim_x):
        self.P[i,j] = gamma[i]*gamma[j]*self.P_post[i,j]+(1-gamma[i]*gamma[j])*self.P_prior[i,j]

  def set_F(self):
    self.F = np.eye(self.dim_x)
    self.F[0:3,3:6] = self.T_*np.eye(3)
    '''note: we no longer have f_k^x (skew symmetry) since we dont have lin force'''
    self.F[3:6,6:9] = np.zeros(3)# -self.T_* self.C.T @ get_skew_symm_X(self.z[0:3,0])
    #self.F[3:6,9:12] = -self.T_* self.C.T
    self.F[6:9,6:9] = np.eye(3) - self.T_*get_skew_symm_X(self.z_TVWrpyQxyzw[6:9,0])
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

  def get_z_TVWrpyQxyzw(self, lin_vel, translation, ang_vel, quat, Vscale=1):
    self.z_TVWrpyQxyzw[0:3,0] = translation
    self.z_TVWrpyQxyzw[3:6,0] = lin_vel*Vscale
    self.z_TVWrpyQxyzw[6:9,0] = ang_vel
    self.z_TVWrpyQxyzw[9:13,0] = quat
    return

def get_skew_symm_X(x):
  X = np.zeros((3,3))
  X[0,1] = -x[2]
  X[0,2] = x[1]
  X[1,0] = x[2]
  X[1,2] = -x[0]
  X[2,0] = -x[1]
  X[2,1] = x[0]
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

# def transform_conj(q):
#   Q = np.array([   [2*q[0]**2-1+2*q[1]**2,    2*q[1]*q[2]-2*q[0]*q[3],  2*q[1]*q[3]+2*q[0]*q[2] ],
#           [2*q[1]*q[2]+2*q[0]*q[3],  2*q[0]**2-1+2*q[2]**2,    2*q[2]*q[3]-2*q[0]*q[1] ],
#           [2*q[1]*q[3]-2*q[0]*q[2],  2*q[2]*q[3]+2*q[0]*q[1],  2*q[0]**2-1+2*q[3]**2] ])
#   return np.reshape(Q,(3,3))

## fixed the error in implementation but results donot match matlab quat2rotm
def transform_(q):
  Q=np.array(
    [ [2*q[0]**2-1+2*q[3]**2,   2*q[0]*q[1]+2*q[2]*q[3], 2*q[0]*q[2]-2*q[1]*q[3]],
      [2*q[0]*q[1]-2*q[2]*q[3], 2*q[1]**2-1+2*q[3]**2,   2*q[1]*q[2]+2*q[0]*q[3]],
      [2*q[0]*q[2]+2*q[1]*q[3], 2*q[1]*q[2]-2*q[0]*q[3], 2*q[2]**2-1+2*q[3]**2]   ]
      )
  return np.reshape(Q,(3,3))

### Exponential Map of Quaternion
def exp_map(x):
  if np.shape(x)[0] !=3:
    print("Vector size is not 3")
    return -1
  norm_x = norm(x)
  x = np.asarray(x)
  if norm_x ==0: return np.array([0,0,0,1])
  temp_ = np.sin(norm_x/2)*x/norm_x
  temp_2 = np.cos(norm_x/2)
  return [temp_[0],temp_[1],temp_[2],temp_2] # xyzw

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
