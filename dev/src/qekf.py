
from pyquaternion import Quaternion
import numpy as np
import scipy.linalg as linalg
from pdb import set_trace as st
from pprint import pprint as pp
from numpy import dot, zeros, eye
from scipy.linalg import norm
from dmm import *
from dlm import *
from util import *

''' general config '''
longhead  = '\n\--->> '
shorthead = '\--->> '
longtail  = '\n\n'
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
    self.Q_c = np.eye(dim_x)        # process uncertainty
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
    self.K = np.zeros((dim_x,1)) # kalman gain -- 9
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
    self.H = np.zeros((dim_z, dim_x)) #12x9

    # quaternion measurement is 4D - so is for updating estimation model
    #self.z = np.zeros((dim_z+1,1)) # trans_xyz, vel_xyz, rot_wxyz, vel_rpy
    self.z_TVWQxyzw = np.zeros((dim_z+1,1))


    ''' init the process iid noise covar matrix Q -- 9x9 diagonal (dim_x by dim_x)
    '''
    self.Q_c = np.diag(np.array([Q_T_xyz,
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
    self.R = np.diag(np.array([R_noise,
                               R_noise,
                               R_noise,
                               R_noise,
                               R_noise,
                               R_noise,
                               R_noise,
                               R_noise,
                               R_noise,
                               R_noise,
                               R_noise,
                               R_noise]))

    # data logger
    self.log = dlm(enabled=log)
    self.plotter = dmm
    ## end of init

  def update(self, z_TVWQxyzw):
    if z_TVWQxyzw is None:
      self.z_TVWQxyzw = np.array([[None]*self.dim_z]).T
      eprint(longhead+'Err: in update(), z is None and z vect is created...'+longtail)
    if np.isscalar(z_TVWQxyzw) and self.dim_z == 1:
      z_TVWQxyzw = np.asarray([z_TVWQxyzw], float)
      eprint(longhead+'Err: in update(), z is a scalar and z vect is created...'+longtail)
    # compute Kalman gain
    PHT = dot(self.P_prior, self.H.T)
    self.S = dot(self.H, PHT) + self.R
    self.K = PHT.dot(linalg.inv(self.S))
    # modified for qekf and quarternion states
    x_prior_TVQxyz_tmp = zeros((self.dim_x,1))
    x_prior_TVQxyz_tmp[0:6,0] = self.x_prior_TVQwxyz[0:6,0]
    x_prior_TVQxyz_tmp[6:9,0] = self.x_prior_TVQwxyz[7:10,0]
    hx = np.dot(self.H, x_prior_TVQxyz_tmp)
    ''' lin part
    '''
    self.y_TVWQxyz = np.subtract(self.z_TVWQxyzw[0:12,0], hx.T).T # TVWQxyz
    ''' quat part
    '''
    x_prior_est_Qwxyz_q = Quaternion(self.x_prior_TVQwxyz[6:10,0]) # wxyz input
    x_obs_est_Qwxyz_q = Quaternion(self.z_TVWQxyzw[12], self.z_TVWQxyzw[9],
                                   self.z_TVWQxyzw[10], self.z_TVWQxyzw[11])
    e__ = (x_obs_est_Qwxyz_q * x_prior_est_Qwxyz_q.inverse) # get quaternion error
    e__log = Q_log(e__.elements) # get the error rotation from subtracting two orientation
    self.y_TVWQxyz[9:12,0] = [e__log[0],e__log[1],e__log[2]] # load quat to
    self.K = self.K * self.K_scale
    ky = dot(self.K, self.y_TVWQxyz)
    self.x_post_TVQxyz = x_prior_TVQxyz_tmp + ky # dot(self.K, self.y)
    temp_exp_map = exp_map(self.T_*ky[6:9]) # quaternion correction
    # equation 6 from EKF2 paper # update quaternion
    exp_map_ = Quaternion([temp_exp_map[3],temp_exp_map[0],temp_exp_map[1],temp_exp_map[2]]) \
      * Quaternion(self.x_prior_TVQwxyz[6:10,0])  ## wxyz format
    self.x_post_TVQxyz[6:9,0] = exp_map_.elements[1:4] # load quat xyz to x_post
    I_KH = self._I - dot(self.K, self.H)
    self.P_post = dot(I_KH, self.P_prior).dot(I_KH.T) + dot(self.K, self.R).dot(self.K.T)
    self.log.log_update(self.y_TVWQxyz, self.x_post_TVQxyz, self.P_post, self.K)
    return

  def predict_x(self, u=0):
    # estimation model
    # eq 16-22 of QEKF2
    # est lin pos and lin vel
    self.x_prior_TVQwxyz[0:3,0] = self.x_post_TVQxyz[0:3,0]+\
                                  self.T_*self.x_post_TVQxyz[3:6,0]
    self.x_prior_TVQwxyz[3:6,0] = self.x_post_TVQxyz[3:6,0]
    # est rot (quat)
    # get observed angular velocity
    z_Wrpy = self.z_TVWQxyzw[6:9,0]
    z_Wrpy = np.expand_dims(z_Wrpy, axis=1)
    # calc observed rotation based on angular rate and delta t
    x_obs_est_Qxyzw = exp_map(self.T_* self.C.T @ z_Wrpy)
    #x_obs_est_Qxyzw = exp_map(self.T_ * z_Wrpy)
    # convert rotation matrix to unit quaternion
    # CHECK: this quaternion obj must be a unit quaternion meaning W is probably non-zero.
    x_obs_est_Qwxyz = Quaternion(x_obs_est_Qxyzw[3],x_obs_est_Qxyzw[0],x_obs_est_Qxyzw[1],x_obs_est_Qxyzw[2]) ##w,x,y,z
    x_prior_blf_Qwxyz = Quaternion(self.get_Qwxyz_from_Qxyz(self.x_post_TVQxyz[6:9]))
    # QEKF02: equation 18 - quaternion estimation
    x_post_Qwxyz = (x_obs_est_Qwxyz * x_prior_blf_Qwxyz)
    self.x_prior_TVQwxyz[6:10,0] = x_post_Qwxyz.elements ##wxyz
    return

  def predict(self, u=0):

    #todo: change this from C = z_{q, k+1} to C = x_{q, k}^{+}
    r = R.from_quat([ self.x_prior_TVQwxyz[7,0], self.x_prior_TVQwxyz[8,0], \
                      self.x_prior_TVQwxyz[9,0], self.x_prior_TVQwxyz[6,0]])
    self.C = r.as_matrix()
    # nprint("r.as_rotvec() -  xyz-rpy", r.as_rotvec())
    # nprint("r.as_quat() -  xyzw", r.as_quat())
    self.set_H()
    self.set_L()
    self.set_F() #
    self.predict_x()
    Q_k = self.T_ * self.F @ self.L @ self.Q_c @ self.L.T @ self.F.T
    self.P_prior = dot(self.F, self.P_post).dot(self.F.T) + Q_k
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
    #self.H[0:3,0:3] = np.zeros((3,3))
    #self.H[0:3,3:6] = -self.C.T ## go away?
    #self.H[0:3,6:9] = -self.C.T @ get_skew_symm_X(self.z_TVWQxyzw[6:9,0])
    #self.H[3:6,3:6] = -self.C.T
    #self.H[6:9,6:9] = np.eye(3)
    self.H[0:6,0:6] = np.eye(6)
    self.H[9:12,6:9] = np.eye(3)
    #self.H[0:3,6:9] = -self.C.T @ get_skew_symm_X(self.z_TVWQxyzw[6:9,0])
    return

  def set_L(self):
    ## QEKF2 L matrix
    self.L = -np.eye(self.dim_x)
    self.L[3:6,3:6] = -self.C.T
    #self.L[0:3,0:3] = 0
    #self.L[6:9,6:9] = -np.eye(3)
    return self

  def get_Qwxyz_from_Qxyz(self, xyz: np.array):
    ''' Calculate the real term (w), given the imaginary terms (xyz)
      calculates unit quaternion from Qxyz (point quaternion)
    '''
    # sqrt(1-x^2-y%2-z^2) to confirm real part calc
    w = np.sqrt(1 -xyz[0]**2 -xyz[1]**2 -xyz[2]**2)
    return [w, xyz[0], xyz[1], xyz[2]]

  def get_losses(self, res:pd.DataFrame, output_dir:str, save_en:bool=True, prt_en:bool=True):
    L1 = list()
    L2 = list()
    # nprint_2('get_L2loss: res', res.head(5))

    # L2 = np.zeros((len(res.index), len(res.columns)))
    for i in range(len(res.index)):
      state_l1 = 0.0
      state_l2 = 0.0
      for j in range(len(res.columns)):
        # st()
        l1 = abs(res.iloc[i,j])
        l2 = res.iloc[i,j] ** 2
        state_l1 += l1
        state_l2 += l2
        # nprint(shorthead+'row sum ', res.iloc[i,:].sum())
      L1.append(state_l1)
      L2.append(state_l2)
    L1_df = pd.DataFrame(L1, columns=['L1'])
    L2_df = pd.DataFrame(L2, columns=['L2'])
    res = pd.concat([res,L1_df, L2_df], axis=1)
    # nprint_2('get_L2loss: res', res.head(5))
    # st()
    if save_en==True and  output_dir is not None:
      file_name = output_dir+'losses.txt'
      # if os.path.exists(file_name):
      with open(file_name, 'a+') as f:
        L1_str = shorthead+f"L1 (total): {res['L1'].sum()}"
        L2_str = shorthead+f"L2 (total): {res['L2'].sum()}"
        f.write(L1_str)
        f.write(L2_str+'\n\n')
        f.close()


    return res


# EOF
