
from pyquaternion import Quaternion
import numpy as np
import scipy.linalg as linalg
from numpy import dot, zeros, eye
from scipy.linalg import norm
from scipy.spatial.transform import Rotation as R

from dmm_dr import *
from dlm_dr import *
from util_dr import *

from nbug import *
from pdb import set_trace as st
from pprint import pprint as pp

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

    ''' state vectors '''
    self.x_TVQxyzw = np.zeros((dim_x,1)) + .0001
    self.x_TVQxyzw[-1,0] = 0.9998
    self.z_Qxyzw = np.zeros((dim_z,1))
    self.u_AWrpy = np.zeros((dim_u,1))

    self.P = np.eye(dim_x) * P_est_0  # uncertainty covariance
    self.F = np.eye(dim_x)     # state transition matrix
    self.Q_c = np.eye(dim_x)        # process uncertainty
    self.y_Qxyzw = np.zeros((dim_z, 1)) # residual
    self.T_ = deltaT #time-period
    self.K = np.zeros((dim_x,1)) # kalman gain -- 9
    self.S = np.zeros((dim_z, dim_z))   # system uncertainty
    self.SI = np.zeros((dim_z, dim_z))  # inverse system uncertainty
    self.L = None
    # identity matrix. Do not alter this.
    self._I = np.eye(dim_x)
    self.C = np.zeros((3,3)) # rotation matrix
    self.H = np.zeros((dim_z, dim_x)) #4x10
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

    ''' init the measurement iid noise covar matrix R -- 9x9 diagonal (dim_z by dim_z)
    '''
    self.R = np.diag(np.array([R_noise,
                               R_noise,
                               R_noise]))

    # data logger
    self.log = dlm(enabled=log)
    self.plotter = dmm
    ## end of init

  def update(self, x_TVQxyzw, z_Qxyzw):
    if z_Qxyzw is None:
      eprint(longhead+'Err: in update(), z is None and z vect is created...'+longtail)

    # compute Kalman gain
    PHT = dot(self.P, self.H.T)
    self.S = dot(self.H, PHT) + self.R
    self.K = PHT.dot(linalg.inv(self.S))
    # modified for qekf and quarternion states

    # x_prior_TVQxyz_tmp = zeros((self.dim_x,1))
    # x_prior_TVQxyz_tmp[0:6] = self.x_TVQwxyz[0:6]
    # x_prior_TVQxyz_tmp[6:9] = self.x_TVQwxyz[7:10]

    st()
    nprint('self.H',self.H)
    nprint('self.H.shape',self.H.shape)
    nprint('x_TVQwxyz', x_TVQxyzw)
    nprint('x_TVQwxyz.shape', x_TVQxyzw.shape)
    st()

    hx = np.dot(self.H, x_TVQxyzw)
    nsprint('hx', hx)
    st()
    ''' lin part
    '''
    self.y_TVWQxyz = np.subtract(self.xz_TVWrpyQxyzwFxyz[0:12], hx.T).T # TVWQxyz
    ''' quat part
    '''
    x_prior_est_Qwxyz_q = Quaternion(self.x_TVQwxyz[6:10]) # wxyz input
    x_obs_est_Qwxyz_q = Quaternion(self.xz_TVWrpyQxyzwFxyz[12], self.xz_TVWrpyQxyzwFxyz[9],
                                   self.xz_TVWrpyQxyzwFxyz[10], self.xz_TVWrpyQxyzwFxyz[11])
    e__ = (x_obs_est_Qwxyz_q * x_prior_est_Qwxyz_q.inverse) # get quaternion error
    e__log = Q_log(e__.elements) # get the error rotation from subtracting two orientation
    self.y_TVWQxyz[9:12] = [e__log[0],e__log[1],e__log[2]] # load quat to
    self.K = self.K * self.K_scale
    ky = dot(self.K, self.y_TVWQxyz)
    self.x_post_TVQxyz = x_prior_TVQxyz_tmp + ky # dot(self.K, self.y)
    temp_exp_map = exp_map(self.T_*ky[6:9]) # quaternion correction
    # equation 6 from EKF2 paper # update quaternion
    exp_map_ = Quaternion([temp_exp_map[3],temp_exp_map[0],temp_exp_map[1],temp_exp_map[2]]) \
      * Quaternion(self.x_TVQwxyz[6:10])  ## wxyz format
    self.x_post_TVQxyz[6:9] = exp_map_.elements[1:4] # load quat xyz to x_post
    I_KH = self._I - dot(self.K, self.H)
    self.P = dot(I_KH, self.P).dot(I_KH.T) + dot(self.K, self.R).dot(self.K.T)
    self.log.log_update(self.y_TVWQxyz, self.x_post_TVQxyz, self.P, self.K)
    return

  def predict_x(self, x_TVQxyzw, u_FWrpy):
    ''' estimation model
      - eq 16-22 of QEKF2
      - this routine is essentially discrete form of \hat{x}_{k|k-1} =\
        f(\hat{x}_{k-1|k-1}, u_{k})
    '''

    nsprint('x_TVQxyzw.T', x_TVQxyzw.T)
    nsprint('u_FWrpy.T', u_FWrpy.T)

    # nprint(longtail+'dev paused here....'+longtail)
    u_Fxyz = u_FWrpy[0:3]
    u_Wrpy = u_FWrpy[3:6]

    # est linPos
    x_TVQxyzw[0:3] = x_TVQxyzw[0:3]+self.T_*x_TVQxyzw[3:6]+\
      ((self.T_)**2/2.0)*np.dot(self.C.T , u_Fxyz)

    # nsprint('x_TVQxyzw[0:3]', x_TVQxyzw[0:3])
    # nsprint('x_TVQxyzw', x_TVQxyzw)

    # est linVel
    x_TVQxyzw[3:6] = x_TVQxyzw[3:6] + self.T_*(self.C.T @ u_Fxyz)

    # nsprint('x_TVQxyzw[6:10]', x_TVQxyzw[6:10])
    # nsprint('x_TVQxyzw', x_TVQxyzw)

    ''' est rotVec (quat) -- eq(18)
    '''
    # get z_FWQ some measurements are considered as system input such as force
    # get observed angular velocity
    # u_Wrpy = self.xz_TVWrpyQxyzwFxyz[6:9]
    # z_Wrpy = np.expand_dims(z_Wrpy, axis=1)
    # est incremental rotation (in quat) based on input angVel (Wrpy) and delta t
    # obs_Qxyzw = exp_map(self.T_* self.C.T @ u_Wrpy)
    _q = exp_map(self.T_ * u_Wrpy)
    _q = np.array(_q, dtype=np.float64)
    # _q = [_q[3],_q[0],_q[1],_q[2]]
    nsprint('_q', _q)
    st()
    # convert rotation matrix to unit quaternion
    # CHECK: this quaternion obj must be a unit quaternion meaning W is probably non-zero.
    __obs_Qwxyz = Quaternion(_q[3],_q[0],_q[1],_q[2])
    x_Qwxyz = Quaternion(self.get_Qwxyz_from_Qxyz(x_TVQxyzw[6:9]))
    # QEKF02: equation 18 - quaternion estimation
    nsprint('x_Qwxyz', x_Qwxyz)
    nsprint('__obs_Qwxyz', __obs_Qwxyz)
    x_Qwxyz = __obs_Qwxyz * x_Qwxyz
    nsprint('x_Qwxyz', x_Qwxyz)
    nsprint('x_TVQxyzw',x_TVQxyzw)
    x_TVQxyzw[6:9] = x_Qwxyz[1:4]
    x_TVQxyzw[10] = x_Qwxyz[0]
    nsprint('x_TVQxyzw',x_TVQxyzw)

    return x_TVQxyzw

  def predict(self, x_TVQxyzw:np.ndarray, u_FWrpy:np.ndarray):
    self.set_C(x_TVQxyzw[6:10,0])
    self.set_H()
    self.set_L()
    self.set_F(u_FWrpy) #
    x_TVQxyzw = self.predict_x(x_TVQxyzw, u_FWrpy)
    Q_k = self.T_ * self.F @ self.L @ self.Q_c @ self.L.T @ self.F.T
    self.P = dot(self.F, self.P).dot(self.F.T) + Q_k

    self.log.log_prediction(x_TVQxyzw, self.P)
    return

  # def partial_update(self,gamma,beta):
  #   for i in range(self.dim_x):
  #     self.x[i] = gamma[i]*self.x_post_TVQxyz[i] + (1-gamma[i])*self.x_TVQwxyz[i]
  #     for j in range(self.dim_x):
  #       self.P_post[i,j] = gamma[i]*gamma[j]*self.P_post[i,j]+(1-gamma[i]*gamma[j])*self.P_prior[i,j]

  def set_F(self, u_FWrpy:np.ndarray):
    self.F = np.eye(self.dim_x)
    self.F[0:3,3:6] = self.T_*np.eye(3)
    self.F[3:6,6:9] = -self.T_* self.C.T @ get_skew_symm_X(u_FWrpy[0:3])
    self.F[6:9,6:9] = np.eye(3)-self.T_*get_skew_symm_X(u_FWrpy[3:6])
    # nsprint('self.F', self.F)
    # st()
    return

  def set_H(self):
    # set measurement transition function (H matrix)
    self.H = np.zeros((self.dim_z, self.dim_x))
    self.H[:,6:10] = np.eye(4)
    # nsprint('self.H', self.H)
    # st()
    return

  def set_L(self):
    ## QEKF2 L matrix
    self.L = -np.eye(self.dim_x)
    self.L[3:6,3:6] = -self.C.T
    #self.L[0:3:3] = 0
    #self.L[6:9,6:9] = -np.eye(3)
    # nsprint('self.L', self.L)
    return self

  def set_C(self, x_Qxyzw:np.ndarray):
    ''' calculates state estimate (belief) rotation matrix (C) given
      the corresponding orientation in quaternion form.
    '''
    r = R.from_quat([x_Qxyzw[3], x_Qxyzw[0],x_Qxyzw[1], x_Qxyzw[2]])
    self.C = r.as_matrix()
    # nprint('r (rotation vector in radians)', r)
    # nsprint('self.C', self.C)
    # st()
    return

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
