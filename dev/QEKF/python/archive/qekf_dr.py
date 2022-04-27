
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



class ExtendedKalmanFilter(object):
  ''' Simple Kalman Filter:
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
    self.x_TVQxyz = np.zeros((dim_x,1)) + .0001
    self.z_Qxyz = np.zeros((dim_z,1))
    self.u_AWrpy = np.zeros((dim_u,1))
    self.P = np.eye(dim_x) * P_est_0  # uncertainty covariance
    self.F = np.eye(dim_x)     # state transition matrix
    self.Q_c = np.eye(dim_z)        # process uncertainty
    # self.y_Qxyz = np.zeros((dim_z, 1)) # residual
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

    ''' init the measurement iid noise covar matrix R -- 4X4 diagonal (dim_z by dim_z)
    '''
    self.R = np.diag(np.array([R_noise,
                               R_noise,
                               R_noise]))

    # data logger
    self.log = dlm(enabled=log)
    self.plotter = dmm
    ## end  of init

  def update(self, x_TVQxyz, z_Qxyz):
    # nsprint('x_TVQxyzw', x_TVQxyz)
    # nsprint('u_AWrpy', u_AWrpy)
    # nsprint('z_Qxyzw', z_Qxyz)
    # nppshape('self.P', self.P)
    # nppshape('self.H.T', self.H.T)
    # nppshape('self.R', self.R)
    # compute Kalman gain
    PHT = dot(self.P, self.H.T)
    self.S = dot(self.H, PHT) + self.R
    self.K = PHT.dot(linalg.inv(self.S))
    # nppshape('self.S', self.S)
    # nppshape('self.K', self.K)
    # st()
    ''' lin part
    '''
    # hx = np.dot(self.H, x_TVQxyz)
    # nsprint('hx.T', hx.T)
    # self.y_Qxyzw = np.subtract(z_Qxyzw, hx) # TVWQxyz
    ''' quat part
    '''
    x_q = Quaternion(get_Qwxyz(x_TVQxyz[6:9,0])) # wxyz input
    z_q = Quaternion(get_Qwxyz(z_Qxyz[0:3,0]))
    # nprint('x_q', x_q)
    # nprint('z_q', z_q)
    # st()
    err_x_q = x_q * z_q.inverse # get quaternion error
    # nprint('err_x_q', err_x_q)
    # st()
    y_PHIxyz = Q_log(err_x_q.elements) # get rotation error
    # st()
    ky_PHIrpy = np.matmul(self.K, y_PHIxyz)
    # nsprint('ky_Qxyz', ky_PHIrpy)
    x_q_corr = exp_map(self.T_*ky_PHIrpy[0:3,0]) # quaternion correction
    # nsprint('x_q_corr', x_q_corr)
    x_q_corr = Quaternion(get_Qwxyz(x_q_corr[0:3]))
    # nprint('x_q_corr', x_q_corr)
    # st()
    # equation 6 from EKF2 paper # update quaternion
    x_q_post = x_q_corr * x_q  ## wxyz format
    x_TVQxyz[6:9,0] = x_q_post.elements[1:4] # from wxyz load quat xyz to x_post
    # nsprint('x_TVQxyzw[6:9,0]', x_TVQxyz[6:9,0])
    # st()
    # x_TVQxyz[9,0] = get_Qwxyz(x_q_post.elements[1:4])[0] # load quat xyz to x_post
    # nsprint('x_TVQxyzw[6:9,0]', x_TVQxyz[6:9,0])
    I_KH = self._I - dot(self.K, self.H)
    self.P = dot(I_KH, self.P).dot(I_KH.T) + dot(self.K, self.R).dot(self.K.T)

    ''' log state vector '''
    x_TVQxyzw = np.ndarray((self.dim_x+1,1))
    # x_TVQxyzw[:6,0] = x_TVQxyz[:6,0]
    x_TVQxyzw[6:10,0] = get_Qxyzw(x_TVQxyz[6:9,0])
    # nsprint('x_TVQxyzw', x_TVQxyzw)
    # st()
    self.log.log_update(y_PHIxyz, x_TVQxyzw, self.P, self.K)

    return x_TVQxyz

  def predict_x(self, x_TVQxyz, u_FWrpy):
    ''' estimation model
      - eq 16-22 of QEKF2
      - this routine is essentially discrete form of \hat{x}_{k|k-1} =\
        f(\hat{x}_{k-1|k-1}, u_{k})
    '''
    u_Fxyz = u_FWrpy[0:3]
    u_Wrpy = u_FWrpy[3:6]

    # est linPos
    x_TVQxyz[0:3] = x_TVQxyz[0:3]+self.T_*x_TVQxyz[3:6]+\
      ((self.T_)**2/2.0)*np.dot(self.C.T , u_Fxyz)
    # est linVel
    x_TVQxyz[3:6] = x_TVQxyz[3:6] + self.T_*(self.C.T @ u_Fxyz)

    ''' est rotVec (quat) -- eq(18) '''
    # est incremental rotation (in quat) based on input angVel (Wrpy) and delta t
    u_Qxyzw = exp_map(self.T_ * u_Wrpy)
    u_Qwxyz = Quaternion(get_Qwxyz(u_Qxyzw[0:3]))
    x_Qwxyz = Quaternion(get_Qwxyz(x_TVQxyz[6:9,0]))
    x_Qwxyz = u_Qwxyz * x_Qwxyz
    x_TVQxyz[6:9,0] = x_Qwxyz.elements[1:4]
    return x_TVQxyz

  def predict(self, x_TVQxyz:np.ndarray, u_FWrpy:np.ndarray):
    self.set_C(get_Qxyzw(x_TVQxyz[6:9,0]))
    self.set_H()
    self.set_L()
    self.set_F(u_FWrpy) #
    x_TVQxyz = self.predict_x(x_TVQxyz, u_FWrpy)
    Q_k = self.T_ * self.F @ self.L @ self.Q_c @ self.L.T @ self.F.T
    self.P = dot(self.F, self.P).dot(self.F.T) + Q_k
    return x_TVQxyz

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
    self.H[:,6:9] = np.eye(3)
    # nsprint('self.H', self.H)
    # st()
    return

  def set_L(self):
    ## QEKF2 L matrix, based on eq26, 27, L_c
    self.L = np.zeros((self.dim_x,self.dim_x))
    self.L[3:6,3:6] = -self.C.T
    self.L[6:9,6:9] = -np.eye(3)
    # nsprint('self.L', self.L)
    # st()
    return

  def set_C(self, x_Qxyz:np.ndarray):
    ''' calculates state estimate (belief) rotation matrix (C) given
      the corresponding orientation in quaternion form.
    '''
    r = R.from_quat(x_Qxyz)
    self.C = r.as_matrix()
    return

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
      L1_str = shead+f"L1 (total): {res['L1'].sum()}"
      L2_str = shead+f"L2 (total): {res['L2'].sum()}"
      f.write(L1_str)
      f.write(L2_str+'\n\n')
      f.close()
  return res

def print_losses(df: pd.DataFrame):
  print(shead+"L1 (total): ", df['L1'].sum())
  print(shead+"L2 (total): ", df['L2'].sum())
  return




# EOF
