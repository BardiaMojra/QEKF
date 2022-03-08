
# from pyquaternion import Quaternion
import quaternion
import numpy as np
import scipy.linalg as linalg
from pdb import set_trace as st
from pprint import pprint as pp
from numpy import dot, dtype, zeros, eye
from scipy.linalg import norm
from scipy.spatial.transform import Rotation as R


from dmm import *
from dlm import *
from util import *

''' general config '''
from nbug import *
from pdb import set_trace as st

class QEKF(object):
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
               K_scale=1.0):

    self.K_scale = K_scale
    self.dim_x = dim_x
    self.dim_z = dim_z
    self.dim_u = dim_u

    ''' state vectors'''
    self.x_TVQxyz = np.zeros((dim_x,1), dtype=np.float64) + .0001
    self.P = np.eye(dim_x) * P_est_0  # uncertainty covariance
    self.F = np.eye(dim_x) # state transition matrix
    self.R = np.eye(dim_z) # state uncertainty
    self.Q_c = np.eye(dim_x) # process uncertainty
    # self.y_TVQxyz = np.zeros((dim_z, 1)) # residual
    self.T_ = deltaT #time-period
    self.K = np.zeros((dim_x,1)) # kalman gain
    self.K_scale = K_scale
    self.S = np.zeros((dim_z, dim_z))   # system uncertainty
    self.SI = np.zeros((dim_z, dim_z))  # inverse system uncertainty
    self.L = None
    self._I = np.eye(dim_x)
    self.C = np.zeros((3,3)) # rotation matrix
    self.H = np.zeros((dim_z, dim_x)) #3x10
    ''' init the process iid noise covar matrix Q - (dim_x by dim_x) '''
    self.Q_c = np.diag(np.array([Q_T_xyz,
                                 Q_T_xyz,
                                 Q_T_xyz,
                                 Q_V_xyz,
                                 Q_V_xyz,
                                 Q_V_xyz,
                                 Q_quat_xyz,
                                 Q_quat_xyz,
                                 Q_quat_xyz]))

    ''' init the measurement iid noise covar matrix R - (dim_z by dim_z) '''
    self.R = np.diag(np.array([R_noise,
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


  def update(self, x_TVQxyz, z_TVQxyz):
    # compute Kalman gain
    PHT = dot(self.P, self.H.T)
    self.S = dot(self.H, PHT) + self.R
    self.K = PHT.dot(linalg.inv(self.S))
    self.K = self.K * self.K_scale
    ''' lin part '''
    hx = np.dot(self.H, x_TVQxyz)
    y_TVQ = np.subtract(z_TVQxyz, hx) # TVWQxyz
    x_TVQ_post = np.zeros((self.dim_x,1))
    Ky = dot(self.K, y_TVQ)
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
    I_KH = self._I - dot(self.K, self.H)
    self.P = dot(I_KH, self.P).dot(I_KH.T) + dot(self.K, self.R).dot(self.K.T)
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

  def predict(self, x_TVQxyz:np.ndarray, u_Wrpy:np.ndarray):
    self.set_C(get_Qxyzw(x_TVQxyz[6:9,0]))
    self.set_H()
    self.set_L()
    self.set_F(u_Wrpy) #
    x_TVQxyz = self.predict_x(x_TVQxyz, u_Wrpy)
    Q_k = self.T_ * self.F @ self.L @ self.Q_c @ self.L.T @ self.F.T
    self.P = dot(self.F, self.P).dot(self.F.T) + Q_k
    return x_TVQxyz

  def set_F(self, u_Wrpy:np.ndarray):
    self.F = np.eye(self.dim_x)
    self.F[0:3,3:6] = self.T_*np.eye(3)
    self.F[6:9,6:9] = np.eye(3) - self.T_*get_skew_symm_X(u_Wrpy)
    return

  def set_H(self):
    # set measurement transition function (H matrix)
    self.H[0:9,0:9] = np.eye(9)
    self.H[0:3,0:3] = -self.C
    return

  def set_L(self):
    ## QEKF2 L matrix
    self.L = -np.eye(self.dim_x)
    self.L[3:6,3:6] = -self.C.T
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


# EOF
