
import numpy as np
import matplotlib as mpl
import mpl.pyplot as plt
import pandas as pd
import time
from scipy.spatial.transform import Rotation as R
from pyquaternion import Quaternion
from qekf import ExtendedKalmanFilter as QEKF

''' private libraries
'''
from dmm import *
from util import *

''' NBUG libraries
'''
from pdb import set_trace as st
from pprint import pprint as pp


''' NBUG config '''
NBUG = True
debug = False #
# debug = True

''' general config '''
adaptive_ekf_window = 25
print_output = True
_show = True # show plots?
_save = True
_prt = False
longhead = '\n\n  \--->> '
shorthead = '\n  \--->> '
longtail = '\n\n'
shorttail = '\n'
attn = 'here ----------- <<<<<\n\n'  #print(longhead+attn)

''' matplotlib config '''
mpl.pyplot.ion()
plt.style.use('ggplot')

''' random number generator seed '''
np.random.seed(5555)


def main():
  #data = 'iphone1_clean'
  #data = 'bigC_06-Aug2021'
  data = 'kitti_imu_001'

  if data == 'iphone1_clean':
    dName = 'dataset-iphone1_clean'
    src_dir = '../data/dataset-iphone1_clean/'
    output_dir = '../out/out_iphone1_clean/'
    file_ext = 'xlsx'
    opt = None
  elif data =='bigC_06-Aug2021':
    dName = 'bigC_06-Aug2021'
    src_dir = '../data/bigC_06-Aug2021/'
    output_dir = '../out/out_bigC_06-Aug2021/'
    file_ext = 'csv'
    opt = ' ' # space separator for csv file
  elif data =='kitti_imu_001':
    dName = 'kitti_imu_001'
    src_dir = '../data/kitti/2011_09_26/'
    output_dir = '../out/out_bigC_06-Aug2021/'
    file_ext = 'csv'
    opt = ' '
  else:
    eprint(longhead+'Err--->> no data selected.....\n\n')

  dataset = dmm(name=dName,
    source_dir=src_dir,
    ext=file_ext,
    dtype='float64',
    output_dir=output_dir,
    prt=_prt,
    options=opt)

  dataset.format()
  dataset.plot(labels=dataset.labels, title='Quest and Vest Data', show=_show)
  dataset.plot_trans_3d(title='Quest Translation in 3D', show=_show)

  '''
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
  '''

  ''' EKF Config
  '''
  x_state_dim = 9 #
  z_meas_dim = 12 #
  df_len = dataset.df.shape[0]
  start_= 0
  deltaT = 1/30
  # Q noise
  T_xyz_noise = 1e5
  V_xyz_noise = 1.5e2
  Q_xyz_noise = 0.5e-6
  # R noise
  r_noise = 0.1e3
  P_est_init = 1e-4
  VScale = 1


  ''' main EKF program
  '''

  # EKF for Quest and Vest
  drk = QEKF(dim_x=x_state_dim,
             dim_z=z_meas_dim,
             deltaT=deltaT,
             Q_T_xyz=T_xyz_noise,
             Q_V_xyz=V_xyz_noise,
             Q_quat_xyz=Q_xyz_noise,
             R_noise=r_noise,
             P_est_0=P_est_init)

  drk.x_qwxyz = Quaternion(dataset.quat_wxyz.head(1).to_numpy()[0])

  for i in range(start_, start_+df_len-1):
    if NBUG:
      print(longhead+"new state:")
    drk.get_z_TVWrpyQxyzw(lin_vel=dataset.vel_xyz.to_numpy()[i,:],\
      translation=dataset.trans_xyz.to_numpy()[i,:],\
      ang_vel=dataset.vel_rpy.to_numpy()[i,:],\
      quat=dataset.quat_xyzw.to_numpy()[i,:],\
      Vscale=VScale)

    drk.log.log_z_state(z=drk.z_TVWrpyQxyzw, idx=i)

    prt_z = True
    if prt_z is True and NBUG is True:
      print(shorthead+"drk.z_TVWrpyQxyzw[9:13]: _xyzw_")
      print(drk.z_TVWrpyQxyzw[9:13])
      print(shorttail)

    prt_z_q = True
    if prt_z_q and NBUG:
      print(shorthead+'drk.x_qwxyz:');
      print(drk.x_qwxyz);
      print(shorttail)

    r = R.from_quat([drk.z_TVWrpyQxyzw[12,0], drk.z_TVWrpyQxyzw[9,0],\
      drk.z_TVWrpyQxyzw[10,0], drk.z_TVWrpyQxyzw[11,0]])
    drk.C = r.as_matrix()

    prt_rot = True
    if prt_rot is True and NBUG is True:
      print(shorthead+"r_rot:xyz vec")
      print(r.as_rotvec())
      print(shorttail)


    # set measurement transition function (H matrix)
    drk.H[0:3,0:3] = -drk.C.T
    drk.H[0:3,3:6] = -drk.C.T ## go away?
    drk.H[0:3,6:9] = -drk.C.T @ get_skew_symm_X(drk.x[3:6,0])
    # drk.H[3:6,0:3] = np.zeros(3)
    drk.H[3:6,3:6] = -drk.C.T
    # drk.H[3:6,6:9] = np.zeros(3)
    # drk.H[6:9,0:3] = np.zeros(3)
    drk.H[6:9,6:9] = np.eye(3)

    ## QEKF2 L matrix
    drk.L = np.eye(x_state_dim)
    drk.L[3:6,3:6] = -drk.C.T # todo:
    drk.L[0:3,0:3] = 0
    drk.L[6:9,6:9] = -np.eye(3)

    drk.predict()

    drk.update(drk.z_TVWrpyQxyzw.T)


  print(longhead+'end of QEKF processing...'+longtail)
  #st()

  #
  quat_meas_df = pd.DataFrame(drk.log.z_hist[:,9:13],
                              index=drk.log.idx,
                              columns=['qx', 'qy', 'qz', 'qw'])# change to xyzw
  # x_quat_wxyz
  quat_est_df = pd.DataFrame(drk.log.x_prior_hist[:,6:10],\
    index=drk.log.idx,\
    columns=['qw', 'qx', 'qy', 'qz'])

  print_quat_df_heads = False
  if print_quat_df_heads is True and NBUG is True:
    print(longhead+'quat data:')
    print(shorthead+'quat_meas_df:\n')
    print(quat_meas_df.head(3))
    print(shorthead+'quat_est_df:\n')
    print(quat_est_df.head(3))

  # plot EKF output
  plot_quat_vs_quat(quat_A_df=quat_meas_df,
    quat_B_df=quat_est_df,
    title='z vs x_prior free range',
    figname='fig_08',
    show=_show,
    colors=['maroon','darkgoldenrod'],
    save=True,
    output_dir=output_dir,
    start=start_,
    labels=['meas.', 'est.'])

  # plot EKF output
  plot_quat_vs_quat(quat_A_df=quat_meas_df,
    quat_B_df=quat_est_df,
    title='z vs x_prior unit range',
    figname='fig_09',
    show=_show,
    colors=['maroon','darkgoldenrod'],
    save=True,
    output_dir=output_dir,
    start=start_,
    labels=['meas.', 'est.'],
    y_range=[-1.1,1.1])

  residual_df = pd.DataFrame(drk.log.v_hist,
    index=drk.log.idx,
    columns=['Tx', 'Ty', 'Tz',\
             'vx', 'vy', 'vz',\
             'wr', 'wp', 'wy',\
             'qx', 'qy', 'qz'])

  prt_res_v_head = False
  if prt_res_v_head is True and NBUG is True:
    print(longhead+'v residual: '+str(residual_df.shape)+'\n')
    print(residual_df.head(5))
    print(longtail)

  plot_df(df=residual_df,
    rows=12,
    cols=1,
    title='v residual',
    show=True,
    figname='fig_10',
    output_dir=output_dir)

  # z_TVWrpyQxyzw
  z_df = pd.DataFrame(drk.log.z_hist,
    index=drk.log.idx,\
    columns=['Tx', 'Ty', 'Tz',\
             'vx', 'vy', 'vz',\
             'wr', 'wp', 'wy',\
             'qx', 'qy', 'qz', 'qw'])

  x_prior_df = pd.DataFrame(drk.log.x_prior_hist,
    index=drk.log.idx,\
    columns=['Tx', 'Ty', 'Tz',\
             'vx', 'vy', 'vz',\
             'qw', 'qx', 'qy', 'qz'])

  x_post_df = pd.DataFrame(drk.log.x_post_hist,
    index=drk.log.idx,\
    columns=['Tx', 'Ty', 'Tz',\
             'vx', 'vy', 'vz',\
             'qx', 'qy', 'qz'])

  prt_df_vs_heads = True
  if prt_df_vs_heads is True and NBUG is True:
    print(shorthead+'z meas vector: '+str(z_df.shape)+'\n')
    print(z_df.head(5))
    print(shorttail)
    print(shorthead+'x_prior est vector: '+str(x_prior_df.shape)+'\n')
    print(x_prior_df.head(5))
    print(shorttail)
    print(shorthead+'x_posterior est vector: '+str(x_post_df.shape)+'\n')
    print(x_post_df.head(5))
    print(shorttail)

  z_Txyz_df = pd.DataFrame(drk.log.z_hist[:,0:3],\
    index=drk.log.idx,
    columns=['Tx', 'Ty', 'Tz'])

  x_post_Txyz_df = pd.DataFrame(drk.log.x_post_hist[:,0:3],\
    index=drk.log.idx,
    columns=['Tx', 'Ty', 'Tz'])

  plot_Txyz_vs_Txyz_3d(z_Txyz_df,
    x_post_Txyz_df,
    title='z vs x_posterior translation',
    figname='fig_11',
    show=_show,
    labels=['z', 'x_post'],
    output_dir=output_dir)

  # z_TVWrpyQxyz
  z_TVQxyz_df = pd.DataFrame(drk.log.z_hist[:,[0,1,2,3,4,5,9,10,11]],\
    index=drk.log.idx,
    columns=['Tx', 'Ty', 'Tz',\
             'vx', 'vy', 'vz',\
  #           'wr', 'wp', 'wy',\
             'qx', 'qy', 'qz'])

  x_post_TVQxyz_df = pd.DataFrame(drk.log.x_post_hist,\
    index=drk.log.idx,
    columns=['Tx', 'Ty', 'Tz',\
             'vx', 'vy', 'vz',\
  #           'wr', 'wp', 'wy',\
             'qx', 'qy', 'qz'])

  if prt_df_vs_heads is True:
    print(longhead+'new z:'+str(z_TVQxyz_df.shape))
    print(z_TVQxyz_df.head(5))
    print(shorthead+'x_prior:'+str(x_post_TVQxyz_df.shape))
    print(x_post_TVQxyz_df.head(5))

  plot_z_df_vs_x_df(z_TVQxyz_df,
    x_post_TVQxyz_df,
    labels=['z', 'x_post'],
    title='z meas vs x_posterior est',
    figname='fig_12',
    show=True,
    output_dir=output_dir)

  plot_df(df=z_df,
    rows=13,
    cols=1,
    title='z meas',
    figname='fig_13',
    show=True,
    output_dir=output_dir)


  # x_quat_wxyz
  '''x_q_df = pd.DataFrame(drk.log.x_q_hist,\
    index=drk.log.idx,\
    columns=['qw', 'qx', 'qy', 'qz'])

  x_prior_q_df = pd.DataFrame(drk.log.x_prior_q_hist,\
    index=drk.log.idx,\
    columns=['qw', 'qx', 'qy', 'qz'])

  prt_x_q_vs_x_prior_q = True
  if prt_x_q_vs_x_prior_q is True and NBUG is True:
    print(longhead+'x_q_df:'+str(x_q_df.shape))
    print(x_q_df.head(5))
    print(shorthead+'x_prior_q_df:'+str(x_prior_q_df.shape))
    print(x_prior_q_df.head(5))

  # plot EKF output
  plot_z_df_vs_x_df(x_q_df,
    x_prior_q_df,
    labels=['x_q', 'x_prior'],
    rows=4,cols=1,
    title='x_q vs x_prior Quaternions',
    figname='fig_13',
    show=_show,
    figsize=[8,10])
  '''

  st()

  '''---------   End of QEKF  -->>>>>>>>>>>>>>>>>'''

  return # end of main

if __name__ == "__main__":
  main()
  print('---------   End   ---------')
  st()
  time.sleep(1000)
  exit()
