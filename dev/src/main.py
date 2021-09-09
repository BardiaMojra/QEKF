
import numpy as np
import matplotlib
from matplotlib import pyplot as plt
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
from nbug import *



def get_fignum_str(fignum):
  ''' usage:
    fignum+=1;get_fignum_str(fignum)
  '''
  return 'fig_%03i' % fignum


''' general config '''
NBUG = True
print_output = True
_show = True
_save = True
_prt = False
_zoom = 10

''' matplotlib config '''
matplotlib.pyplot.ion()
plt.style.use('ggplot')

''' #TODO Research
  - Test on KITTI dataset and other datasets
  - Read ICRA papers on QEKF,
  - Look at QUEST and VEST citation and cited by papers
  - Look into the impact on other papers.
'''

def main():
  # globals
  global fignum
  fignum = int(0)

  # select dataset
  #data = 'dataset-iphone1_clean'
  #data = 'bigC_06-Aug2021'
  #data = 'Y2021M08D05_ZoomTwistJackal_BigC-off_ransac-off'
  #data = 'Y2021M08D05_zoom-twist-jackal_BigC-off_ransac-off'
  #data = 'Y2021M08D05_CircleAoundMetal_BigC-off_ransac-off' #TODO: name index bug: df.columns and labels
  #data = 'Y2021M08D05_BoxWalkKuka_BigC-off_ransac-off_Q-Select-on_FP-Last6'
  data = 'Y2021M08D06_BoxWalkKuka_BigC-off_ransac-off_Q-Select-off_FP-HighLow6'
  #data = 'kitti_imu_0926_0001'
  #data = 'kitti_imu_0926_0002'
  #data = 'kitti_imu_0926_0005'
  #data = 'kitti_imu_0926_0018'
  #data = 'kitti_imu_0926_0060'
  #data = 'kitti_imu_0926_0084' #todo: show this to Dr. Gans
  #data = 'kitti_imu_0926_0113' #todo: show this to Dr. Gans - Quat flips
  #data = 'kitti_imu_0928_0001'

  # init dataset object
  dset = dmm(name=data,
             VestScale=1,
             data_rate_inv=1/30,
             #start=0,
             #end=16,
             prt=_prt)

  dset.format()
  fignum+=1; get_fignum_str(fignum)
  dset.plot(labels=dset.df.columns, title=data, show=_show)
  fignum+=1; get_fignum_str(fignum)
  dset.plot_trans_3d(title='Ground Truth Translation', show=_show)

  # init QEKF object
  qekf = QEKF(dim_x=9,
              dim_z=12,
              deltaT=dset.data_rate_inv,
              Q_T_xyz=1.0e-5, # process noise covar
              Q_V_xyz=1.5e-2,
              Q_quat_xyz=0.5e-4,
              R_noise=1e-5, # measurement noise covar
              P_est_0=1e-4,
              K_scale=1.0)

  #qekf.x_post_Qwxyz= Quaternion(dset.quat_wxyz.head(1).to_numpy()[0])
  nprint('qekf.x_Qwxyz', qekf.x_post_TVQxyz)

  ''' init state'''
  qekf.x_post_TVQxyz[0] = dset.df.Tx.iloc[0]
  qekf.x_post_TVQxyz[1] = dset.df.Ty.iloc[0]
  qekf.x_post_TVQxyz[2] = dset.df.Tz.iloc[0]
  qekf.x_post_TVQxyz[3] = dset.df.vx.iloc[0]
  qekf.x_post_TVQxyz[4] = dset.df.vy.iloc[0]
  qekf.x_post_TVQxyz[5] = dset.df.vz.iloc[0]
  #qekf.x_post_TVQxyz[6] = dset.df.qw.iloc[0]
  qekf.x_post_TVQxyz[6] = dset.df.qx.iloc[0]
  qekf.x_post_TVQxyz[7] = dset.df.qy.iloc[0]
  qekf.x_post_TVQxyz[8] = dset.df.qz.iloc[0]

  for i in range(dset.start, dset.end):
    nprint(longhead+'   \\--->>> new state ------->>>>>', i)
    qekf.get_z_TVWQxyzw(lin_vel=dset.vel_xyz.to_numpy()[i,:],\
      translation=dset.trans_xyz.to_numpy()[i,:],\
      ang_vel=dset.vel_rpy.to_numpy()[i,:],\
      quat=dset.quat_xyzw.to_numpy()[i,:],\
      #quat=np.asarray([.001, .002, -.994, .003]),\
      Vscale=dset.VestScale)

    qekf.log.log_z_state(z=qekf.z_TVWQxyzw, idx=i)

    nprint('qekf.z_TVWQxyzw[9:13] - Qxyzw', qekf.z_TVWQxyzw[9:13])


    #todo move to predict routine?
    r = R.from_quat([ qekf.z_TVWQxyzw[9, 0], qekf.z_TVWQxyzw[10,0], \
                      qekf.z_TVWQxyzw[11,0], qekf.z_TVWQxyzw[12,0]])
    qekf.C = r.as_matrix()

    '''NBUG'''
    nprint("r.as_rotvec() -  xyz-rpy", r.as_rotvec())
    nprint("r.as_quat() -  xyzw", r.as_quat())

    qekf.predict()
    qekf.update(qekf.z_TVWQxyzw.T)

  # end of qekf data iterator ----->>
  nprint('end of qekf data iterator ----->>', '')


  ''' post processing
  '''
  quat_meas_df = pd.DataFrame(qekf.log.z_hist[:,9:13],
                              index=qekf.log.idx,
                              columns=['qx', 'qy', 'qz', 'qw'])# change to xyzw
  # x_quat_wxyz
  quat_est_df = pd.DataFrame(qekf.log.x_prior_hist[:,6:10],\
    index=qekf.log.idx,\
    columns=['qw', 'qx', 'qy', 'qz'])

  # plot EKF output
  fignum+=1; get_fignum_str(fignum)
  plot_quat_vs_quat(quat_A_df=quat_meas_df,
    quat_B_df=quat_est_df,
    title='z vs x_prior free range',
    figname='fig_08',
    show=_show,
    colors=['maroon','darkgoldenrod'],
    save=True,
    output_dir=dset.output_dir,
    start=dset.start,
    end=dset.end,
    labels=['meas.', 'est.'])

  # plot EKF output
  fignum+=1; get_fignum_str(fignum)
  plot_quat_vs_quat(quat_A_df=quat_meas_df,
    quat_B_df=quat_est_df,
    title='z vs x_prior unit range',
    figname='fig_09',
    show=_show,
    #show=True,
    colors=['maroon','darkgoldenrod'],
    save=True,
    output_dir=dset.output_dir,
    start=dset.start,
    end=dset.end,
    labels=['meas.', 'est.'],
    #y_range=[-1.1,1.1],
    )

  residual_df = pd.DataFrame(qekf.log.v_hist,
    index=qekf.log.idx,
    columns=['Tx', 'Ty', 'Tz',\
             'vx', 'vy', 'vz',\
             'wr', 'wp', 'wy',\
             'qx', 'qy', 'qz'])


  fignum+=1; get_fignum_str(fignum)
  plot_df(df=residual_df,
    rows=12,
    cols=1,
    title='v residual',
    show=_show,
    #show=True,
    figname='fig_10',
    output_dir=dset.output_dir)

  # z_TVWQxyzw
  z_df = pd.DataFrame(qekf.log.z_hist,
    index=qekf.log.idx,\
    columns=['Tx', 'Ty', 'Tz',\
             'vx', 'vy', 'vz',\
             'wr', 'wp', 'wy',\
             'qx', 'qy', 'qz', 'qw'])

  x_prior_df = pd.DataFrame(qekf.log.x_prior_hist,
    index=qekf.log.idx,\
    columns=['Tx', 'Ty', 'Tz',\
             'vx', 'vy', 'vz',\
             'qw', 'qx', 'qy', 'qz'])

  x_post_df = pd.DataFrame(qekf.log.x_post_hist,
    index=qekf.log.idx,\
    columns=['Tx', 'Ty', 'Tz',\
             'vx', 'vy', 'vz',\
             'qx', 'qy', 'qz'])

  z_Txyz_df = pd.DataFrame(qekf.log.z_hist[:,0:3],\
    index=qekf.log.idx,
    columns=['Tx', 'Ty', 'Tz'])

  x_post_Txyz_df = pd.DataFrame(qekf.log.x_post_hist[:,0:3],\
    index=qekf.log.idx,
    columns=['Tx', 'Ty', 'Tz'])

  fignum+=1; get_fignum_str(fignum)
  plot_Txyz_vs_Txyz_3d(z_Txyz_df,
    x_post_Txyz_df,
    title='z vs x_posterior translation',
    figname='fig_11',
    show=_show,
    #show=True,
    labels=['z', 'x_post'],
    output_dir=dset.output_dir)

  fignum+=1; get_fignum_str(fignum)
  plot_Txyz_vs_Txyz_3d(z_Txyz_df,
    x_post_Txyz_df,
    title='z vs x_posterior translation zoom',
    figname='fig_12',
    show=_show,
    #show=True,
    labels=['z', 'x_post'],
    end=_zoom,
    output_dir=dset.output_dir)

  # z_TVWQxyz
  z_TVQxyz_df = pd.DataFrame(qekf.log.z_hist[:,[0,1,2,3,4,5,9,10,11]],\
    index=qekf.log.idx,
    columns=['Tx', 'Ty', 'Tz',\
             'vx', 'vy', 'vz',\
  #           'wr', 'wp', 'wy',\
             'qx', 'qy', 'qz'])

  x_post_TVQxyz_df = pd.DataFrame(qekf.log.x_post_hist,\
    index=qekf.log.idx,
    columns=['Tx', 'Ty', 'Tz',\
             'vx', 'vy', 'vz',\
  #           'wr', 'wp', 'wy',\
             'qx', 'qy', 'qz'])


  fignum+=1; get_fignum_str(fignum)
  plot_z_df_vs_x_df_iso(z_TVQxyz_df,
    x_post_TVQxyz_df,
    labels=['z', 'x_post'],
    title='z meas vs x_posterior est - K_scalar '+str(qekf.K_scale),
    figname='fig_13',
    show=_show,
    #show=True,
    output_dir=dset.output_dir)

  #todo: still working on this routine
  #plot_z_df_vs_x_df_grp(z_TVQxyz_df,
  #  x_post_TVQxyz_df,
  #  labels=['z', 'x_post'],
  #  title='z meas vs x_posterior est - K_scalar '+str(qekf.K_scale),
  #  figname='fig_13',
  #  show=_show,
  #  output_dir=dset.output_dir)

  K_columns = ['K_%03d' % i for i in range(qekf.log.K_hist.shape[1])]


  K_df = pd.DataFrame(qekf.log.K_hist,
    index=qekf.log.idx,
    columns= K_columns)


  fignum+=1; get_fignum_str(fignum)
  plot_df_grp_K(df=K_df,
    title='Kalman Gain - K_scalar '+str(qekf.K_scale),
    figname='fig_14',
    #show=True,
    show=_show,
    output_dir=dset. output_dir)


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
  fignum+=1; get_fignum_str(fignum)
  plot_z_df_vs_x_df(x_q_df,
    x_prior_q_df,
    labels=['x_q', 'x_prior'],
    rows=4,cols=1,
    title='x_q vs x_prior Quaternions',
    figname='fig_13',
    show=_show,
    figsize=[5,10])
  '''

  ''' L2 loss '''
  #L2loss_df = get_L2loss(residual_df)

  # plot L2 loss
  #fignum+=1; get_fignum_str(fignum)
  #plot_df(L2loss_df)

  #todo: L2 norm


  '''NBUG'''
  st()

  '''---------   End of QEKF  -->>>>>>>>>>>>>>>>>'''

  return # end of main

if __name__ == "__main__":
  main()
  print('---------   End   ---------')
  st()
  time.sleep(1000)
  exit()
