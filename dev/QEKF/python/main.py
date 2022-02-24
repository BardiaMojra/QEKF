
# import numpy as np
import matplotlib
import pandas as pd
import time


''' private libraries
'''
from dmm import *
from util import *
from qekf import *


''' NBUG libraries
'''
from pdb import set_trace as st
from nbug import *

''' general config '''
NBUG          = True
prt_output    = True
_show         = True
_save         = True
_prt          = True
# _START        = 0
# _END          = 150
# _TEST_MODE    = 'all'; _TEST_ID = None
_TEST_ID      = 13; _TEST_MODE = 'single'


''' matplotlib config '''
# matplotlib.pyplot.ion()
# plt.style.use('ggplot')

''' datasets
 0  'dataset-iphone1_clean'
 1  'bigC_06-Aug2021'
 2  'kitti_imu_0926_0001'
 3  'kitti_imu_0926_0002'
 4  'kitti_imu_0926_0005'
 5  'kitti_imu_0926_0018'
 6  'kitti_imu_0926_0060'
 7  'kitti_imu_0926_0084'
 8  'kitti_imu_0926_0113'
 9  'kitti_imu_0928_0001'
10  'Y2021M08D05_ZoomTwistJackal_BigC-off_ransac-off'
11  'Y2021M08D05_BoxWalkKuka_BigC-off_ransac-off_Q-Select-on_FP-Last6'
12  'Y2021M08D06_BoxWalkKuka_BigC-off_ransac-off_Q-Select-off_FP-HighLow6'
13  'Y2021M08D05_CircleAoundMetal_BigC-off_ransac-off'
14
15
16
17
18
19

'''

def main():
  if sys.argv[1] is not None and isinstance(int(sys.argv[1]), int) is True:
    _TEST_ID = int(sys.argv[1])
    print(shorthead+'user input (int): '+str(_TEST_ID)+' ---> '+datasets[_TEST_ID])
    run(str(datasets[_TEST_ID]))
  elif _TEST_MODE == 'all':
    print(shorthead+'starting all test sequences:')
    for i in range(len(datasets)):
      print('> '+str(i)+': '+datasets[i])
    print('\n')
    for i in range(len(datasets)):
      print('\n\n\n'+longhead+'> '+str(i)+': '+datasets[i])
      run(datasets[i])
  elif isinstance(_TEST_ID, int) is True:
    if _TEST_ID not in range(len(datasets)):
      eprint('usr input (int) is out of range. datasets[] range: 0-'+str(len(datasets)-1))
      eprint('usr input: '+str(_TEST_ID))
      exit()
    else:
      print(shorthead+'user input (int): '+str(_TEST_ID)+' ---> '+datasets[_TEST_ID])
      run(str(datasets[_TEST_ID]))
  print(longhead+'---- end of main ----')
  return

''' local routines '''
def run(data:str):
  global fignum; fignum = int(0)

  # init dataset object
  dset = dmm(name=data,
             VestScale=1,
             data_rate_inv=1/10,
            #  start=_START,
            #  end=_END,
             prt=_prt)

  dset.format_data()

  fignum+=1;
  dset.plot(df=dset.df,
            labels=dset.df.columns,
            fignum=fignum,
            title=data,
            show=_show)

  fignum+=1;
  dset.plot_trans_3d(title='Measured Translation',
                     fignum=fignum,
                     show=_show)

  # init QEKF object
  qekf = QEKF(dim_x=9, # Txyz, Vxyz, Qxyz - linPos, linVel, rotVec (quat)
              dim_z=9, # Txyz, Vxyz, Qxyz - linPos, linVel, rotVec (quat)
              dim_u=6, # Axyz, Wrpy
              deltaT=dset.data_rate_inv,
              Q_T_xyz=1.0e-5, # process noise covar
              Q_V_xyz=1.5e-2,
              Q_quat_xyz=0.5e-3,
              R_noise=1e-6, # measurement noise covar
              P_est_0=1e-4,
              K_scale=1.0)

  x_TVQxyz = qekf.x_TVQxyz # init state vectors
  for i in range(dset.start, dset.end):
    ''' EKF state machine '''
    # print('    \\--->>> new state ------->>>>>:  ', i)
    u_Wrpy = dset.u_Wrpy_np[i].reshape(-1,1)
    z_TVQxyz = dset.z_TVQxyzw_np[i,:-1].reshape(-1,1)
    z_TVQxyzw = dset.z_TVQxyzw_np[i].reshape(-1,1) # only for data logging
    # nsprint('x_TVQxyz', x_TVQxyz)
    # nsprint('u_Wrpy', u_Wrpy)
    # nsprint('z_TVQxyz', z_TVQxyz)
    # st()
    x_TVQxyz = qekf.predict(x_TVQxyz, u_Wrpy)
    # nsprint('x_TVQxyz', x_TVQxyz)
    x_TVQxyz = qekf.update(x_TVQxyz, z_TVQxyz)
    # st()
    ''' log z state vector '''
    qekf.log.log_z_state(z_TVQxyzw, i)
  print('end of qekf data iterator ----->>')


  ''' post processing
  '''
  quat_meas_df = pd.DataFrame(qekf.log.z_hist[:,6:10],
                              index=qekf.log.idx,
                              columns=['qx', 'qy', 'qz', 'qw'])# change to xyzw
  # x_quat_wxyz
  quat_est_df = pd.DataFrame(qekf.log.x_hist[:,6:10],\
    index=qekf.log.idx,\
    columns=['qx', 'qy', 'qz', 'qw'])

  # plot EKF output
  fignum+=1;
  plot_quat_vs_quat(quat_A_df=quat_meas_df,
    quat_B_df=quat_est_df,
    title='z vs x_prior free range',
    fignum=fignum,
    show=_show,
    colors=['maroon','darkgoldenrod'],
    save=True,
    output_dir=dset.output_dir,
    start=dset.start,
    end=dset.end,
    labels=['meas.', 'est.'])

  # plot EKF output
  fignum+=1;
  plot_quat_vs_quat(quat_A_df=quat_meas_df,
    quat_B_df=quat_est_df,
    title='z vs x_prior unit range',
    fignum=fignum,
    show=_show,
    colors=['maroon','darkgoldenrod'],
    save=True,
    output_dir=dset.output_dir,
    start=dset.start,
    end=dset.end,
    labels=['meas.', 'est.'],
    y_range=[-1.1,1.1])

  residual_df = pd.DataFrame(qekf.log.y_hist,
    index=qekf.log.idx,
    columns=['Tx', 'Ty', 'Tz',\
             'vx', 'vy', 'vz',\
            #  'wr', 'wp', 'wy',\
             'qx', 'qy', 'qz'])

  residual_df = get_losses(residual_df, dset.output_dir)
  fignum+=1;
  plot_df(df=residual_df,
    rows=residual_df.shape[1],
    cols=1,
    title='v residual',
    show=_show,
    fignum=fignum,
    output_dir=dset.output_dir)

  # print losses
  print_losses(residual_df)


  # z_TVWQxyzw
  z_df = pd.DataFrame(qekf.log.z_hist,
    index=qekf.log.idx,\
    columns=['Tx', 'Ty', 'Tz',\
             'vx', 'vy', 'vz',\
             'qx', 'qy', 'qz', 'qw'])

  x_post_df = pd.DataFrame(qekf.log.x_hist,
    index=qekf.log.idx,\
    columns=['Tx', 'Ty', 'Tz',\
             'vx', 'vy', 'vz',\
             'qx', 'qy', 'qz', 'qw'])

  z_Txyz_df = pd.DataFrame(qekf.log.z_hist[:,0:3],\
    index=qekf.log.idx,
    columns=['Tx', 'Ty', 'Tz'])

  x_post_Txyz_df = pd.DataFrame(qekf.log.x_hist[:,0:3],\
    index=qekf.log.idx,
    columns=['Tx', 'Ty', 'Tz'])

  fignum+=1;
  plot_Txyz_vs_Txyz_3d(z_Txyz_df,
    x_post_Txyz_df,
    title='z vs x_post translation',
    fignum=fignum,
    show=_show,
    labels=['z', 'x_post'],
    output_dir=dset.output_dir)


  fignum+=1;
  plot_z_df_vs_x_df_iso(z_df,
    x_post_df,
    labels=['z', 'x_post'],
    title='z meas vs x_post est - K_scalar '+str(qekf.K_scale),
    fignum=fignum,
    show=_show,
    output_dir=dset.output_dir)

  '''---------   End of QEKF  -->>>>>>>>>>>>>>>>>'''
  return # end of main

if __name__ == "__main__":
  main()
  print('---------   End   ---------')
# EOF
