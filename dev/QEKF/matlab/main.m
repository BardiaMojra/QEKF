% QEKF matlab implimentation
% 
% 1  'dataset-iphone1_clean'
% 2  'bigC_06-Aug2021'
% 3  'Y2021M08D05_ZoomTwistJackal_BigC-off_ransac-off'
% 4  'Y2021M08D05_BoxWalkKuka_BigC-off_ransac-off_Q-Select-on_FP-Last6'
% 5  'Y2021M08D06_BoxWalkKuka_BigC-off_ransac-off_Q-Select-off_FP-HighLow6'
% 6  'Y2021M08D05_CircleAoundMetal_BigC-off_ransac-off'

% general config
NBUG          = True;
prt_output    = True;
show_         = True;
save_         = True;
prt_          = True;
START_        = 0;
END_          = 150;
% TEST_MODE_    = 'all'; TEST_ID_ = 'None';
TEST_ID_      = 1; TEST_MODE_ = 'single';

% module config
% warning('off','all'); % Ignore warnings for Matlab version compatibility


% init dataset object
dset = dmm(datName);

% plot data

% init and run qekf
qekf_obj = qekf(dset, % dataset
                9, % dim_x Txyz, Vxyz, Qxyz - linPos, linVel, rotVec (quat)
                9, % dim_z Txyz, Vxyz, Qxyz - linPos, linVel, rotVec (quat)
                6, % dim_u  Axyz, Wrpy
                dset.data_rate_inv, % data rate
                1.0e-5, % Q_T_xyz - process noise covar
                1.5e-2, % Q_V_xyz
                0.5e-3, % Q_quat_xyz
                1e-6, % R_noise - measurement noise covar
                1e-4, % P_est_0
                dset.z_TVQxyzw(1,:),
                1.0); % k-scale

  x_TVQxyz = qekf.x_TVQxyz # init state vectors #todo add IC from dataset
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

