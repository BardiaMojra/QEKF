% QEKF matlab implimentation
% 


% general config
NBUG          = True
prt_output    = True
show_         = True
save_         = True
prt_          = True
START_        = 0
END_          = 150
TEST_MODE_    = 'all'; TEST_ID_ = None
TEST_ID_      = 1; TEST_MODE_ = 'single'

% module config
% warning('off','all'); % Ignore warnings for Matlab version compatibility


% init dataset object
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
              IC=dset.z_TVQxyzw_np[0],
              K_scale=1.0)

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

