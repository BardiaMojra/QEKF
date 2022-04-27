% QEKF matlab implimentation
% 
% 1  'dataset-iphone1_clean'
% 2  'bigC_06-Aug2021'
% 3  'Y2021M08D05_ZoomTwistJackal_BigC-off_ransac-off'
% 4  'Y2021M08D05_BoxWalkKuka_BigC-off_ransac-off_Q-Select-on_FP-Last6'
% 5  'Y2021M08D06_BoxWalkKuka_BigC-off_ransac-off_Q-Select-off_FP-HighLow6'
% 6  'Y2021M08D05_CircleAoundMetal_BigC-off_ransac-off'

% select dataset 
dataset = 'dataset-iphone1_clean';
% dataset = 'bigC_06-Aug2021';
% dataset = 'Y2021M08D05_ZoomTwistJackal_BigC-off_ransac-off';
% dataset = 'Y2021M08D05_BoxWalkKuka_BigC-off_ransac-off_Q-Select-on_FP-Last6';
% dataset = 'Y2021M08D06_BoxWalkKuka_BigC-off_ransac-off_Q-Select-off_FP-HighLow6';
% dataset = 'Y2021M08D05_CircleAoundMetal_BigC-off_ransac-off';


% general config
% NBUG          = true;
% prt_output    = true;
% show_         = true;
% save_         = true;
% prt_          = true;
% TEST_MODE_    = 'all'; TEST_ID_ = 'None'; %todo impliment feature
% TEST_ID_      = 1; TEST_MODE_ = 'single'; %todo impliment feature

% module config
% warning('off','all'); % Ignore warnings for Matlab version compatibility


% add depenendencies
addpath(genpath('./'));

% init dataset object
dset = dat_class;
dset = dset.load(dataset);

dlog = log_class;

% plot data

% init and run qekf
qekf = qekf_class;
qekf = qekf.config(dset, ... % dataset object 
                   dlog, ... % datalog object 
                   9, ... % dim_x Txyz, Vxyz, Qxyz 
                   9, ... % dim_z Txyz, Vxyz, Qxyz 
                   6, ... % dim_u Axyz, Wrpy
                   1.0e-5, ... % Q_T_xyz - process noise covar
                   1.5e-2, ... % Q_V_xyz
                   0.5e-3, ... % Q_quat_xyz
                   1e-6, ... % R_noise - measurement noise covar
                   1e-4 ... % P_est_0
                   ); %

x_TVQxyz = qekf.x_TVQxyz; % load prior state
for i = dset.START_:dset.END_
  u_Wrpy            = dset.Wrpy(i,:);
  z_TVQxyz          = dset.z_TVQxyzw(i,1:end-1); % copy all except w term 
  z_TVQxyzw         = dset.z_TVQxyzw(i,:); % only for data logging
  [qekf, x_TVQxyz]  = qekf.predict(x_TVQxyz, u_Wrpy);
  [qekf, x_TVQxyz]  = qekf.update(x_TVQxyz, z_TVQxyz);
  dlog              = dlog.log_meas(z_TVQxyzw, i);
end
print('end of qekf data iterator ----->>')

