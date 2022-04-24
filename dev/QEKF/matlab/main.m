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
addpath(genpath('utils'));

% init dataset object
% dset = dat_class;
dset = dat_class.byname(dataset);

% plot data

% init and run qekf
qekf_obj, qekf_log = qekf(dset, ... % dataset object 
                          dlog, ... % datalog object 
                          9, ... % dim_x Txyz, Vxyz, Qxyz 
                          9, ... % dim_z Txyz, Vxyz, Qxyz 
                          6, ... % dim_u Axyz, Wrpy
                          dset.data_rate_inv, ... % data rate
                          1.0e-5, ... % Q_T_xyz - process noise covar
                          1.5e-2, ... % Q_V_xyz
                          0.5e-3, ... % Q_quat_xyz
                          1e-6, ... % R_noise - measurement noise covar
                          1e-4, ... % P_est_0
                          dset.z_TVQxyzw(1,:), ... % initial conditions 
                          1.0); % k-scale


print('end of qekf data iterator ----->>')

