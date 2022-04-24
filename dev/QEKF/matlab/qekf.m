function [qekf_obj, dlog] = qekf(dset, ...
                            dlog, ...
                              dim_x, ...
                              dim_z, ...
                              dim_u, ...
                              T_, ...
                              Q_T_xyz, ...
                              Q_V_xyz, ...
                              Q_quat_xyz, ...
                              R_noise, ...
                              P_est_0, ...
                              IC)
%QEKF Summary of this function goes here
%   Detailed explanation goes here

addpath(genpath('qekf_utils'));

qekf_obj = qekf_class(dim_x, ...
                      dim_z, ...
                      dim_u, ...
                      T_, ...
                      Q_T_xyz, ...
                      Q_V_xyz, ...
                      Q_quat_xyz, ...
                      R_noise, ...
                      P_est_0, ...
                      IC);


for i = dset.start:dset.end
  
  u_Wrpy = dset.u_Wrpy(i,:);

  
  z_TVQxyz  = dset.z_TVQxyzw(i,1:end-1); % copy all except w term 
  z_TVQxyzw = dset.z_TVQxyzw(i,:); % only for data logging
  x_TVQxyz  = qekf.predict(x_TVQxyz, u_Wrpy)
  x_TVQxyz  = qekf.update(x_TVQxyz, z_TVQxyz)
  
  qekf.log.log_z_state(z_TVQxyzw, i)
  print('end of qekf data iterator ----->>')
   
end

