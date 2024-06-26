%% Dead Reckoning Project - gyro interpolation and resampling 
% @author Bardia Mojra
% @date 01/20/2022
% @title  Dead Reckoning Project - gyro interpolation and resampling 

clear
close all


% config
% testDir = 'dead_reckoning_data/1/';
testDir = 'test_001_vicon_training_day/';
dataDir = '/home/smerx/git/QEKF/dev/data/';

rv_f    = strcat(dataDir, testDir, 'imu/rv.txt');
gy_f    = strcat(dataDir, testDir, 'imu/gyro.txt');
gy_n_f  = strcat(dataDir, testDir, 'imu/gyro_resamp.txt');
vi_f    = strcat(dataDir, testDir, 'vicon/vi_w_timestamps.txt');
vi_n_f  = strcat(dataDir, testDir, 'vicon/vi_resamp.txt');

% low res data 
data_rv = importdata(rv_f);    
t = data_rv(:,1); 

% high res data
data = importdata(gy_f);
t_h = data(:,1);
x_h = data(:,2);
y_h = data(:,3);
z_h = data(:,4);

x_hq = interp1(t_h,x_h,t);
y_hq = interp1(t_h,y_h,t);
z_hq = interp1(t_h,z_h,t);


% high res data - vicon
vi_data = importdata(vi_f);
t_vi = vi_data(:,1);
qx_vi  = vi_data(:,2);
qy_vi  = vi_data(:,3);
qz_vi  = vi_data(:,4);
qw_vi  = vi_data(:,5);
x_vi   = vi_data(:,6);
y_vi   = vi_data(:,7);
z_vi   = vi_data(:,8);


qx_viq = interp1(t_vi,qx_vi,t);
qy_viq = interp1(t_vi,qy_vi,t);
qz_viq = interp1(t_vi,qz_vi,t);
qw_viq = interp1(t_vi,qw_vi,t);
x_viq = interp1(t_vi,x_vi,t);
y_viq = interp1(t_vi,y_vi,t);
z_viq = interp1(t_vi,z_vi,t);


% figure
% plot(t_h,x_h,'o',t,x_hq,':.');
% title('interpolation review - gyro: x');
% legend('x', 'xq');

% figure
% plot(t,x_hq,':.');
% title('gyro: x');
% legend('xq');
% 
% figure
% plot(t,y_hq,':.');
% title('gyro: y');
% legend('yq');
% 
% figure
% plot(t,z_hq,':.');
% title('gyro: z');
% legend('zq');


figure
plot(t,qx_viq,':.');
title('vi: qx');
legend('qx');

figure
plot(t,qy_viq,':.');
title('vi: qy');
legend('qy');

figure
plot(t,qz_viq,':.');
title('vi: qz');
legend('qz');

figure
plot(t,qw_viq,':.');
title('vi: qw');
legend('qw');


figure
plot(t,x_viq,':.');
title('vi: x');
legend('x');

figure
plot(t,y_viq,':.');
title('vi: y');
legend('y');

figure
plot(t,z_viq,':.');
title('z');
legend('vi: z');


% save new dataset as .txt file 
% create a table [t, x, y, z]
new_data = table(t,x_hq,y_hq,z_hq);
writetable(new_data, gy_n_f);

% save vicon data 
new_data = table(t,qx_viq,qy_viq,qz_viq,qw_viq,x_viq,y_viq,z_viq);
writetable(new_data, vi_n_f);



