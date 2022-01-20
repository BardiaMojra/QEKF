%% Dead Reckoning Project - gyro interpolation and resampling 
% @author Bardia Mojra
% @date 01/20/2022
% @title  Dead Reckoning Project - gyro interpolation and resampling 

% config
rv_fname = '/home/smerx/git/QEKF/dev/data/dead_reckoning_data/1/imu/rv.txt';
gyro_fname = '/home/smerx/git/QEKF/dev/data/dead_reckoning_data/1/imu/gyro.txt';
gyro_o_fname = '/home/smerx/git/QEKF/dev/data/dead_reckoning_data/1/imu/gyro_resamp.txt';

% low res data 
data_rv = importdata(rv_fname);    
tq = data_rv(:,1); 

% high res data
data = importdata(gyro_fname);
t_h = data(:,1);
x = data(:,2);
y = data(:,3);
z = data(:,4);


%xq = resample(x,t_rv);
%yq = resample(y,t_rv);
%zq = resample(z,t_rv);

xq = interp1(t_h,x,tq);
yq = interp1(t_h,y,tq);
zq = interp1(t_h,z,tq);

figure
hold on 
plot(t_h,x,'o',tq,xq,':.');
%plot(tq,xq,':.');
%xlim([10000 10050]);
title('interpolation review - gyro: x');
legend('x', 'xq');

figure
%plot(t_h,x,'o',tq,xq,':.');
plot(tq,xq,':.');
%xlim([10000 10050]);
title('gyro: x');
legend('xq');

figure
% plot(t,y,'o',tq,yq,':.');
plot(tq,yq,':.');
%xlim([10000 10100]);
title('gyro: y');
legend('yq');

figure
%plot(t,z,'o',tq,zq,':.');
plot(tq,zq,':.');
%xlim([10000 10100]);
title('gyro: z');
legend('zq');

% save new dataset as .txt file 
% create a table [t, x, y, z]
new_data = table(tq,xq,yq,zq);
writetable(new_data, gyro_o_fname);



