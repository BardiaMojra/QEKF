plt_range_x = [10000 10100]
data_rv = importdata('/home/smerx/git/QEKF/dev/data/dead_reckoning_data/1/imu/rv.txt') ;  
data = importdata('/home/smerx/git/QEKF/dev/data/dead_reckoning_data/1/imu/gyro.txt') ;  
t_rv = data_rv(:,1); 
t_rv = t_rv/1000000;
t_rv = round(t_rv);
t_rv = t_rv-t_rv(1);
t = data(:,1);
x = data(:,2);
y = data(:,3);
z = data(:,4);
tq_len = size(t_rv);
tq = 0:tq_len(1);

xq = resample(x,t_rv);
yq = resample(y,t_rv);
zq = resample(z,t_rv);


figure
plot(t,x,'o',tq,xq,':.');
%plot(tq,xq,':.');
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


