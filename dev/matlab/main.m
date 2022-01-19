plt_range_x = [10000 10100]
data = importdata('/home/smerx/git/QEKF/dev/data/dead_reckoning_data/1/imu/rv.txt') ;  
t = data(:,1); 
t = t/1000000;
t = round(t);
t = t-t(1);
x = data(:,2);
y = data(:,3);
z = data(:,4);
w = data(:,5);
tq_len = size(t);
tq = 0:tq_len(1);

xq = interp1(t,x,tq,'spline');
yq = interp1(t,y,tq,'spline');
zq = interp1(t,z,tq,'spline');
wq = interp1(t,w,tq,'spline');


figure
% plot(t,x,'o',tq,xq,':.');
plot(tq,xq,':.');
%xlim([10000 10100]);
title('rv: Qx');
legend('xq');

figure
% plot(t,y,'o',tq,yq,':.');
plot(tq,yq,':.');
%xlim([10000 10100]);
title('rv: Qy');
legend('yq');

figure
%plot(t,z,'o',tq,zq,':.');
plot(tq,zq,':.');
%xlim([10000 10100]);
title('rv: Qz');
legend('zq');

figure
% plot(t,w,'o',tq,wq,':.');
plot(tq,wq,':.');
%xlim([10000 10100]);
title('rv: Qw');
legend('wq');
