x = 0:pi/4:2*pi; 
v = sin(x);


xq = 0:pi/16:2*pi;

figure
vq1 = interp1(x,v,xq);
plot(x,v,'o',xq,vq1,':.');
xlim([0 2*pi]);
title('(Default) Linear Interpolation');
legend('v','vq');

figure
vq2 = interp1(x,v,xq,'spline');
plot(x,v,'o',xq,vq2,':.');
xlim([0 2*pi]);
title('Spline Interpolation');
legend('v','vq');
