%¹â»¬Â·¾¶
clear all;close all;
%load r_new4.mat
load r_planning6.mat
r_new=r_planning6;
x_smoothed=linspace(min(r_new(1,:)),max(r_new(1,:)),200);
y_smoothed=interp1(r_new(1,:),r_new(2,:),x_smoothed,'pchip');
z_smoothed=interp1(r_new(1,:),r_new(3,:),x_smoothed,'pchip');
r_new6=[x_smoothed;y_smoothed;z_smoothed];

figure(1)
plot(r_new(1,:),r_new(2,:),x_smoothed,y_smoothed)
legend('ori','smoothed')
figure(2)
plot(r_new(1,:),r_new(3,:),x_smoothed,z_smoothed)
legend('ori','smoothed')


%values = spcrv([[a(1) a a(end)];[b(1) b b(end)]],3);