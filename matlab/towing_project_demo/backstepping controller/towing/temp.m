
%% 拖船参数
M1 = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
Ma1 = [7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5, 8.77e6];
MM = M1 + Ma1;
D1 = [MM(1,1)/100, 0, 0; 0, MM(2,2)/40, 0; 0, 0, MM(3,3)/20];
clear MM;

%% pid 参数整定
wn = 0.05;
T = 2*pi/wn;
eta_D = 0.7; %临界阻尼系数
kp_x = wn^2 *(M1(1, 1)+Ma1(1, 1));
kp_y = wn^2 *(M1(2, 2)+Ma1(2, 2));
kp_psi = wn^2 *(M1(3, 3)+Ma1(3, 3));

kd_x = 1.4*sqrt((M1(1, 1)+Ma1(1, 1))*kp_x)-D1(1, 1);
kd_y = 1.4*sqrt((M1(2, 2)+Ma1(2, 2))*kp_y) - D1(2, 2);
kd_psi = 1.4*sqrt((M1(3, 3)+Ma1(3, 3))*kp_psi) - D1(3, 3);


ki_x = 0.01 * kp_x;
ki_y = 0.01 * kp_y;
ki_psi = 0.01 * kp_psi;

%无因次化
Kp = diag([kp_x/M1(1, 1), kp_y/M1(2, 2), kp_psi/M1(3, 3)]);
Kd = diag([kd_x/M1(1, 1), kd_y/M1(2, 2), kd_psi/M1(3, 3)]);
Ki = diag([ki_x/M1(1, 1), ki_y/M1(2, 2), ki_psi/M1(3, 3)]);

%% 推力分配程序测试
clear all; clc;
f0 = [100 100 100]';
a0 = [10 20 30]'
tau = [300, 480, -150]';
u0 = [1 1 1]';
n = 500;

f_his = zeros(3, n);
a_his = zeros(3, n);
tau_r_his = zeros(3, n);


for i = 1 : n
    [f,df,a,da,tau_r,dtau] = Thrust_Allocation(f0,a0,tau);
    tau_r;
    f_his(:, i) = f;
    a_his(:, i) = a;
    tau_r_his(:, i) = tau_r;
    f0 = f;
    a0 = a;
end

plot(1:n, tau_r_his)
legend('tau_x', 'tau_y', 'tau_m')



%% 求解racciti方程

A = [-2, -1; 2, 1];
B = [1, 0]';
Q = eye(2);
R = 1;
N = [];
[K, S, e] = lqr(A, B, Q, R, N)
diag([0.0028, 0.004, 0.0031])*0.01;




%% 定义状态反馈方程的系数，求解raccati方程
wn = 0.05; 
cd = 0.7;

A1 = -2 * cd * wn * eye(3);
A0 = -wn^2 * eye(3);
B1 = -A0;

A = [zeros(3, 3), eye(3); A0, A1];
B = [zeros(3, 3); B1];

Q = 5*eye(6);
R = eye(3);
N = [];
[K, S, e] = lqr(A, B, Q, R, N);
K



%% 测试一致性控制
% function [pd1, pd2] = consensus_guidance_controller(p0, v0, p1, v1, p2, v2)

p1 = [397, 237, 31]';
p2 = [397, -237, -31]';
p0 = [1, 0, 0]';
v0 = zeros(3, 1);
v1 = zeros(3, 1);
v2 = zeros(3, 1);
[pd1, pd2] = consensus_guidance_controller(p0, v0, p1, v1, p2, v2);



%%  角度转化成（-pi, pi)

% x = 4 * pi/3;
% y = transfer_deg(x) / pi * 180
% 
% 
% function y = transfer_deg(x)
%     y = -sign(x) * pi + rem((x + sign(x) * pi), 2 * pi);
% end
% 

% function y = transter_deg(x)
%     y = -sign(x) * 180 + rem((x +



%% BS controller
clear all; clc;
global ei M1 Ma1 MM1 D1 integ_var ;
M1 = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
Ma1 = [7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5, 8.77e6];
MM1 = M1 + Ma1;
D1 = [MM1(1,1)/100, 0, 0; 0, MM1(2,2)/40, 0; 0, 0, MM1(3,3)/20];
ei = zeros(3,1);%初始化
integ_var = zeros(3, 1);%积分初值，初始化
%实时运动
x1 = [-16, 12, 16, 12, -16, -16];
y1 = [6, 6, 0, -6, -6, 6];
figure(5)
clf;
title('Motion of the vessel');
% figure('Name','Motion of the vessel','units','normalized','position',...
%            [0.6,0.2,0.4,0.4],'color',[0.8 1 0.8]);
axis([-50, 50, -50, 50]);
hgtrans1 = hgtransform;
fill(x1,y1,'m', 'parent',hgtrans1);
line(0,0,'marker','o','markeredgecolor','k','markerfacecolor','k','parent',hgtrans1);

accel = 0.001;
max_step = 1000;
p = zeros(3, 1); p_his = zeros(3, max_step); pd_his = zeros(3, max_step);
v = zeros(3, 1); v_his = zeros(3, max_step); vd_his = zeros(3, max_step);
tau_his = p_his;  f_his = p_his; taur_his = p_his;  a_his = f_his;
f0 = [100;100;100];
a0 = [-50;60;-100];
makeh1 = makehgtform('translate',[p(1) p(2) 0]) * makehgtform('zrotate',p(3));
set(hgtrans1, 'Matrix',makeh1);

T = 100; %时间常数 ref model
R = [10, 20, pi/4]';
p0 = p;

for i = 1 : max_step
%     pd = [accel * i * i / 2, 0, 0]';
%     dot_pd = [accel * i, 0, 0]';
%     ddot_pd = [accel, 0, 0]';
% 
% 
%     pd = [1, 1, -pi/4]';
%     dot_pd = zeros(3, 1);
%     ddot_pd = zeros(3,1);
    t = i;
    [pd, dot_pd, ddot_pd] = ref_model(T, t, p0, R);

    
    
    p_his(:, i) = p;
    v_his(:, i) = v;
    pd_his(:, i) = pd;
    vd_his(:, i) = dot_pd;
    
    
    
    
    [tau, s] = bs_constroller(pd, dot_pd, ddot_pd, p, v);
    
    [f,df,a,da,tau_r,dtau] = Thrust_Allocation(f0,a0,tau);
    
    [p_next, v_next] = ship_dynamic_solver(p, v, tau_r*1e3);
    p = p_next;
    v = v_next;
    f0=f;
    a0=a;
    tau_his(:, i) = tau;
    taur_his(:, i) = tau_r;
    f_his(:, i) = f;
    a_his(:, i) = a;
    
    makeh1 = makehgtform('translate',[p(1) p(2) 0]) * makehgtform('zrotate',p(3));
    set(hgtrans1, 'Matrix',makeh1);
    pause(0.001);

    
    
end

figure_plot

% function [pd, dot_pd, ddot_pd] = tracjectory_generate(t, 

%% pid controller

clear all; clc;
global ei M1 Ma1 MM1 D1 integ_var ;
M1 = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
Ma1 = [7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5, 8.77e6];
MM1 = M1 + Ma1;
D1 = [MM1(1,1)/100, 0, 0; 0, MM1(2,2)/40, 0; 0, 0, MM1(3,3)/20];
ei = zeros(3,1);%初始化
integ_var = zeros(3, 1);%积分初值，初始化
accel = 0.01;
max_step = 1000;
p = zeros(3, 1); p_his = zeros(3, max_step); pd_his = zeros(3, max_step);
v = zeros(3, 1); v_his = zeros(3, max_step); vd_his = zeros(3, max_step);
tau_his = p_his;  f_his = p_his; taur_his = p_his;  a_his = f_his;
f0 = [100;100;100];
a0 = [-50;60;-100];



for i = 1 : max_step
%     pd = [accel * i * i / 2, 0, 0]';
%     dot_pd = [0, 0, 0]';
%     ddot_pd = [0, 0, 0]';


    pd = [1, 1, pi/6]';
    dot_pd = [0, 0, 0]';
    ddot_pd = [0, 0, 0]';
    
    p_his(:, i) = p;
    v_his(:, i) = v;
    pd_his(:, i) = pd;
    vd_his(:, i) = dot_pd;
    
    
%     pd = [1, 1, 45/pi]';
%     dot_pd = zeros(3, 1);
%     ddot_pd = zeros(3,1);
    tau=pid_controller(pd, dot_pd, p, v);
%     [tau, s] = bs_constroller(pd, dot_pd, ddot_pd, p, v);
    
    [f,df,a,da,tau_r,dtau] = Thrust_Allocation(f0,a0,tau);
    
    [p_next, v_next] = ship_dynamic_solver(p, v, tau*1e3);
    p = p_next;
    v = v_next;
    f0=f;
    a0=a;
    tau_his(:, i) = tau;
    taur_his(:, i) = tau_r;
    f_his(:, i) = f;
    
    a_his(:, i) = a;
    

    
    
end

figure_plot

% function [pd, dot_pd, ddot_pd] = tracjectory_generate(t, 



%% ship_dynamic_solver
clc;
p = zeros(3, 1); p_his = p;
v = zeros(3, 1); v_his = v;
tau = [1e5, 0, 0]';
[p_next, v_next] = ship_dynamic_solver(p, v, tau)


%% 船舶位置动态图


p0 = [0, 0, 0]';

x1 = [-16, 12, 16, 12, -16, -16];
y1 = [6, 6, 0, -6, -6, 6];
close(findobj('type','figure','name','Motion of the vessels'))
figure('Name','Motion of the vessel','units','normalized','position',...
           [0.6,0.2,0.4,0.4],'color',[0.8 1 0.8]);
axis([-50, 50, -50, 50]);
hgtrans1 = hgtransform;
fill(x1,y1,'k', 'parent',hgtrans1);
line(0,0,'marker','o','markeredgecolor','k','markerfacecolor','k','parent',hgtrans1);   
M1 = makehgtform('translate',[p0(1) p0(2) 0]) * makehgtform('zrotate',p0(3));
set(hgtrans1, 'Matrix',M1);
    


for i = 1 : length(p_his)
    p0 = p_his(:, i);
    M1 = makehgtform('translate',[p0(1) p0(2) 0]) * makehgtform('zrotate',p0(3));
    set(hgtrans1, 'Matrix',M1);
    pause(0.01);
    
end
    

%% first reference model 
clear all; clc;

max_step = 10;
r = [1, 1.2, 160]';
T = 5*0.66;
p0 = [1, 1, -170]';

global fai_old;
fai_old = p0(3);
pd_his = zeros(3, max_step);  
dot_pd_his = pd_his;
ddot_pd_his = pd_his;
for i = 1 : max_step
    t = i;
    [pd, dot_pd, ddot_pd] = ref_model(T, i, p0, r);

    pd_his(:, i) = pd;
    dot_pd_his(:, i) = dot_pd;
    ddot_pd_his(:, i) = ddot_pd;
end

figure
plot(pd_his(1, :));
hold on 
plot(pd_his(2, :));
hold on 
plot(pd_his(3, :));
hold off
legend('pdx', 'pdy', 'pdfai');
legend boxoff

const_pi = pi/180;
fai = atan2(sin(-330*const_pi), cos(-330*const_pi))/const_pi
% figure
% plot(dot_pd_his(1, :));
% hold on 
% plot(dot_pd_his(2, :));
% hold on 
% plot(dot_pd_his(3, :));
% hold off
% legend('dot_pdx', 'dot_pdy', 'dot_pdfai');
% legend boxoff
% figure
% plot(dot_pd_his);



%% 坐标系转换，编队的队形控制
clear all; clc; clf;
delta10 = [-10, 10, pi/36]'*2;
delta20 = [-10, -10, -pi/36]'*2;
p0 = [0, 0, 0]';
p1 = p0 + delta10;
p2 = p0 + delta20;

hull_x1 = [-16, 12, 16, 12, -16, -16];
hull_y1 = [6, 6, 0, -6, -6, 6];
figure(1)
title('Motion of the ships');
axis([-50, 50, -50, 50]*2);
hgtrans0 = hgtransform;
fill(hull_x1,hull_y1,'y', 'parent',hgtrans0);
line(0,0,'marker','o','markeredgecolor','k','markerfacecolor','k','parent',hgtrans0);   
text(p0(1)+5, p0(2)+3,'0', 'parent', hgtrans0)
makehg0 = makehgtform('translate',[p0(1) p0(2) 0]) * makehgtform('zrotate',p0(3));
set(hgtrans0, 'Matrix',makehg0);

hgtrans1 = hgtransform;
fill(hull_x1,hull_y1, 'm', 'parent',hgtrans1);
line(0,0,'marker','o','markeredgecolor','k','markerfacecolor','k','parent',hgtrans1); 
text(p1(1)+25, p1(2)-20,'1', 'parent', hgtrans1)
makehg1 = makehgtform('translate',[p1(1) p1(2) 0]) * makehgtform('zrotate',p1(3));
set(hgtrans1, 'Matrix',makehg1);

hgtrans2 = hgtransform;
fill(hull_x1, hull_y1, 'm', 'parent',hgtrans2);
line(0,0,'marker','o','markeredgecolor','k','markerfacecolor','k','parent',hgtrans2); 
text(p2(1)+25, p2(2)+20, '2', 'parent', hgtrans2)
makehg2 = makehgtform('translate',[p2(1) p2(2) 0]) * makehgtform('zrotate',p2(3));
set(hgtrans2, 'Matrix',makehg2);
    
max_step = 1000;
r = [20, 33, pi/3]'*2;
T = 50;

p00 = p0;

for i = 1 : max_step
    
    t = i;
    [pd, dot_pd, ddot_pd] = ref_model(T, i, p00, r);
    p0 = pd;
    p1 = BtoG(p0, delta10);
    p2 = BtoG(p0, delta20);
    makehg0 = makehgtform('translate',[p0(1) p0(2) 0]) * makehgtform('zrotate',p0(3));
    set(hgtrans0, 'Matrix',makehg0);
    makehg1 = makehgtform('translate',[p1(1) p1(2) 0]) * makehgtform('zrotate',p1(3));
    set(hgtrans1, 'Matrix',makehg1);
    makehg2 = makehgtform('translate',[p2(1) p2(2) 0]) * makehgtform('zrotate',p2(3));
    set(hgtrans2, 'Matrix',makehg2);
    hold on
    plot(p0(1), p0(2), '.')
    pause(0.01);
    
end


function p = BtoG(p0, delta)
    R = rotate_matrix(p0);
    p = R * delta + p0;
end


function R= rotate_matrix(p)
    fai = p(3);
    R = [cos(fai), -sin(fai), 0;
        sin(fai), cos(fai), 0;
        0, 0, 1];
end

%% 测试二阶参考模型

