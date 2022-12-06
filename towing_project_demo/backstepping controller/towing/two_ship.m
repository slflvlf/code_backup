clear all; clc; clf;
%% 初始化变量
M1 = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
Ma1 = [7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5, 8.77e6];
MM1 = M1 + Ma1;
D1 = [MM1(1,1)/100, 0, 0; 0, MM1(2,2)/40, 0; 0, 0, MM1(3,3)/20];
ei1 = zeros(3,1);%初始化
ei2 = zeros(3,1);%初始化
integ_var = zeros(3, 1);%积分初值，初始化


max_step = 1000;

% 信息记录
p00 = zeros(3, 1);
p0_his = zeros(3, max_step);
p1 = zeros(3, 1); p1_his = zeros(3, max_step); pd1_his = zeros(3, max_step);
v1 = zeros(3, 1); v1_his = zeros(3, max_step); vd1_his = zeros(3, max_step);
tau1_his = p1_his;  f1_his = p1_his; a1_his = f1_his; taur1_his = p1_his;
s1_his = p1_his;  tauc1_his = p1_his;

p2 = zeros(3, 1); p2_his = zeros(3, max_step); pd2_his = zeros(3, max_step);
v2 = zeros(3, 1); v2_his = zeros(3, max_step); vd2_his = zeros(3, max_step);
tau2_his = p2_his;  f2_his = p2_his; a2_his = f2_his; taur2_his = p2_his;
s2_his = p2_his;    tauc2_his = p2_his;

length_his = zeros(3, max_step);

delta10 = [-10, 10, 0]'*2;
delta20 = [-10, -10, 0]'*2;
p0 = [0, 0, 0]';
p1 = p0 + delta10;
p2 = p0 + delta20;

% 推力分配中，螺旋桨初始推力和转角
f1 = [100;100;100];
a1 = [-50;60;-100];
f2 = [100;100;100];
a2 = [-50;60;-100];

Kd = diag([20, 20, 5])*10;
Ki = diag([1, 1, 1])*1;

Kc1 = diag([20, 20, 5])*1;
Kc2 = diag([20, 20, 5])*1;



%% 主循环
%reference model的时间常数
% 表示一个时间常数会达到目标值的63%
T = 100; 
p10 = p1;
R0 = [20, 30, pi/4]';

for i = 1 : max_step
    [length01, length02, length12] = distance_among_ships(p0, p1, p2);
    
    length_his(:, i) = [length01-20*sqrt(2), length02-20*sqrt(2), length12-40]';
    
    % reference model
    t = i;
    [pd0, dot_pd0, ddot_pd0] = ref_model(T, t, p00, R0);
    pd1 = BtoG(p0, delta10);
    pd2 = BtoG(p0, delta20);
    dot_pd1 = dot_pd0;
    dot_pd2 = dot_pd0;
    
    ddot_pd1 = zeros(3, 1); ddot_pd2 = zeros(3, 1);
    
    p0 = pd0;
    p0_his(:, i) = pd0;

    
    p1_his(:, i) = p1;
    v1_his(:, i) = v1;
    pd1_his(:, i) = pd1;
    vd1_his(:, i) = dot_pd1;
    
    p2_his(:, i) = p2;
    v2_his(:, i) = v2;
    pd2_his(:, i) = pd2;
    vd2_his(:, i) = dot_pd2;
    
    
    [tau1, s1, ei1] = bs_constroller(pd1, dot_pd1, ddot_pd1, p1, v1, MM1, D1, Kd , Ki, ei1);
    [tau2, s2, ei2] = bs_constroller(pd2, dot_pd2, ddot_pd2, p2, v2, MM1, D1, Kd , Ki, ei2);
    
    [tauc1, tauc2] = coop_controller(s1, s2, Kc1, Kc2);
    % 加入一致性协议
    tau1 = tau1 + tauc1;    tau2 = tau2 + tauc2;
    
    [f1,df1,a1,da1,taur1,dtau1] = Thrust_Allocation(f1,a1,tau1);
    [f2,df2,a2,da2,taur2,dtau2] = Thrust_Allocation(f2,a2,tau2);
    
    [p1, v1] = ship_dynamic_solver(p1, v1, taur1, MM1, D1);
    [p2, v2] = ship_dynamic_solver(p2, v2, taur2, MM1, D1); 
    
    tau1_his(:, i) = tau1;
    taur1_his(:, i) = taur1;
    tauc1_his(:, i) = tauc1;
    f1_his(:, i) = f1;
    a1_his(:, i) = a1;
    s1_his(:, i) = s1;
    
    tau2_his(:, i) = tau2;
    taur2_his(:, i) = taur2;
    tauc2_his(:, i) = tauc2;
    f2_his(:, i) = f2;
    a2_his(:, i) = a2;
    s2_his(:, i) = s2;
    
    display_motion_flag = 0;
    figure_plot2;


    
end







function [length01, length02, length12] = distance_among_ships(p0, p1, p2)

    length01 = sqrt( (p0(1)-p1(1))^2 + (p0(2)-p1(2))^2 );
    length02 = sqrt( (p0(1)-p2(1))^2 + (p0(2)-p2(2))^2 );
    length12 = sqrt( (p1(1)-p2(1))^2 + (p1(2)-p2(2))^2 );

end





