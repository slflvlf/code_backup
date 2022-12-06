clear all; clc; clf;

M1 = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
Ma1 = [7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5, 8.77e6];
MM1 = M1 + Ma1;
D1 = [MM1(1,1)/100, 0, 0; 0, MM1(2,2)/40, 0; 0, 0, MM1(3,3)/20];

% pid 控制器中的误差积分的初值
integral_error1 = zeros(3, 1);%积分初值，初始化
integral_error2 = zeros(3, 1);


% 运行的最大步长
max_step = 1000;

%变量初始化和历史记录
[p1_his, v1_his, pd1_his, vd1_his, p2_his, v2_his, pd2_his, vd2_his, ...
 tau1_his, taur1_his, f1_his, a1_his, tau2_his, taur2_his, f2_his, a2_his]...
 = deal(zeros(3, max_step));


% 在体坐标系下的相对间距，相当于一个刚体的不同位置 
delta10 = [-10, 10, 0]'*2;
delta20 = [-10, -10, 0]'*2;
delta12 = delta10 - delta20;
delta21 = -delta12;

p0 = [0, 0, 0]';
p1 = p0 + delta10;
p2 = p0 + delta20;

% 推力分配中，螺旋桨初始推力和转角
f1 = [100;100;100];
a1 = [-50;60;-100];
f2 = [100;100;100];
a2 = [-50;60;-100];


% Kd = diag([20, 20, 5])*10;
% Ki = diag([1, 1, 1])*1;

T = 100; %reference model的时间常数
p00 = p0;
R = [10, 15, pi/3]';

for i = 1 : max_step

    
    % reference model
    t = i;
    [pd0, dot_pd0, ddot_pd0] = ref_model(T, t, p00, R);
    
    
% %     set point mode
% %     pd = [1, 1, pi/3]';
%     pd = R;
%     dot_pd = [0, 0, 0]';
%     ddot_pd = [0, 0, 0]';
    
    p1_his(:, i) = p1;
%     v1_his(:, i) = v1;
    pd_his(:, i) = pd0;
    vd_his(:, i) = dot_pd0;
    
    
    % pid controller
    [tau1, integral_error1] = pid_controller(pd1, dot_pd1, p1, v1, MM1, D1, integral_error1);
    [tau2, integral_error2] = pid_controller(pd2, dot_pd2, p2, v2, MM1, D1, integral_error2);
    
%     % backstepping controller
%     [tau, s, ei] = bs_constroller(pd, dot_pd, ddot_pd, p, v, MM1, D1, Kd , Ki, ei);

    % 推力分配
    [f1,df1,a1,da1,taur1,dtau1] = Thrust_Allocation(f1, a1, tau1);
    [f2,df2,a2,da2,taur2,dtau2] = Thrust_Allocation(f2, a2, tau2);
    
    % 运动方程求解
    [p1, v1] = ship_dynamic_solver(p1, v1, taur1, MM1, D1);
    [p2, v2] = ship_dynamic_solver(p2, v2, taur2, MM1, D1);

    tau1_his(:, i) = tau1;  tau2_his(:, i) = tau2;
    taur1_his(:, i) = taur1;   taur2_his(:, i) = taur2;
    f1_his(:, i) = f1;  f2_his(:, i) = f2;
    a1_his(:, i) = a1;  a2_his(:, i) = a2;

    display_mothon_flag = 0;
    figure_plot;
    
end




