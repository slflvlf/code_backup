clear all; clc; clf;
tic
%% 船舶基本参数
% global ei M1 Ma1 MM1 D1 integ_var ;
M1 = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
Ma1 = [7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5, 8.77e6];
MM1 = M1 + Ma1;
D1 = [MM1(1,1)/100, 0, 0; 0, MM1(2,2)/40, 0; 0, 0, MM1(3,3)/20];


%% 模拟参数
max_step = 1000;
p = zeros(3, 1);
v = zeros(3, 1);

%% 参数记录
[p_his, v_his, pd_his, vd_his, s_his] = deal(zeros(3, max_step));
[tau_his, f_his, a_his, taur_his] = deal(zeros(3, max_step));

%% 推力分配
f = [100;100;100];
a = [-50;60;-100];

%% 控制器
Kd = diag([20, 20, 2])*5;
Ki = diag([1, 1, 0.05])*1;

integral_error = zeros(3, 1);%积分初值，初始化
time_step = 1;

%% 导航
T = 100; %reference model的时间常数
p0 = p;
R = [10, 15, pi/3]';

% 二阶reference model
wn = [0.01, 0.01, 0.009]';
eta = [0.9, 0.9, 1.2]';

% 目的地
Detination = [100, 10, deg2rad(30)]';

pd = p0;
dot_pd = zeros(3, 1);

%% 主模拟
for i = 1 : max_step
    i
    
   % reference model 1st
%     [pd, dot_pd, ddot_pd] = ref_model(T, i, p0, R);

    time_step = 1;
    vmax = [2, 1, 1/180*pi/time_step]';
    [pd, dot_pd, ddot_pd] = ref_model_2ord(Detination, pd, dot_pd, wn, eta, vmax, time_step);


    p_his(:, i) = p;    v_his(:, i) = v;
    pd_his(:, i) = pd;  vd_his(:, i) = dot_pd;
        
%     % pid controller
%     [tau, integral_error] = pid_controller(pd, dot_pd, p, v, MM1, D1, integral_error);
    [tau, integral_error] = pid_controller(pd, dot_pd, p, v, MM1, D1, integral_error, time_step);

    % backstepping controller
%     [tau, s, integral_error] = bs_constroller(pd, dot_pd, ddot_pd, p, v, MM1, D1, Kd , Ki, integral_error);

    % 推力分配
    [f,df,a,da,tau_r,dtau] = Thrust_Allocation(f, a, tau);
    tau_r = tau;

    % 运动方程求解
    [p, v] = ship_dynamic_solver(p, v, tau_r, MM1, D1);


    tau_his(:, i) = tau;    taur_his(:, i) = tau_r;
    f_his(:, i) = f;    a_his(:, i) = a;
%     s_his(:, i) = s;


    
end
    figure_plot;
toc

% figure(4);
% plot(s_his(1, :));
% hold on
% plot(s_his(2, :));
% hold on
% plot(s_his(3, : ));
% hold off
% legend('sx', 'sy', 'sfai')
% legend boxoff


