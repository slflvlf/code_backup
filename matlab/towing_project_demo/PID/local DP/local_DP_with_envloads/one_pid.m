clear all; clc; clf;

% global ei M1 Ma1 MM1 D1 integ_var ;
M1 = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
Ma1 = [7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5, 8.77e6];
MM1 = M1 + Ma1;
D1 = [MM1(1,1)/100, 0, 0; 0, MM1(2,2)/40, 0; 0, 0, MM1(3,3)/20];

integral_error = zeros(3, 1);%积分初值，初始化

max_step = 500;

p = zeros(3, 1); p_his = zeros(3, max_step); pd_his = zeros(3, max_step);
v = zeros(3, 1); v_his = zeros(3, max_step); vd_his = zeros(3, max_step);
tau_his = p_his;  f_his = p_his; a_his = f_his; taur_his = p_his;
s_his = p_his;
[tau_wind_his, tau_current_his, tau_wave1_his, tau_wave2_his, tau_env_his] = deal(zeros(3, max_step));


f = [100;100;100];
a = [-50;60;-100];


wind_param = struct('wind_speed', 27, 'wind_angle',  0/180*pi);
current_param = struct('current_speed', 0.65, 'current_angle', 0/180*pi);
wave_param = struct('Hs', 5.27, 'Tp', 10.4, 'gamma', 3.3, 'env_dir', 0/180*pi, 'nfr', 100);

% Kd = diag([20, 20, 5])*10;
% Ki = diag([1, 1, 1])*1;

T = 100; %reference model的时间常数

p0 = p;
R = [1, 5, pi/4]';
tic
for i = 1 : max_step

    i
    [tau_wind, tau_current, tau_wave1, tau_wave2] = tau_env_tug_interface(p, i, wind_param, current_param, wave_param);
    tau_env = tau_wind + tau_current + tau_wave1 + tau_wave2;
    
    tau_wind_his(:, i) = tau_wind;
    tau_current_his(:, i) = tau_current;
    tau_wave1_his(:, i) = tau_wave1; 
    tau_wave2_his(:, i) = tau_wave2;
    tau_env_his(:, i) = tau_env;
    
    
    % reference model
    t = i;
    [pd, dot_pd, ddot_pd] = ref_model(T, t, p0, R);
    
%     set point mode
%     pd = [1, 1, pi/3]';
%     pd = R;
%     dot_pd = [0, 0, 0]';
%     ddot_pd = [0, 0, 0]';
    
    p_his(:, i) = p;
    v_his(:, i) = v;
    pd_his(:, i) = pd;
    vd_his(:, i) = dot_pd;
    
    
    % pid controller
    [tau, integral_error] = pid_controller(pd, dot_pd, p, v, MM1, D1, integral_error);

%     % backstepping controller
%     [tau, s, ei] = bs_constroller(pd, dot_pd, ddot_pd, p, v, MM1, D1, Kd , Ki, ei);

    % 推力分配
    [f,df,a,da,tau_r,dtau] = Thrust_Allocation(f, a, tau);
    
    % 运动方程求解
    [p, v] = ship_dynamic_solver(p, v, tau_r, MM1, D1);

    f0=f;
    a0=a;
    tau_his(:, i) = tau;
    taur_his(:, i) = tau_r;
    f_his(:, i) = f;
    a_his(:, i) = a;
%     s_his(:, i) = s;

    figure_plot;
    
end
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


