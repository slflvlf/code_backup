clear all; clc; clf;
tic
% addpath("C:\Users\jiang\Desktop\towing_project_demo\环境力计算");
% 环境力参数设置 
wind_param = struct('wind_speed', 3, 'wind_angle',  30/180*pi);
current_param = struct('current_speed', 0.3, 'current_angle', 30/180*pi);
wave_param = struct('Hs', 1.8, 'Tp', 13.4, 'gamma', 3.3, 'env_dir', 30/180*pi, 'nfr', 100);


% global ei M1 Ma1 MM1 D1 integ_var ;
M1 = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
Ma1 = [7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5, 8.77e6];
MM1 = M1 + Ma1;
D1 = [MM1(1,1)/100, 0, 0; 0, MM1(2,2)/40, 0; 0, 0, MM1(3,3)/20];

integral_error = zeros(3, 1);%积分初值，初始化

max_step = 1000;

p = zeros(3, 1); p_his = zeros(3, max_step); pd_his = zeros(3, max_step);
v = zeros(3, 1); v_his = zeros(3, max_step); vd_his = zeros(3, max_step);
tau_his = p_his;  f_his = p_his; a_his = f_his; taur_his = p_his;
s_his = p_his;
[p_hat_his, v_hat_his, tau_wind_his, tau_current_his, tau_wave1_his, tau_wave2_his, tau_wave_his] = deal(zeros(3, max_step));

%%%%%%%%%%%%推力分配%%%%%%%%%%%%%%%%%
f = [100;100;100];
a = [-50;60;-100];

%%%%%%%%%%%%导航参数设置%%%%%%%%%%%%%
T = 100; %reference model的时间常数
p0 = p;
R = [10, 15, pi/3]';

%%%%%%%%%滤波器参数设置%%%%%%%%%%%%%
state = zeros(15, 1);
state(7:9, 1) = p0;
output = p0;
p_hat = p0;
v_hat = zeros(3, 1);
K4 = diag([1.4e4, 1.4e4, 2.5e6]);
K3 = 0.1 * K4;
wave_w0 = 2*pi/wave_param.Tp;
observer_paramters1 = struct('M', MM1, 'D', D1, 'G', zeros(3, 3), 'T', 100, 'lambda', [0.089, 0.089, 0.1],...
    'w', [wave_w0, wave_w0, wave_w0], 'wc', 1.2*wave_w0, 'step_size', 1, 'K3', K3, 'K4', K4);

%%%%%%%%%%%%控制器参数设置%%%%%%%%%%%%%
Kd = diag([2.5, 2.5, 2])*1;
Ki = diag([0.1, 0.1, 0.05])*0.1;

% Kd = diag([20, 20, 5])*0.1;
% Ki = diag([1, 1, 1])*0.01;



for i = 1 : max_step
    i
    
    % reference model
    t = i;
    [pd, dot_pd, ddot_pd] = ref_model(T, t, p0, R);
    
% % %     set point mode
%     pd = [2, 1, pi/3]';
% % %     pd = [1, 0, 0]';
% % %     pd = R;
%     dot_pd = [0, 0, 0]';
%     ddot_pd = [0, 0, 0]';
    
    p_his(:, i) = p;
    v_his(:, i) = v;
    pd_his(:, i) = pd;
    vd_his(:, i) = dot_pd;
    current_time = i;
    [tau_wind_tug, tau_current_tug, tau_wave1_tug, tau_wave2_tug] = tau_env_tug_interface(p, current_time, wind_param,... 
            current_param, wave_param);
    
    tau_wind_tug = 1 * tau_wind_tug;
    tau_current_tug = 1 * tau_current_tug;
    tau_wave1_tug = 1 * tau_wave1_tug;
    tau_wave2_tug = 1 * tau_wave2_tug;
    tau_wind_his(:, i) = tau_wind_tug;
    tau_current_his(:, i) = tau_current_tug;
    tau_wave1_his(:, i) = tau_wave1_tug;
    tau_wave2_his(:, i) = tau_wave2_tug;
    



%     % pid controller
%     [tau, integral_error] = pid_controller(pd, dot_pd, p_hat, v_hat, MM1, D1, integral_error);

%     % backstepping controller
    [tau, s, integral_error] = bs_constroller(pd, dot_pd, ddot_pd, p_hat, v_hat, MM1, D1, Kd , Ki, integral_error);
    
%     %加入前馈（风和流）
%     tau = tau - tau_wind_tug - tau_current_tug;
    
    % 推力分配
    [f,df,a,da,tau_r,dtau] = Thrust_Allocation(f, a, tau);
    
    % 运动方程求解
    tau_env = tau_wind_tug + tau_current_tug + tau_wave1_tug + tau_wave2_tug;
%     tau_env =  tau_wave1_tug + tau_wave2_tug;
    [p, v] = ship_dynamic_solver(p, v, tau_r + tau_env, MM1, D1);
    
    [p_hat, v_hat, state, output] = nonlinear_passive_observer(p, tau_r, state, output, observer_paramters1);
    p_hat_his(:, i) = p_hat;
    v_hat_his(:, i) = v_hat;
%     p = p_next;
%     v = v_next;
%     f0=f;
%     a0=a;
    tau_his(:, i) = tau;
    taur_his(:, i) = tau_r;
    f_his(:, i) = f;
    a_his(:, i) = a;
%     s_his(:, i) = s;
    motion_display_flag = 0;
    figure_plot;
    
end
toc

figure(4)
subplot(3, 2, 1)
plot(p_hat_his(1, :));
hold on 
plot(p_his(1, :));
hold off
legend("p_hat", "p");

subplot(3, 2, 3)
plot(p_hat_his(2, :));
hold on 
plot(p_his(2, :));
hold off

subplot(3, 2, 5)
plot(p_hat_his(3, :)/pi*180);
hold on 
plot(p_his(3, :)/pi*180);
hold off

subplot(3, 2, 2)
plot(v_hat_his(1, :));
hold on 
plot(v_his(1, :));
hold off
legend("v_hat", "v");

subplot(3, 2, 4)
plot(v_hat_his(2, :));
hold on 
plot(v_his(2, :));
hold off

subplot(3, 2, 6)
plot(v_hat_his(3, :));
hold on 
plot(v_his(3, :));
hold off

figure(5)
subplot(3, 1, 1)
plot(tau_wind_his(1, :));
hold on 
plot(tau_current_his(1, :));
hold on 
plot(tau_wave1_his(1, :));
hold on 
plot(tau_wave2_his(1, :));
hold off
legend("wind", "current", "wave1", "wave2")
subplot(3, 1, 2)
plot(tau_wind_his(2, :));
hold on 
plot(tau_current_his(2, :));
hold on 
plot(tau_wave1_his(2, :));
hold on 
plot(tau_wave2_his(2, :));
hold off
subplot(3, 1, 3)
plot(tau_wind_his(3, :));
hold on 
plot(tau_current_his(3, :));
hold on 
plot(tau_wave1_his(3, :));
hold on 
plot(tau_wave2_his(3, :));
hold off

wave_load_data = textread("wave_load_file.txt");
time = wave_load_data(2e4:end, 1);
wave1_load = [wave_load_data(2e4:end, 2:3), wave_load_data(2e4:end, 7)];
wave2_load = [wave_load_data(2e4:end, 14:15), wave_load_data(2e4:end, 19)];
figure
plot(time, wave1_load(:, 1)/1e3);
hold on
plot(time, wave2_load(:, 1)/1e3);
hold off
legend("wave1", "wave2")

% figure(4);
% plot(s_his(1, :));
% hold on
% plot(s_his(2, :));
% hold on
% plot(s_his(3, : ));
% hold off
% legend('sx', 'sy', 'sfai')
% legend boxoff


