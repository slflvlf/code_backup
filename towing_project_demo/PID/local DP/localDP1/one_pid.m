clear all; clc; clf;
% global ei M1 Ma1 MM1 D1 integ_var ; M1 = diag([6.25e5, 6.25e5,
% 6.25e5*7.5^2]); Ma1 = [7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5,
% 8.77e6]; MM1 = M1 + Ma1;
addpath(genpath('C:\Users\jiang\Desktop\towing_project_demo\环境力计算'));
wind_param = struct('wind_speed', 27, 'wind_angle',  0/180*pi);
current_param = struct('current_speed', 0.65, 'current_angle', 0/180*pi);
wave_param = struct('Hs', 5.27, 'Tp', 10.4, 'gamma', 3.3, 'env_dir', 0/180*pi, 'nfr', 100);

% [tau_wave1_tug_his, tau_wave2_tug_his, tau_wave1_semi_his, tau_wave2_semi_his] = deal(zeros(3, N));

M0 = diag([5.25e7, 5.25e7, 7.5e10]+[1.7e7, 4.9e7, 5.4e10]);
D0 = diag([2.4e6, 4.8e6, M0(3,3)/20]);
N=8;
integral_error = zeros(3, 1);%积分初值，初始化
p = zeros(3, 1);  v = zeros(3, 1);
p = [0, 0, 0*pi/180]';
max_step = 5200;
[p_his, v_his, pd_his, vd_his, tau_his, taur_his, s_his] = deal(zeros(3, max_step));
[tau_wind_his, tau_current_his, tau_wave1_his, tau_wave2_his] = deal(zeros(3, max_step));
[p_hat_his, v_hat_his] = deal(zeros(3, max_step));
[f_his, a_his] = deal(zeros(N, max_step));

% f = [200; 200; 200];
% a = [45; -45; 60];
f = [100;100;100; 100; 100; 100; 100; 100]*2;
a = [90;115;115;115; -160;-160; -160;-160;];

% Kd = diag([20, 20, 5])*10; Ki = diag([1, 1, 1])*1;

T_observer = 1000;
lambda = 0.1; 
w = 2*pi/10.4;% 波浪谱的谱峰频率
wc = 1.17*w;    %截止频率wc = 1.1-1.2w
step_size = 1;%步长
% K3 = diag([1e5, 1e5, 1e8]);
% K4 = diag([0.6e6, 0.9e6, 1.1e9]);
K4 = 1e6*diag([1 1.42 1.8e3]);
K3 = 0.1*K4;
G = zeros(3);

tau_wind = [-1102.306781;1391.269528;-3450.207656];
observer_paramters = struct('M', M0, 'D', D0, 'G', G, 'T', T_observer, 'lambda', lambda,...
    'w', w, 'wc', wc, 'step_size', step_size, 'K3', K3, 'K4', K4);

T = 100; %reference model的时间常数
p0 = p;
R = [1, 5, pi/4]';

build_time = 200;

state = zeros(15, 1);
[output, p_hat, v_hat] = deal(zeros(3, 1));
%% 主循环
tic
for i = 1 : max_step
    
    i
    current_time = i;
    pose = p;
    [tau_wind_semi, tau_current_semi, tau_wave1_semi, tau_wave2_semi] = tau_env_semi_interface(pose, current_time, wind_param,... 
            current_param, wave_param);

    tau_env0 = tau_wind_semi + tau_current_semi + tau_wave1_semi + tau_wave2_semi;
    tau_env0 = 1*tau_env0;
    if i<=build_time
        factor = i/build_time;
        tau_env0 = tau_env0 * factor;
        tau_wind_semi = tau_wind_semi * factor;
        tau_current_semi = tau_current_semi*factor;
    end
    
    tau_wave1_his(:, i) = tau_wave1_semi;
    tau_wave2_his(:, i) = tau_wave2_semi;
    
    %     % reference model t = i; [pd, dot_pd, ddot_pd] = ref_model(T, t,
    %     p0, R);
    
    %     set point mode pd = [1, 1, pi/3]';
    pd = R;
    dot_pd = [0, 0, 0]';

    pd = [0, 0, 0*pi/180]';
%     pd = [0, 0, 0]';
    dot_pd = [0, 0, 0]';
    
    p_his(:, i) = p;
    v_his(:, i) = v;
    pd_his(:, i) = pd;
    vd_his(:, i) = dot_pd;
    
    
    
    % pid controller
    [tau, integral_error] = pid_controller(pd, dot_pd, p_hat, v_hat, M0, D0, integral_error);
%     tau = tau-tau_wind-tau_current-tau_wave2;%环境力前聩
    tau = tau-tau_wind_semi-tau_current_semi;%环境力前聩
    %     % backstepping controller [tau, s, ei] = bs_constroller(pd,
    %     dot_pd, ddot_pd, p, v, MM1, D1, Kd , Ki, ei);
    
    % 推力分配
    [f,df,a,da,tau_r,dtau] = Thrust_Allocation8(f, a, tau);
    
    % 运动方程求解
    [p, v] = ship_dynamic_solver(p, v, tau_r+tau_env0, M0, D0);
    
    [p_hat, v_hat, state, output] = nonlinear_passive_observer(p,...
        tau_r, state, output, observer_paramters);
    
    p_hat_his(:, i) = p_hat;
    v_hat_his(:, i) = v_hat;
    tau_his(:, i) = tau;
    taur_his(:, i) = tau_r;
    f_his(:, i) = f;
    a_his(:, i) = a;
    %     s_his(:, i) = s;
    
    figure_plot;
    
end
toc

figure(6)
subplot(3,1,1)
plot(p_his(1, build_time+1:end));
hold on
plot(p_hat_his(1, build_time+1:end));
hold off
legend('p', 'p_{hat}')
subplot(3,1,2)
plot(p_his(2, build_time+1:end));
hold on
plot(p_hat_his(2, build_time+1:end));
hold off
subplot(3,1,3)
plot(p_his(3, build_time+1:end)/pi*180);
hold on
plot(p_hat_his(3, build_time+1:end)/pi*180);
hold off



% figure(4); plot(s_his(1, :)); hold on plot(s_his(2, :)); hold on
% plot(s_his(3, : )); hold off legend('sx', 'sy', 'sfai') legend boxoff


