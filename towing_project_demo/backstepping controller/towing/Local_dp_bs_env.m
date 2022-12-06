clear all; clc; clf;
tic
%% 船舶基本参数
% global ei M1 Ma1 MM1 D1 integ_var ;
M1 = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
Ma1 = [7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5, 8.77e6];
MM1 = M1 + Ma1;
D1 = [MM1(1,1)/80, 0, 0; 0, MM1(2,2)/40, 0; 0, 0, MM1(3,3)/5];

%% 环境力
wind_param = struct('wind_speed',5, 'wind_angle',  30/180*pi);
current_param = struct('current_speed', 0.5, 'current_angle', 30/180*pi);
wave_param = struct('Hs', 1.8, 'Tp', 13.4, 'gamma', 3.3, 'env_dir', 30/180*pi, 'nfr', 100);

%% 模拟参数
max_step = 1000;
p = zeros(3, 1);
v = zeros(3, 1);

%% 参数记录
[p_his, v_his, pd_his, vd_his, s_his] = deal(zeros(3, max_step));
[tau_his, f_his, a_his, taur_his] = deal(zeros(3, max_step));
[tau_wind_his, tau_current_his, tau_wave1_his, tau_wave2_his, tau_env_his] =  deal(zeros(3, max_step));
[p_hat, v_hat] = deal(zeros(3, max_step));
%% 推力分配
% 三个浆
f = [100;100;100;];  a = [-50;60;-100];

% % 四个浆
% f = [100;100;100;100];      a = [130;-130;50;-50];
% f_his = zeros(4, max_step); a_his = f_his;

%% 控制器
% Kd = diag([20, 20, 2])*5;
% Ki = diag([1, 1, 0.05])*1;

Kd = diag([20, 20, 40])*5;
Ki = diag([1, 1, 1])*5;

Kd = diag([1, 1, 100])*1e5;
Ki = diag([1, 1, 100])*1e2;

% Kd = diag([1e4, 1e4, 1e5]);
% Ki = diag([0, 0, 1e0]);

integral_error = zeros(3, 1);%积分初值，初始化

%% 导航
T = 800; %reference model的时间常数
p0 = p; 
R = [400, 10, pi/6]';
% 二阶reference model
wn = [0.01, 0.01, 0.009]';
eta = [0.9, 0.9, 1.2]';

% 目的地
Detination = [400, -10, deg2rad(30)]';
% Detination = [0, 0, deg2rad(0)]';

pd = p0;
dot_pd = zeros(3, 1);

%% 滤波器
state = zeros(15, 1);
state(7:9, 1) = p;   output = p;    p_hat = p;  v_hat = zeros(3, 1);
K4 = diag([1.4e4, 1.4e4, 2.5e6]);      
K4 = diag([5.4e4, 5.4e4, 13e6]);
K3 = 0.1 * K4;
wave_w0 = 2*pi/wave_param.Tp;
observer_paramters1 = struct('M', MM1, 'D', D1, 'G', zeros(3, 3), 'T', 100, 'lambda', [0.089, 0.089, 0.089],...
    'w', [wave_w0, wave_w0, wave_w0], 'wc', 1.2*wave_w0, 'step_size', 1, 'K3', K3, 'K4', K4);
% observer_paramters1 = struct('M', MM1, 'D', D1, 'G', zeros(3, 3), 'T', 100, 'lambda', [0.1, 0.1, 0.1],...
%     'w', [wave_w0, wave_w0, wave_w0], 'wc', 1.2*wave_w0, 'step_size', 1, 'K3', K3, 'K4', K4);

%% 主模拟
for i = 1 : max_step
    i
    
   % reference model

    time_step = 1;
    vmax = [2, 1, 1/180*pi/time_step]';
    [pd, dot_pd, ddot_pd] = ref_model_2ord(Detination, pd, dot_pd, wn, eta, vmax, time_step);

%     [pd, dot_pd, ddot_pd] = deal(zeros(3, 1));   

%     [pd, dot_pd, ddot_pd] = ref_model(T, i, p0, R);
    
    p_his(:, i) = p;    v_his(:, i) = v;
    pd_his(:, i) = pd;  vd_his(:, i) = rotate_matrix(pd)' * dot_pd;

    [tau_wind, tau_current, tau_wave1, tau_wave2] = tau_env_tug_interface(p, i, wind_param,... 
            current_param, wave_param);

    tau_wave1 = 0.5 * tau_wave1;    tau_wave2 = tau_wave2;
    tau_wind = 50 * tau_wind;    tau_current = 10 * tau_current;

    tau_wind_his(:, i) = tau_wind;  tau_current_his(:, i) = tau_current;
    tau_wave1_his(:, i) = tau_wave1;    tau_wave2_his(:, i) = tau_wave2;
    tau_env = tau_wind + tau_current + tau_wave1 + tau_wave2;

    if i < 100
        tau_env = i/100 * tau_env;
    end
    tau_env_his(:, i) = tau_env;
        
%     % pid controller
%     [tau, integral_error] = pid_controller(pd, dot_pd, p, v, MM1, D1, integral_error);

    % backstepping controller
    [tau, s, integral_error] = bs_constroller(pd, dot_pd, ddot_pd, p_hat, v_hat, MM1, D1, Kd , Ki, integral_error);
%     tau = tau;

    % 推力分配
%     [f,df,a,da,tau_r,dtau] = Thrust_Allocation(f, a, tau);

    [tau_r, f, a] = Thrust_Allocation_inverse(tau);
%     tau_r = tau;
    
    % 运动方程求解
    [p, v] = ship_dynamic_solver(p, v, tau_r+tau_env, MM1, D1);


    [p_hat, v_hat, state, output] = nonlinear_passive_observer(p, tau_r, state, output, observer_paramters1);

    tau_his(:, i) = tau;    taur_his(:, i) = tau_r;
    f_his(:, i) = f;    a_his(:, i) = a;
    s_his(:, i) = s;

    p_hat_his(:, i) = p_hat;    v_hat_his(:, i) = v_hat;


    
end
%     figure_plot;
toc





%% position and velocity
figure(1)
title('位置和速度');
subplot(3, 2, 1);
plot(p_his(1, :)); hold on; plot(pd_his(1, :)); hold on; plot(p_hat_his(1, :)); hold off;
legend('p', 'pd', 'p_{hat}')
subplot(3, 2, 3);
plot(p_his(2, :)); hold on; plot(pd_his(2, :)); hold on; plot(p_hat_his(2, :)); hold off;
subplot(3, 2, 5);
plot(p_his(3, :)/pi*180); hold on; plot(pd_his(3, :)/pi*180); hold on; plot(p_hat_his(3, :)/pi*180); hold off
subplot(3, 2, 2);
plot(v_his(1, :)); hold on; plot(vd_his(1, :)); hold on; plot(v_hat_his(1, :)); hold off
legend('p', 'pd', 'p_hat'); legend boxoff
subplot(3, 2, 2);
plot(v_his(1, :)); hold on; plot(vd_his(1, :)); hold on; plot(v_hat_his(1, :)); hold off
legend('v', 'vd', 'v_{hat}'); legend boxoff
subplot(3, 2, 4);
plot(v_his(2, :)); hold on; plot(vd_his(2, :)); hold on; plot(v_hat_his(2, :)); hold off
subplot(3, 2, 6);
plot(v_his(3, :)/pi*180); hold on; plot(vd_his(3, :)/pi*180); hold on; plot(v_hat_his(3, :)/pi*180); hold off


figure(2)
title('推力分配');
subplot(3, 3, 1); plot(tau_his(1, :)); hold on; plot(taur_his(1, :)); hold off
legend('tau', 'tau_r'); legend boxoff
subplot(3, 3, 4); plot(tau_his(2, :)); hold on; plot(taur_his(2, :)); hold off
subplot(3, 3, 7); plot(tau_his(3, :)); hold on; plot(taur_his(3, :)); hold off
subplot(3, 3, 2); plot(f_his(1,:));
subplot(3, 3, 5); plot(f_his(2,:));
subplot(3, 3, 8); plot(f_his(3,:));
subplot(3, 3, 3); plot(a_his(1,:)/pi*180);
subplot(3, 3, 6); plot(a_his(2,:)/pi*180);
subplot(3, 3, 9); plot(a_his(3,:)/pi*180);

figure(4);
plot(s_his(1, :));  hold on; plot(s_his(2, :));  hold on; plot(s_his(3, : )/pi*180);  hold off
legend('sx', 'sy', 'sfai'); legend boxoff


figure(5)
subplot(3, 1, 1)
plot(tau_wind_his(1, :)); hold on; plot(tau_current_his(1, :)); hold on;
plot(tau_wave1_his(1, :)); hold on; plot(tau_wave2_his(1, :)); hold off;
legend("wind", "current", "wave1", "wave2");
subplot(3, 1, 2)
plot(tau_wind_his(2, :)); hold on; plot(tau_current_his(2, :)); hold on;
plot(tau_wave1_his(2, :)); hold on; plot(tau_wave2_his(2, :)); hold off;
subplot(3, 1, 3)
plot(tau_wind_his(3, :)); hold on; plot(tau_current_his(3, :)); hold on;
plot(tau_wave1_his(3, :)); hold on; plot(tau_wave2_his(3, :)); hold off;
% 
% 
figure(6)
subplot(3,1,1)
plot(tau_wave1_his(1, :)); hold on; plot(tau_wave2_his(1, :)); hold off;
legend("wave1", "wave2");
subplot(3,1,2)
plot(tau_wave1_his(2, :)); hold on; plot(tau_wave2_his(2, :)); hold off;
subplot(3,1,3)
plot(tau_wave1_his(3, :)); hold on; plot(tau_wave2_his(3, :)); hold off;



%% 旋转矩阵
function R = rotate_matrix(p)
    fai = p(3);
    R = [cos(fai),  -sin(fai),  0;
         sin(fai),  cos(fai),   0;
         0,          0,         1];   
end