clear all;clc;clf;
load("matlab.mat")
M = M + Ma;
p_measure = p.data;
p_hat_demo = p_hat.data;
v_hat_demo = v_hat.data;

T = 1000;
lambda = 0.1; 
w = 2*pi/10;% 波浪谱的谱峰频率
wc = 1.1*w;    %截止频率wc = 1.1-1.2w
step_size = 1;%步长
% K3 = diag([1e5, 1e5, 1e8]);
% K4 = diag([0.6e6, 0.9e6, 1.1e9]);
K4 = 1e6*diag([2 2 3500]);
K3 = 0.1*K4;
G = zeros(3);

tau_wind = [-1102.306781;1391.269528;-3450.207656];
observer_paramters = struct('M', M, 'D', D, 'G', G, 'T', T, 'lambda', lambda,...
    'w', w, 'wc', wc, 'step_size', step_size, 'K3', K3, 'K4', K4);

% G = zeros(3);

tau = tau_real.data;
[p_hat_his,v_hat_his]=deal(zeros(3, length(p_measure)));
state_his = zeros(15, length(p_measure));

output = zeros(3, 1);

p_init = zeros(3, 1);
state = zeros(15, 1);
state(7:9) = p_init;

for i = 1 : length(p_measure)

    [p_hat, v_hat, state, output] = nonlinear_passive_observer(p_measure(i, :)',...
        tau(i, :)'+tau_wind, state, output, observer_paramters);

    p_hat_his(:, i) = p_hat;
    v_hat_his(:, i) = v_hat;
    state_his(:, i) = state;
end
figure(1)
subplot(3, 1, 1)
plot(p_measure(:, 1))
hold on
plot(p_hat_his(1, :))
hold on
plot(p_hat_demo(:, 1))
hold off
legend('p_{measure}', 'p_{hat}', 'p_{hatdemo}')
legend boxoff

subplot(3, 1, 2)
plot(p_measure(:, 2))
hold on
plot(p_hat_his(2, :))
hold on
plot(p_hat_demo(:, 2))
hold off
legend('p_{measure}', 'p_{hat}', 'p_{hat1}')
legend boxoff

subplot(3, 1, 3)
plot(p_measure(:, 3))
hold on
plot(p_hat_his(3, :))
hold on
plot(p_hat_demo(:, 3))
hold off
legend('p_{measure}', 'p_{hat}', 'p_{hat1}')
legend boxoff


figure(2)
subplot(3, 1, 1)
plot(v_hat_his(1, :))
hold on
plot(v_hat_demo(:, 1))
hold off
legend( 'v_{hat}', 'v_{hatdemo}')
legend boxoff

subplot(3, 1, 2)
plot(v_hat_his(2, :))
hold on
plot(v_hat_demo(:, 2))
hold off

subplot(3, 1, 3)
plot(v_hat_his(3, :))
hold on
plot(v_hat_demo(:, 3))
hold off


%% 李博c++程序里的滤波器参数设计
% M = diag([5.2e7, 5.2e7, 7.5e10])+diag([1.7e7, 4.9e7, 5.4e10]);
% D = diag([2.4e6, 4.8e6, 3.9e7]);
% G = diag([87527, 87549, 7.2e8]);
% T = 1000;
% lambda = 0.1; 
% w = 2*pi/10;% 波浪谱的谱峰频率
% wc = 0.7;    %截止频率
% step_size = 0.1;%步长
% % K3 = diag([1e5, 1e5, 1e8]);
% K4 = diag([1e6, 1e6, 1e9]);
% K3 = 0.1*K4;



%% alpha beta observer test
clear all; clc;
N = 2001;
xk_his = zeros(3, N);
xm_his = zeros(3,N);
load("matlab.mat")
% load("60deg.mat")
p_measure = p.data;
p_hat_demo = p_hat.data;
dt = 1;
xk = [0,0,0]';
vk = [0,0,0]'; 
a = 0.15;
b = 0.005;
N = 2001;
% xm_his = sin((1:N)/N*2*pi)+ rand(1, N) * 0.1;


for i = 1 : N
    xm = p_measure(i, :)';
    [xk, vk] = alpha_beta_observer(a, b, xm, dt, xk, vk)
    xk_his(:, i) = xk;
    xm_his(:, i) = xm;
end
figure(1)
subplot(3, 1, 1)
plot(xk_his(1, :))
hold on 
plot(xm_his(1, :))
hold off
legend('x_{hat}', 'x_m')
legend boxoff

subplot(3, 1, 2)
plot(xk_his(2, :))
hold on 
plot(xm_his(2, :))
hold off
legend('y_{hat}', 'y_m')
legend boxoff

subplot(3, 1, 3)
plot(xk_his(3, :))
hold on 
plot(xm_his(3, :))
hold off
legend('fai_{hat}', 'fai_m')
legend boxoff



%% 测试新dp观测器
clear all;clc;clf;
load("matlab.mat")
M = M + Ma;
p_measure = p.data;
p_hat_demo = p_hat.data;

T = 1000;
lambda = 0.1; 
w = 2*pi/13.4;% 波浪谱的谱峰频率
wc = 1.1*w;    %截止频率wc = 1.1-1.2w
step_size = 1;%步长
% K3 = diag([1e5, 1e5, 1e8]);
% K4 = diag([0.6e6, 0.9e6, 1.1e9]);
K4 = 1e6*diag([2 2 3500]);
K3 = 0.1*K4;
G = zeros(3);

tau_wind = [-1102.306781;1391.269528;-3450.207656];
observer_paramters = struct('M', M, 'D', D, 'G', G, 'T', T, 'lambda', lambda,...
    'w', w, 'wc', wc, 'step_size', step_size, 'K3', K3, 'K4', K4);

% G = zeros(3);

tau = tau_real.data;
[p_hat_his,v_hat_his]=deal(zeros(3, length(p_measure)));
state_his = zeros(15, length(p_measure));

output = zeros(3, 1);

p_init = zeros(3, 1);
state = zeros(15, 1);
state(7:9) = p_init;

for i = 1 : length(p_measure)

    [p_hat, v_hat, state, output] = nonlinear_passive_observer(p_measure(i, :)',...
        tau(i, :)', state, output, observer_paramters);

    p_hat_his(:, i) = p_hat;
    v_hat_his(:, i) = v_hat;
    state_his(:, i) = state;
end
figure(1)
subplot(3, 1, 1)
plot(p_measure(:, 1))
hold on
plot(p_hat_his(1, :))
hold on
plot(p_hat_demo(:, 1))
hold off
legend('p_{measure}', 'p_{hat}', 'p_{hat1}')
legend boxoff

subplot(3, 1, 2)
plot(p_measure(:, 2))
hold on
plot(p_hat_his(2, :))
hold on
plot(p_hat_demo(:, 2))
hold off
legend('p_{measure}', 'p_{hat}', 'p_{hat1}')
legend boxoff

subplot(3, 1, 3)
plot(p_measure(:, 3))
hold on
plot(p_hat_his(3, :))
hold on
plot(p_hat_demo(:, 3))
hold off
legend('p_{measure}', 'p_{hat}', 'p_{hat1}')
legend boxoff

%% 李博c++程序里的滤波器参数设计
% M = diag([5.2e7, 5.2e7, 7.5e10])+diag([1.7e7, 4.9e7, 5.4e10]);
% D = diag([2.4e6, 4.8e6, 3.9e7]);
% G = diag([87527, 87549, 7.2e8]);
% T = 1000;
% lambda = 0.1; 
% w = 2*pi/10;% 波浪谱的谱峰频率
% wc = 0.7;    %截止频率
% step_size = 0.1;%步长
% % K3 = diag([1e5, 1e5, 1e8]);
% K4 = diag([1e6, 1e6, 1e9]);
% K3 = 0.1*K4;



%% 测试新dp观测器,60deg
clear all;clc;clf;
load("60deg.mat")
M = M0;
p_measure = p_his;
p_hat_demo = p_hat_his;

T = 1000;
lambda = 0.1; 
w = 2*pi/10.4;% 波浪谱的谱峰频率
wc = 1.1*w;    %截止频率wc = 1.1-1.2w
step_size = 1;%步长
% K3 = diag([1e5, 1e5, 1e8]);
% K4 = diag([0.6e6, 0.9e6, 1.1e9]);
K4 = 1e6*diag([2 2 3500]);
K3 = 0.1*K4;
G = zeros(3);

% tau_wind = [-1102.306781;1391.269528;-3450.207656];
observer_paramters = struct('M', M, 'D', D, 'G', G, 'T', T, 'lambda', lambda,...
    'w', w, 'wc', wc, 'step_size', step_size, 'K3', K3, 'K4', K4);

% G = zeros(3);

tau = taur_his;
[p_hat_his,v_hat_his]=deal(zeros(3, length(p_measure)));
state_his = zeros(15, length(p_measure));

output = zeros(3, 1);

p_init = zeros(3, 1);
state = zeros(15, 1);
state(7:9) = p_init;

for i = 1 : length(p_measure)

    [p_hat, v_hat, state, output] = nonlinear_passive_observer(p_measure(i, :)',...
        tau(i, :)', state, output, observer_paramters);

    p_hat_his(:, i) = p_hat;
    v_hat_his(:, i) = v_hat;
    state_his(:, i) = state;
end
figure(1)
subplot(3, 1, 1)
plot(p_measure(:, 1))
hold on
plot(p_hat_his(1, :))
hold on
plot(p_hat_demo(:, 1))
hold off
legend('p_{measure}', 'p_{hat}', 'p_{hat1}')
legend boxoff

subplot(3, 1, 2)
plot(p_measure(:, 2))
hold on
plot(p_hat_his(2, :))
hold on
plot(p_hat_demo(:, 2))
hold off
legend('p_{measure}', 'p_{hat}', 'p_{hat1}')
legend boxoff

subplot(3, 1, 3)
plot(p_measure(:, 3))
hold on
plot(p_hat_his(3, :))
hold on
plot(p_hat_demo(:, 3))
hold off
legend('p_{measure}', 'p_{hat}', 'p_{hat1}')
legend boxoff



