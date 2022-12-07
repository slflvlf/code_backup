%% 滤波器 测试
clc;
p = out.p.data;
v = out.v.data;
p_hat_his = out.p_hat.data;
v_hat_his = out.v_hat.data;
tau = out.tau.data;

%%%%%%%%%滤波器参数设置%%%%%%%%%%%%%
p0 = zeros(3, 1);
wave_param = struct('Hs', 2.8, 'Tp', 13.4, 'gamma', 3.3, 'env_dir', 0/180*pi, 'nfr', 100);
state = zeros(15, 1);
state(7:9, 1) = p0;
output = p0;
p_hat = p0;
v_hat = zeros(3, 1);
K4 = diag([1.4e4, 1.6e4, 2.5e6]);
K3 = 0.1 * K4;
wave_w0 = 2*pi/wave_param.Tp;
observer_paramters1 = struct('M', MM1, 'D', D1, 'G', zeros(3, 3), 'T', 1000, 'lambda', 0.1,...
    'w', wave_w0, 'wc', 1.3*wave_w0, 'step_size', 1, 'K3', K3, 'K4', K4);

[p_hat1_his, v_hat1_his] = deal(zeros(length(p), 3));

for i = 1 : length(p)
    i
    [p_hat1, v_hat1, state, output] = nonlinear_passive_observer1(p(i, :)', 1e3*tau(i, :)', state, output, observer_paramters1);
    p_hat1_his(i, :) = p_hat1;
    v_hat1_his(i, :) = v_hat1; 
end

figure(1)
subplot(3, 1, 1)
plot(p(:, 1))
hold on
plot(p_hat_his(:, 1))
hold on
plot(p_hat1_his(:, 1))
hold off
legend('p_{measure}', 'p_{hat}', 'p_{hat1}')
legend boxoff

subplot(3, 1, 2)
plot(p(:, 2))
hold on
plot(p_hat_his(:, 2))
hold on
plot(p_hat1_his(:, 2))
hold off


subplot(3, 1, 3)
plot(p(:, 3))
hold on
plot(p_hat_his(:, 3))
hold on
plot(p_hat1_his(:, 3))
hold off



figure(2)
subplot(3, 1, 1)
plot(v(:, 1))
hold on
plot(v_hat_his(:, 1))
hold on
plot(v_hat1_his(:, 1))
hold off
legend('v_{measure}', 'v_{hat}', 'v_{hat1}')
legend boxoff

subplot(3, 1, 2)
plot(v(:, 2))
hold on
plot(v_hat_his(:, 2))
hold on
plot(v_hat1_his(:, 2))
hold off


subplot(3, 1, 3)
plot(v(:, 3))
hold on
plot(v_hat_his(:, 3))
hold on
plot(v_hat1_his(:, 3))
hold off

%% 能量谱
N = 2001;
y = 10*log10(abs(fft(p_hat_his(:,1)).^2)/N);
f=(0:length(y)-1)/length(y)
figure(1);
plot(f,y);


