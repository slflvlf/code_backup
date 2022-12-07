
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



%% 角度转化成（-pi, pi)

x = 4 * pi/3;
y = transfer_deg(x) / pi * 180


function y = transfer_deg(x)
    y = -sign(x) * pi + rem((x + sign(x) * pi), 2 * pi);
end


% function y = transter_deg(x)
%     y = -sign(x) * 180 + rem((x +






