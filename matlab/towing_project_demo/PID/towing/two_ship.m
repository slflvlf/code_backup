%% 测试用
clear all; clc; clf;

M1 = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
Ma1 = [7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5, 8.77e6];
MM1 = M1 + Ma1;
D1 = [MM1(1,1)/100, 0, 0; 0, MM1(2,2)/40, 0; 0, 0, MM1(3,3)/20];

% pid 控制器中的误差积分的初值
integral_error1 = zeros(3, 1);%积分初值，初始化
integral_error2 = zeros(3, 1);

max_step = 1000;

%变量初始化和历史记录
[p0_his, p1_his, v1_his, pd1_his, vd1_his, p2_his, v2_his, pd2_his, vd2_his, ...
 tau1_his, taur1_his, f1_his, a1_his, tau2_his, taur2_his, f2_his, a2_his, length_his]...
 = deal(zeros(3, max_step));





delta10 = [-10, 10, 0]'*2;
delta20 = [-10, -10, 0]'*2;

p0 = [0, 0, 0]';
p1 = p0 + delta10;
p2 = p0 + delta20;

v1 = zeros(3, 1);
v2 = zeros(3, 1);


% 间距转换为大地坐标系下
R0 = rotate_matrix(p0);
delta10_g = R0 * delta10;
delta20_g = R0 * delta20;

p00 = p0;

% 推力分配中，螺旋桨初始推力和转角
f1 = [100;100;100];
a1 = [-50;60;-100];
f2 = [100;100;100];
a2 = [-50;60;-100];




%% 主循环
%reference model的时间常数
% 表示一个时间常数会达到目标值的63%
T = 100; 
p10 = p1;
Detination = [20, 30, pi/4]';

for i = 1 : max_step
    [length01, length02, length12] = distance_among_ships(p0, p1, p2);
    
    length_his(:, i) = [length01-20*sqrt(2), length02-20*sqrt(2), length12-40]';
    
    % reference model,领航者的位置
    t = i;
    [pd0, dot_pd0, ddot_pd0] = ref_model(T, t, p00, Detination);
    
    
    % 间距转换为大地坐标系下
    R0 = rotate_matrix(p0);
    delta10_g = R0 * delta10;
    delta20_g = R0 * delta20;
    
    % 加入dot_p 的偏差
    delta10_g = [delta10_g; 0; 0; 0];
    delta20_g = [delta20_g; 0; 0; 0];
    
    delta12_g = delta10_g - delta20_g;
    
    %将速度v转化为dot_p
    R1 = rotate_matrix(p1);
    R2 = rotate_matrix(p2);
    
    
    %定义协同变量（一致性）
    x0 = [pd0; dot_pd0];
    x1 = [p1; R1 * v1];
    x2 = [p2; R2 * v2];
    
    error1 = (x2 - x1 + delta12_g) + (x0 - x1 + delta10_g);
    error2 = (x1 - x2 - delta12_g) + (x0 - x2 + delta20_g);
    
    wn = 0.05; 
    cd = 0.7;

    A1 = -2 * cd * wn * eye(3);
    A0 = -wn^2 * eye(3);
    B1 = -A0;

    A = [zeros(3, 3), eye(3); A0, A1];
    B = [zeros(3, 3); B1];

Q = diag([100, 100, 100, 1, 1, 1]);
% R = diag([0.01, 0.01, 4e-8]);
%     Q = 40 * eye(6);
%     Q(4, 4) = 1; Q(5, 5) = 1; Q(6, 6) = 1;
    R = 1 * eye(3);
    N = [];
    [K, S, e] = lqr(A, B, Q, R, N);

    K;
    u1 = K * error1;
    u2 = K * error2;

    pd1 = u1 + p1;
    pd2 = u2 + p2;
    
    dot_pd1 = zeros(3, 1); dot_pd2 = dot_pd1;
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
    
    [tau1, integral_error1] = pid_controller(pd1, dot_pd1, p1, v1, MM1, D1, integral_error1);
    [tau2, integral_error2] = pid_controller(pd2, dot_pd2, p2, v2, MM1, D1, integral_error2);
    
%     [tau1, s1, ei1] = bs_constroller(pd1, dot_pd1, ddot_pd1, p1, v1, MM1, D1, Kd , Ki, ei1);
%     [tau2, s2, ei2] = bs_constroller(pd2, dot_pd2, ddot_pd2, p2, v2, MM1, D1, Kd , Ki, ei2);
%     
%     [tauc1, tauc2] = coop_controller(s1, s2, Kc1, Kc2);
%     % 加入一致性协议
%     tau1 = tau1 + tauc1;    tau2 = tau2 + tauc2;
    
    [f1,df1,a1,da1,taur1,dtau1] = Thrust_Allocation(f1,a1,tau1);
    [f2,df2,a2,da2,taur2,dtau2] = Thrust_Allocation(f2,a2,tau2);
    
    [p1, v1] = ship_dynamic_solver(p1, v1, taur1, MM1, D1);
    [p2, v2] = ship_dynamic_solver(p2, v2, taur2, MM1, D1); 
    
    tau1_his(:, i) = tau1;
    taur1_his(:, i) = taur1;
%     tauc1_his(:, i) = tauc1;
    f1_his(:, i) = f1;
    a1_his(:, i) = a1;
%     s1_his(:, i) = s1;
    
    tau2_his(:, i) = tau2;
    taur2_his(:, i) = taur2;
%     tauc2_his(:, i) = tauc2;
    f2_his(:, i) = f2;
    a2_his(:, i) = a2;
%     s2_his(:, i) = s2;
    
    display_motion_flag = 1;
    figure_plot2;

    if i == 1
        error1
        error2
        K
        pd1
        pd2
    end
        
        
    
end







%% 旋转矩阵
function R= rotate_matrix(p)
    fai = p(3);
    R = [cos(fai), -sin(fai), 0;
        sin(fai), cos(fai), 0;
        0, 0, 1];
end
%% 将角度转化为（-pi, pi)
function y = transfer_deg(x)
    y = -sign(x) * pi + rem((x + sign(x) * pi), 2 * pi);
end

%% 多船的间距
function [length01, length02, length12] = distance_among_ships(p0, p1, p2)

    length01 = sqrt( (p0(1)-p1(1))^2 + (p0(2)-p1(2))^2 );
    length02 = sqrt( (p0(1)-p2(1))^2 + (p0(2)-p2(2))^2 );
    length12 = sqrt( (p1(1)-p2(1))^2 + (p1(2)-p2(2))^2 );

end