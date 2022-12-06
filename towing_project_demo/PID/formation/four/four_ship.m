%% 测试用
clear all; clc; clf;

M1 = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
Ma1 = [7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5, 8.77e6];
MM1 = M1 + Ma1;
D1 = [MM1(1,1)/100, 0, 0; 0, MM1(2,2)/40, 0; 0, 0, MM1(3,3)/20];

% pid 控制器中的误差积分的初值
integral_error1 = zeros(3, 1);%积分初值，初始化
integral_error2 = zeros(3, 1);
integral_error3 = zeros(3, 1);%积分初值，初始化
integral_error4 = zeros(3, 1);

max_step = 1000;
deg2rad = pi/180;

%变量初始化和历史记录
[p0_his, p1_his, v1_his, pd1_his, vd1_his, p2_his, v2_his, pd2_his, vd2_his, ...
 tau1_his, taur1_his, f1_his, a1_his, tau2_his, taur2_his, f2_his, a2_his]...
 = deal(zeros(3, max_step));

[p3_his, v3_his, pd3_his, vd3_his, p4_his, v4_his, pd4_his, vd4_his, ...
 tau3_his, taur3_his, f3_his, a3_his, tau4_his, taur4_his, f4_his, a4_his]...
 = deal(zeros(3, max_step));

length_his = zeros(8, max_step);    length_error_his = length_his;



global p0_init;
p0_init = zeros(3, 1);

global delta10 delta20 delta30 delta40;
global length_init;
% global length10 length12;
delta10 = [-10, 10, 15*deg2rad]'*2;
delta20 = [-10, -10, -15*deg2rad]'*2;
delta30 = [10, -10, -15*deg2rad]'*2;
delta40 = [10, 10, 15*deg2rad]'*2;
length_init = computeLength_Init();


p0 = p0_init;
p1 = p0 + delta10 ; %在初始位加入随机数
p2 = p0 + delta20;
p3 = p0 + delta30;
p4 = p0 + delta40;

v1 = zeros(3, 1);
v2 = zeros(3, 1);
v3 = zeros(3, 1);
v4 = zeros(3, 1);


% 间距转换为大地坐标系下
R0 = rotate_matrix(p0);
delta10_g = R0 * delta10;
delta20_g = R0 * delta20;
delta30_g = R0 * delta30;
delta40_g = R0 * delta40;




% 推力分配中，螺旋桨初始推力和转角
f1 = [100;100;100];
a1 = [-50;60;-100];
f2 = [100;100;100];
a2 = [-50;60;-100];
f3 = [100;100;100];
a3 = [-50;60;-100];
f4 = [100;100;100];
a4 = [-50;60;-100];

wn = 0.05; 
cd = 0.7;
Q = diag([10, 10, 10, 1, 1, 1]);
R = 1 * eye(3);



%% 主循环
%reference model的时间常数
% 表示一个时间常数会达到目标值的63%
T = 100; 
p10 = p1;


Detination = [20, 30, 20*deg2rad]';

for i = 1 : max_step
    [length, length_error] = distance_among_ships(p0, p1, p2, p3, p4);
    
    length_his(:, i) = length;
    length_error_his(:, i) = length_error;
    
    % reference model,领航者的位置
    t = i;
    [pd0, dot_pd0, ddot_pd0] = ref_model(T, t, p0_init, Detination);
    
    % 协同导航控制器
    [pd1, pd2, pd3, pd4, coop_error, K] = LQR_consensus_guidance(pd0, p1, p2,...
        p3, p4, v1, v2, v3, v4, wn, cd, Q, R);
    
    [dot_pd1, dot_pd2, dot_pd3, dot_pd4] = deal(zeros(3, 1));
    
    p0 = pd0;
    p0_his(:, i) = pd0;  
    p1_his(:, i) = transfer_deg(p1);
    v1_his(:, i) = v1;
    pd1_his(:, i) = transfer_deg(pd1);
    vd1_his(:, i) = dot_pd1;
    
    p2_his(:, i) = transfer_deg(p2);
    v2_his(:, i) = v2;
    pd2_his(:, i) = transfer_deg(pd2);
    vd2_his(:, i) = dot_pd2;
    
    p3_his(:, i) = transfer_deg(p3);
    v3_his(:, i) = v3;
    pd3_his(:, i) = transfer_deg(pd3);
    vd3_his(:, i) = dot_pd3;
    
    p4_his(:, i) = transfer_deg(p4);
    v4_his(:, i) = v4;
    pd4_his(:, i) = transfer_deg(pd4);
    vd4_his(:, i) = dot_pd4;
    
    
    [tau1, integral_error1] = pid_controller(pd1, dot_pd1, p1, v1, MM1, D1, integral_error1);
    [tau2, integral_error2] = pid_controller(pd2, dot_pd2, p2, v2, MM1, D1, integral_error2);
    [tau3, integral_error3] = pid_controller(pd3, dot_pd3, p3, v3, MM1, D1, integral_error3);
    [tau4, integral_error4] = pid_controller(pd4, dot_pd4, p4, v4, MM1, D1, integral_error4);
%     [tau1, s1, ei1] = bs_constroller(pd1, dot_pd1, ddot_pd1, p1, v1, MM1, D1, Kd , Ki, ei1);
%     [tau2, s2, ei2] = bs_constroller(pd2, dot_pd2, ddot_pd2, p2, v2, MM1, D1, Kd , Ki, ei2);
%     
%     [tauc1, tauc2] = coop_controller(s1, s2, Kc1, Kc2);
%     % 加入一致性协议
%     tau1 = tau1 + tauc1;    tau2 = tau2 + tauc2;
    
    [f1,df1,a1,da1,taur1,dtau1] = Thrust_Allocation(f1,a1,tau1);
    [f2,df2,a2,da2,taur2,dtau2] = Thrust_Allocation(f2,a2,tau2);
    [f3,df3,a3,da3,taur3,dtau3] = Thrust_Allocation(f3,a3,tau3);
    [f4,df4,a4,da4,taur4,dtau4] = Thrust_Allocation(f4,a4,tau4);
    
    [p1, v1] = ship_dynamic_solver(p1, v1, taur1, MM1, D1);
    [p2, v2] = ship_dynamic_solver(p2, v2, taur2, MM1, D1); 
    [p3, v3] = ship_dynamic_solver(p3, v3, taur3, MM1, D1);
    [p4, v4] = ship_dynamic_solver(p4, v4, taur4, MM1, D1); 
    
    tau1_his(:, i) = tau1;
    taur1_his(:, i) = taur1;
    tau2_his(:, i) = tau2;
    taur2_his(:, i) = taur2;
    tau3_his(:, i) = tau3;
    taur3_his(:, i) = taur3;
    tau4_his(:, i) = tau4;
    taur4_his(:, i) = taur4;
%     tauc1_his(:, i) = tauc1;
    f1_his(:, i) = f1;
    a1_his(:, i) = a1;
    f2_his(:, i) = f2;
    a2_his(:, i) = a2;
    f3_his(:, i) = f3;
    a3_his(:, i) = a3;
    f4_his(:, i) = f4;
    a4_his(:, i) = a4;
    
    
    coop_error1_his(:, i) = coop_error(:, 1);
    coop_error2_his(:, i) = coop_error(:, 2);
    coop_error3_his(:, i) = coop_error(:, 3);
    coop_error4_his(:, i) = coop_error(:, 4);
    
    display_motion_flag = 1;
    figure_plot4;
     
end







%% 旋转矩阵
function R= rotate_matrix(p)
    fai = p(3);
    R = [cos(fai), -sin(fai), 0;
        sin(fai), cos(fai), 0;
        0, 0, 1];
end


%% 多船的间距
function [length, length_error] = distance_among_ships(p0, p1, p2, p3, p4)
    
    global length_init;
%     length = zeros(8, 1);
    length01 = lengthij(p0, p1);
    length02 = lengthij(p0, p2);
    length03 = lengthij(p0, p3);
    length04 = lengthij(p0, p4);
    length12 = lengthij(p2, p1);
    length23 = lengthij(p2, p3);
    length14 = lengthij(p1, p4);
    length34 = lengthij(p3, p4);
    length = [length01, length02, length03, length04, length12, length14, length23, length34]';
    
    length_error = length - length_init;
    
    
    

end
%% 计算船舶之间的初始间距
function length_init = computeLength_Init()
    global delta10 delta20 delta30 delta40;
    length01_init = lengthij(zeros(3, 1), delta10);
    length02_init = lengthij(zeros(3, 1), delta20);
    length03_init = lengthij(zeros(3, 1), delta30);
    length04_init = lengthij(zeros(3, 1), delta40);
    length12_init = lengthij(delta20, delta10);
    length14_init = lengthij(delta40, delta10);
    length23_init = lengthij(delta30, delta20);
    length34_init = lengthij(delta40, delta30);
    length_init = [length01_init, length02_init, length03_init, length04_init,...
                    length12_init, length14_init, length23_init, length34_init]';

end

%% 计算两点间的长度
function length = lengthij(pi, pj)
    length = sqrt( (pi(1)-pj(1))^2 + (pi(2)-pj(2))^2 );
end

%% 将角度转化为（-pi, pi)
function p1 = transfer_deg(p)
    x = p(3);
    y = -sign(x) * pi + rem((x + sign(x) * pi), 2 * pi);
    p1 = [p(1), p(2), y]';
end

