%% 基本数据
clear all; clc; clf; close all;
M0 = diag([5.3e7, 5.3e7, 7.5e10]);
Ma0 = diag([0.8e7, 3.3e7, 3.5662e10]);
MM0 = M0 + Ma0;
D0 = diag([2.44e6, 4.88e6, 2*3.89e9]);
M1 = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
Ma1 = [7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5, 8.77e6];
MM1 = M1 + Ma1;
D1 = [MM1(1,1)/100, 0, 0; 0, MM1(2,2)/40, 0; 0, 0, MM1(3,3)/20];

max_step = 1500;
deg2rad = pi/180;



global hull_x0 hull_y0 hull_x1 hull_y1;
hull_x0 = [-75, 75, 75, -75, -75];
hull_y0 = [50, 50, -50, -50, 50];
hull_x1 = [-16, 12, 16, 12, -16, -16];
hull_y1 = [6, 6, 0, -6, -6, 6];


%% 数据记录初始化
%变量初始化和历史记录
[p0_his, pd0_his, v0_his, vd0_his, p1_his, v1_his, pd1_his, vd1_his, p2_his, v2_his, pd2_his, vd2_his, ...
 tau1_his, taur1_his, f1_his, a1_his, tau2_his, taur2_his, f2_his, a2_his, pd0_ref_his, dot_pd0_ref_his]...
 = deal(zeros(3, max_step));

[p3_his, v3_his, pd3_his, vd3_his, p4_his, v4_his, pd4_his, vd4_his, ...
 tau3_his, taur3_his, f3_his, a3_his, tau4_his, taur4_his, f4_his, a4_his]...
 = deal(zeros(3, max_step));

[s1_his, s2_his, s3_his, s4_his] = deal(zeros(3, max_step));
[tauc1_his, tauc2_his, tauc3_his, tauc4_his] = deal(zeros(3, max_step));

[tau0_cable_his, tau0_cable1_his, tau0_cable2_his, tau0_cable3_his, tau0_cable4_his,...
    tau_tug1_cable_his, tau_tug2_cable_his, tau_tug3_cable_his, tau_tug4_cable_his]...
    = deal(zeros(3, max_step));

[cable_length_his, cable_angle_his, cable_force_his] = deal(zeros(4, max_step));

length_his = zeros(8, max_step);    length_error_his = length_his;



ei1 = zeros(3,1);%初始化
ei2 = zeros(3,1);%初始化
ei3 = zeros(3,1);%初始化
ei4 = zeros(3,1);%初始化


%% 基本数据设定
global delta10 delta20 delta30 delta40;
global length_init;
% global length10 length12;
% delta10 = [375-200, 50, 0*deg2rad]';
% delta20 = [-375+200, 50, 0*deg2rad]';
% delta30 = [-375+200, -50, 0*deg2rad]';
% delta40 = [375-200, -50, 0*deg2rad]';
delta10 = [375, 350, 0*deg2rad]';
delta20 = [-375, 350, 0*deg2rad]';
delta30 = [-375, -350, 0*deg2rad]';
delta40 = [375, -350, 0*deg2rad]';
length_init = computeLength_Init();

global p0_init;
p0_init = zeros(3, 1);
p0 = p0_init;
v0 = zeros(3, 1);
p1 = p0 + delta10 ; %在初始位加入随机数 + [rand(2, 1); 0];
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
a2 = [100;60;-30];
f3 = [100;100;100];
a3 = [100;60;-10];
f4 = [100;100;100];
a4 = [-50;60;-100];


%reference model的时间常数
% 表示一个时间常数会达到目标值的63%
T = 200; 
p10 = p1;

Detination = [800, 100, 40*deg2rad]';

Kd = diag([20, 20, 2])*5;
Ki = diag([1, 1, 0.05])*1;

Kc1 = diag([20, 20, 2])*1;
Kc2 = Kc1;  Kc3 = Kc1;  Kc4 = Kc1;

% leader guidance pid 参数
ei_leader = zeros(3, 1);
pd0_ref = p0_init;
kp_ref = diag([0.8, 0.5, 0.8])*0.05;
kd_ref = diag([1, 1.5, 1])*0.03;
ki_ref = diag([1, 1, 1])*0.001;

% kp_ref = diag([1.5, 1.5, 0.6])*1.1;
% kd_ref = diag([1.0, 1.0, 1.5])*0.3;
% ki_ref = diag([1, 1, 1])*0.010;

pd0 =  p0_init;
dot_pd0 = zeros(3, 1);
Kcable = 200;
cable_length0 = [1, 1, 1, 1]*420;
[tau0_cable, tau_tug_cable, cable_length, cable_angle, cable_force0] = cable_force(p0,...
        p1, p2, p3, p4, Kcable, cable_length0);

%% 主循环


for i = 1 : max_step
% for i = 1 : 100
    i
    [length, length_error] = distance_among_ships(p0, p1, p2, p3, p4);
    
    length_his(:, i) = length;
    length_error_his(:, i) = length_error;
    
    % reference model,领航者的位置
%     t = i
%     [pd0, dot_pd0, ddot_pd0] = ref_model(T, t, p0_init, Detination);
    wn = [0.01, 0.01, 0.005]';
    eta = [0.9, 0.9, 1.2]';
    
    
%     wn = [0.017, 0.01, 0.01]';
%     eta = [0.9, 0.9, 1.2]';
    time_step = 1;
    vmax = [4, 4, 1/180*pi/time_step]';
    [pd0, dot_pd0, ddot_pd0] = ref_model_2ord(Detination, pd0, dot_pd0, wn, eta, vmax, time_step);
    
    [pd0_ref, dot_pd0_ref, ddot_pd0_ref, ei_leader] = leader_guiddance_pid_controller(p0, v0, pd0, dot_pd0, pd0_ref,...
                                                        kp_ref, kd_ref, ki_ref, time_step, ei_leader);
                                                    
     pd0_ref_his(:, i) = pd0_ref;
     dot_pd0_ref_his(:, i) = dot_pd0_ref;
%     dot_pd0_ref = zeros(3, 1);                                                
%     pd0 = transfer_deg(pd0);
    [pd1, pd2, pd3, pd4, dot_pd1, dot_pd2, dot_pd3, dot_pd4,...
                ddot_pd1, ddot_pd2, ddot_pd3, ddot_pd4] = guidance_followers(pd0_ref, dot_pd0_ref);
    % 间距转换为大地坐标系下
    R0 = rotate_matrix(p0);

    
    p0_his(:, i) = p0;
    v0_his(:, i) = v0;
    pd0_his(:, i) = pd0;
    vd0_his(:, i) = dot_pd0;
    
    p1_his(:, i) = p1;
    v1_his(:, i) = v1;
    pd1_his(:, i) = pd1;
    vd1_his(:, i) = dot_pd1;
    
    p2_his(:, i) = p2;
    v2_his(:, i) = v2;
    pd2_his(:, i) = pd2;
    vd2_his(:, i) = dot_pd2;
    
    p3_his(:, i) = p3;
    v3_his(:, i) = v3;
    pd3_his(:, i) = pd3;
    vd3_his(:, i) = dot_pd3;
    
    p4_his(:, i) = p4;
    v4_his(:, i) = v4;
    pd4_his(:, i) = pd4;
    vd4_his(:, i) = dot_pd4;
    
    [tau1, s1, ei1] = bs_constroller(pd1, dot_pd1, ddot_pd1, p1, v1, MM1, D1, Kd , Ki, ei1);
    [tau2, s2, ei2] = bs_constroller(pd2, dot_pd2, ddot_pd2, p2, v2, MM1, D1, Kd , Ki, ei2);
    [tau3, s3, ei3] = bs_constroller(pd3, dot_pd3, ddot_pd3, p3, v3, MM1, D1, Kd , Ki, ei3);
    [tau4, s4, ei4] = bs_constroller(pd4, dot_pd4, ddot_pd4, p4, v4, MM1, D1, Kd , Ki, ei4);
    
    [tauc1, tauc2, tauc3, tauc4] = coop_controller(s1, s2, s3, s4, Kc1, Kc2, Kc3, Kc4);
    
    % 加入一致性协议和锚链力前馈
    tau1 = tau1 + tauc1 - tau_tug_cable(:, 1);    
    tau2 = tau2 + tauc2 - tau_tug_cable(:, 2);
    tau3 = tau3 + tauc3 - tau_tug_cable(:, 3);    
    tau4 = tau4 + tauc4 - tau_tug_cable(:, 4);
    
    [f1,df1,a1,da1,taur1,dtau1] = Thrust_Allocation(f1,a1,tau1);
    [f2,df2,a2,da2,taur2,dtau2] = Thrust_Allocation(f2,a2,tau2);
    [f3,df3,a3,da3,taur3,dtau3] = Thrust_Allocation(f3,a3,tau3);
    [f4,df4,a4,da4,taur4,dtau4] = Thrust_Allocation(f4,a4,tau4);
    
    [p1, v1] = ship_dynamic_solver(p1, v1, taur1 + tau_tug_cable(:, 1), MM1, D1);
    [p2, v2] = ship_dynamic_solver(p2, v2, taur2 + tau_tug_cable(:, 2), MM1, D1); 
    [p3, v3] = ship_dynamic_solver(p3, v3, taur3 + tau_tug_cable(:, 3), MM1, D1);
    [p4, v4] = ship_dynamic_solver(p4, v4, taur4 + tau_tug_cable(:, 4), MM1, D1); 
    
    [tau0_cable, tau_tug_cable, cable_length, cable_angle, cable_force0] = cable_force(p0,...
        p1, p2, p3, p4, Kcable, cable_length0);
    
   
    
    tau0_cable1_his(:, i) = tau0_cable(:, 1);
    tau0_cable2_his(:, i) = tau0_cable(:, 2);
    tau0_cable3_his(:, i) = tau0_cable(:, 3);
    tau0_cable4_his(:, i) = tau0_cable(:, 4);
    tau0_cable_his(:, i) = tau0_cable(:, 5);
    tau_tug1_cable_his(:, i) = tau_tug_cable(:, 1);
    tau_tug2_cable_his(:, i) = tau_tug_cable(:, 2);
    tau_tug3_cable_his(:, i) = tau_tug_cable(:, 3);
    tau_tug4_cable_his(:, i) = tau_tug_cable(:, 4);
    for j = 1 : 4 
        cable_angle(j) = transfer_deg1(cable_angle(j));
    end
    
    cable_length_his(:, i) = cable_length;
    cable_angle_his(:, i) = cable_angle;
    cable_force_his(:, i) = cable_force0;
    
    tau0 = tau0_cable(:, end);
    [p0, v0] = ship_dynamic_solver(p0, v0, tau0, MM0, D0);
    
    tau1_his(:, i) = tau1;
    taur1_his(:, i) = taur1;
    tauc1_his(:, i) = tauc1;
    f1_his(:, i) = f1;
    a1_his(:, i) = a1;
    s1_his(:, i) = s1;
    
    tau2_his(:, i) = tau2;
    taur2_his(:, i) = taur2;
    tauc2_his(:, i) = tauc2;
    f2_his(:, i) = f2;
    a2_his(:, i) = a2;
    s2_his(:, i) = s2;
    
    tau3_his(:, i) = tau3;
    taur3_his(:, i) = taur3;
    tauc3_his(:, i) = tauc3;
    f3_his(:, i) = f3;
    a3_his(:, i) = a3;
    s3_his(:, i) = s3;
    
    tau4_his(:, i) = tau4;
    taur4_his(:, i) = taur4;
    tauc4_his(:, i) = tauc4;
    f4_his(:, i) = f4;
    a4_his(:, i) = a4;
    s4_his(:, i) = s4;
    
    display_motion_flag = 1;
    figure_plot4;
    
end



figure(6)
subplot(3, 1, 1)
plot(pd0_his(1, :));
hold on
plot(pd0_ref_his(1, :));
hold on
plot(p0_his(1, :));
hold off
legend('pd0', 'pd0_{ref}', 'p0')
subplot(3, 1, 2)
plot(pd0_his(2, :));
hold on
plot(pd0_ref_his(2, :));
hold on
plot(p0_his(2, :));
hold off
subplot(3, 1, 3)
plot(pd0_his(3, :)*180/pi);
hold on
plot(pd0_ref_his(3, :)*180/pi);
hold on
plot(p0_his(3, :)*180/pi);
hold off













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

%% 计算两点间的长度
function length = lengthij(pi, pj)
    length = sqrt( (pi(1)-pj(1))^2 + (pi(2)-pj(2))^2 );
end

%% 旋转矩阵
function R= rotate_matrix(p)
    fai = p(3);
    R = [cos(fai), -sin(fai), 0;
        sin(fai), cos(fai), 0;
        0, 0, 1];
end

%% 将角度转化为（-pi, pi)
function p1 = transfer_deg(p)
    x = p(3);
    y = -sign(x) * pi + rem((x + sign(x) * pi), 2 * pi);
    p1 = [p(1), p(2), y]';
end

function fai1 = transfer_deg1(fai)
    fai1 = -sign(fai) * pi + rem((fai + sign(fai) * pi), 2 * pi);
end


