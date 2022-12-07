clear all; clc;
tic
% addpath("C:\Users\jiang\Desktop\nlmpc\pdfmpc")
% df, da作为控制量，测试

%-------------------------------------------------------------------------------
% Definition of p_ode
%-------------------------------------------------------------------------------
p_ode.Nship = 2;
M1 = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
Ma1 = [7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5, 8.77e6];
MM1 = M1 + Ma1;
D1 = [MM1(1,1)/100, 0, 0; 0, MM1(2,2)/40, 0; 0, 0, MM1(3,3)/20];
p_ode.tau = 1;%步长
p_ode.rk_order = 4;%积分阶数

p00 = zeros(3, 1);  v00 = p00;
p10 = [0; 0; 0];    v10 = zeros(3, 1);
p20 = [0; 0; 0];    v20 = zeros(3, 1);
p_ode.x0 = [p10; v10; p20; v20]; %初值
delta10 = p10 - p00;
delta20 = p20 - p00;

u10 = [240, 120, 120, 0*pi/180, 180*pi/180, 180*pi/180]';
u20 = [240, 120, 120, 0*pi/180, 180*pi/180, 180*pi/180]';
% u10 = [50, 25, 25, 180*pi/180, 0*pi/180, 0*pi/180]';
% u20 = [50, 25, 25, 180*pi/180, 0*pi/180, 0*pi/180]';
u10 = zeros(6, 1);  u20 = u10;

p_ode.u0 = zeros(6 * p_ode.Nship, 1);
p_ode.u_last = [u10; u20];

p_ode.M = MM1;
p_ode.D = D1;
p_ode.thrust_config = [-5.02 0; 11.956 2.7; 11.956 -2.7];


%-------------------------------------------------------------------------------
% 环境载荷设置
%-------------------------------------------------------------------------------
wind_param = struct('wind_speed', 5, 'wind_angle',  30/180*pi);
current_param = struct('current_speed', 0.3, 'current_angle', 30/180*pi);
wave_param = struct('Hs', 2.8, 'Tp', 13.4, 'gamma', 3.3, 'env_dir', 30/180*pi, 'nfr', 100);

%-------------------------------------------------------------------------------
% 滤波器设置
%-------------------------------------------------------------------------------
%%%%%%%%%滤波器参数设置%%%%%%%%%%%%%
%船1 滤波数据初始化
state1 = zeros(15, 1);
state1(7:9, 1) = p10;
output1 = p10;
p1_hat = p10;
v1_hat = v10;

%船2 滤波数据初始化
state2 = zeros(15, 1);
state2(7:9, 1) = p20;
output2 = p20;
p2_hat = p20;
v2_hat = v20;
K4 = diag([1.4e4, 1.4e4, 2.5e6]);
K3 = 0.1 * K4;
wave_w0 = 2*pi/wave_param.Tp;
observer_paramters = struct('M', MM1, 'D', D1, 'G', zeros(3, 3), 'T', 1000, 'lambda', [0.1, 0.1, 0.1],...
    'w', [wave_w0, wave_w0, wave_w0], 'wc', 1.2*wave_w0, 'step_size', 1, 'K3', K3, 'K4', K4);


%-------------------------------------------------------------------------------
% Definition of p_uparam
%-------------------------------------------------------------------------------
p_uparam.nu = 6 * p_ode.Nship;
p_uparam.Np = 20;
p_uparam.Ifree = [1;2;3;10]; 
p_uparam.R = compute_R(p_uparam.Ifree, p_uparam.Np, p_uparam.nu);
p_uparam.np = size(p_uparam.R,2);
p_uparam.p = repmat(p_ode.u0, size(p_uparam.Ifree, 1), 1); %p为初值，u0*4


F_max = 500;%KN
a_max = 2*pi;
dF_max = 100;
da_max = 20/180*pi;
% 里面的repmat是多船

pmax_temp = [dF_max, dF_max, dF_max, da_max da_max, da_max,...
            2*dF_max, 2*dF_max, 2*dF_max, da_max da_max, da_max]';
pmin_temp = -pmax_temp;
p_uparam.pmin = repmat(pmin_temp, size(p_uparam.Ifree, 1), 1);
p_uparam.pmax = -p_uparam.pmin;
% p_uparam.pmin = repmat(repmat([-dF_max, -dF_max, -dF_max, -da_max -da_max, -da_max]', p_ode.Nship, 1), size(p_uparam.Ifree, 1), 1);
% p_uparam.pmax = -p_uparam.pmin;


% p_uparam.pmin = repmat(repmat([0, 0, 0, -2*pi -2*pi, -2*pi]', p_ode.Nship, 1), size(p_uparam.Ifree, 1), 1);
% p_uparam.pmax = repmat(repmat([F_max, F_max, F_max, 2*pi 2*pi, 2*pi]', p_ode.Nship, 1), size(p_uparam.Ifree, 1), 1);




%-------------------------------------------------------------------------------
% Definition of p_ocp
%-------------------------------------------------------------------------------
p_ocp.Q = diag(repmat([1e9;1e9;2e11;0;0;0], p_ode.Nship, 1));%多条船的增益系数一样
p_ocp.R = diag(repmat([1e6; 1e6; 1e6; 0; 0; 0]*1e0*1, p_ode.Nship, 1));
p_ocp.Rdu = diag(repmat([1; 1; 1; 1e0; 1e0; 1e0]*1e0*0, p_ode.Nship, 1));
p_ocp.xd_his = repmat([p10; v10; p20; v20]', p_uparam.Np, 1);%这里后续需要修改
p_ocp.F_max = F_max;
p_ocp.a_max = a_max;
p_ocp.u_last = p_ode.u_last;

% 导航参数设置
leader_detination = [200, 40, 30/180*pi]';
guidance_param = struct("T", 100, "p0_init", p00, "leader_destination", leader_detination,...
    "delta10", delta10, "delta20", delta20);

%-------------------------------------------------------------------------------
% Create the param strcurure 
%-------------------------------------------------------------------------------
[param, flag, message, teval] = create_solution(p_ode, p_uparam, p_ocp, 1);%1表示重新编译， 0表示不编译直接运行

%-------------------------------------------------------------------------------
% Closed-loop simulation 
%-------------------------------------------------------------------------------
tsim = 400;
param.Nev = 1300;%这个意义不太清楚，好像是迭代上限
subset=[1];
% rrd=zeros(ntsim,1);
param = update_trust_region_parameters(param,[1.00001, 0.999999]);
param.ode.rk_order=4;

[pd0_his, vd0_his,...
    p1_his, v1_his, pd1_his, vd1_his, tau1_his, f1_his, a1_his, p1_hat_his, v1_hat_his,...
    p2_his, v2_his, pd2_his, vd2_his, tau2_his, f2_his, a2_his, p2_hat_his, v2_hat_his,]...
    =deal(zeros(tsim+1, 3));
[df1_his, da1_his, df2_his, da2_his] = deal(zeros(tsim, 3));
p1_his(1, :) = p_ode.x0(1:3)';  v1_his(1, :) = p_ode.x0(4:6)';
p2_his(1, :) = p_ode.x0(7:9)';  v2_his(1, :) = p_ode.x0(10:12)';
f1_his(1, :) = p_ode.u_last(1:3)';  a1_his(1, :) = p_ode.u_last(4:6)';
f2_his(1, :) = p_ode.u_last(7:9)';  a2_his(1, :) = p_ode.u_last(10:12)';
p1_hat_his(1, :) = p_ode.x0(1:3)';  v1_hat_his(1, :) = p_ode.x0(4:6)';
p2_hat_his(1, :) = p_ode.x0(7:9)';  v2_his(1, :) = p_ode.x0(10:12)';
pd0_his(1, :) = p00';   vd0_his(1, :) = v00';  
pd1_his(1, :) = p10';   vd1_his(1, :) = v10'; 
pd2_his(1, :) = p20';   vd2_his(1, :) = v20'; 

[p1, v1, p2, v2] = deal(zeros(3, 1));

% 主模拟程序
for i=1:tsim
    i
    
    param.ode.u_last = param.ocp.u_last;
%     param=user_sim(tt,i,param);
    for j = 1 : p_uparam.Np
        [pd0, vd0, pd1, vd1, pd2, vd2] = leader_follower_guidance(i+j-1, guidance_param);
        pd1 = pd0; vd1 = vd0; pd2 = pd0; vd2 = vd0;
        vd1 = zeros(3,1); vd2 = vd1;
        param.ocp.xd_his(j, :) = [pd1; vd1; pd2; vd2]';          
    end
    
    pd0_his(i+1, :) = pd0';   vd0_his(i+1, :) = vd0';
    pd1_his(i+1, :) = pd1';   vd1_his(i+1, :) = vd1';
    pd2_his(i+1, :) = pd2';   vd2_his(i+1, :) = vd2';
    
    x = [p1; v1; p2; v2];
%     x_hat = [p1_hat; v1_hat; p2_hat; v2_hat];
    [param, u, ~, ~] = pdf_mpc(x, param);
    
    u_next = param.ocp.u_last + u;
    param.ocp.u_last = u_next; 
    u = u_next;
    
    
    f1_his(i+1, :) = u(1:3)';   a1_his(i+1, :) = u(4:6)';
    f2_his(i+1, :) = u(7:9)';   a2_his(i+1, :) = u(10:12)';
    df1_his(i, :) = f1_his(i+1, :) - f1_his(i, :);
    da1_his(i, :) = a1_his(i+1, :) - a1_his(i, :);
    df2_his(i, :) = f2_his(i+1, :) - f2_his(i, :);
    da2_his(i, :) = a2_his(i+1, :) - a2_his(i, :);
    tau1  = thrusters_configuration(u(4:6), p_ode.thrust_config) * u(1:3); %KN
    tau2  = thrusters_configuration(u(10:12), p_ode.thrust_config) * u(7:9);
    tau1_his(i+1, :) = tau1';   tau2_his(i+1, :) = tau2';
    
    current_time = i;
    [tau1_wind_tug, tau1_current_tug, tau1_wave1_tug, tau1_wave2_tug] = tau_env_tug_interface(p1_hat_his(i, :)', ...
        current_time, wind_param, current_param, wave_param);
    [tau2_wind_tug, tau2_current_tug, tau2_wave1_tug, tau2_wave2_tug] = tau_env_tug_interface(p2_hat_his(i, :)', ...
        current_time, wind_param, current_param, wave_param);
    tau1_env = tau1_wind_tug + tau1_current_tug + tau1_wave1_tug + tau1_wave2_tug;
    tau2_env = tau2_wind_tug + tau2_current_tug + tau2_wave1_tug + tau2_wave2_tug;
    
    [p1, v1] = ship_dynamic_solver(p1_his(i,:)', v1_his(i,:)', tau1 , MM1, D1);
    [p2, v2] = ship_dynamic_solver(p2_his(i,:)', v2_his(i,:)', tau2 , MM1, D1);
    p1_his(i+1, :) = p1';   v1_his(i+1, :) = v1';
    p2_his(i+1, :) = p2';   v2_his(i+1, :) = v2';
    
    
    
    [p1_hat, v1_hat, state1, output1] = nonlinear_passive_observer(p1, tau1, state1, output1, observer_paramters);
    [p2_hat, v2_hat, state2, output2] = nonlinear_passive_observer(p2, tau2, state2, output2, observer_paramters);
    p1_hat_his(i+1, :) = p1_hat';     v1_hat_his(i+1, :) = v1_hat';
    p2_hat_his(i+1, :) = p2_hat';     v2_hat_his(i+1, :) = v2_hat';
    
    motion_display_flag = 0;
    user_plot;

end


toc



%% 推力计算
function T = thrusters_configuration(a,L)
% function thruster_configuration is used to obtain the configuration
% matrix
N_Col = length(a);

if N_Col ~= length(L)
    error('the length of a and L do not match');
end

T = zeros(3,N_Col);   % initialization

for i = 1 : N_Col
    T(1,i) = cos(a(i));
    T(2,i) = sin(a(i));
    T(3,i) = L(i,1)*sin(a(i)) - L(i,2)*cos(a(i));
end

end