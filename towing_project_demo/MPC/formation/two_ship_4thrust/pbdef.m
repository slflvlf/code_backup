clear all; clc;

addpath("C:\Users\jiang\Desktop\towing_project_demo\MPC\pdfmpc")

%-------------------------------------------------------------------------------
% Definition of p_ode
%-------------------------------------------------------------------------------
p_ode.Nship = 2;

M1 = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
Ma1 = [7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5, 8.77e6];
MM1 = M1 + Ma1;
D1 = [MM1(1,1)/100, 0, 0; 0, MM1(2,2)/40, 0; 0, 0, MM1(3,3)/20];
p_ode.tau=1;%步长
p_ode.rk_order=4;%积分阶数


p_ode.M = MM1;
p_ode.D = D1;
p_ode.thrust_config = [-11.956 -2.7; -11.956 2.7; 11.956 2.7; 11.956 -2.7];

p00 = zeros(3, 1);  v00 = p00;
p10 = [0; 0; 0];    v10 = zeros(3, 1);
p20 = [0; 0; 0];    v20 = zeros(3, 1);
p_ode.x0 = [p10; v10; p20; v20]; %初值
u10 = [100, 100, 100, 100, 180*pi/180, 180*pi/180, 0*pi/180, 0*pi/180]';
u20 = [100, 100, 100, 100, 180*pi/180, 180*pi/180, 0*pi/180, 0*pi/180]';
p_ode.u0=[u10; u20];

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
state = zeros(15, 1);
state(7:9, 1) = p10;
output = p10;
p_hat = p10;
v_hat = v10;
K4 = diag([1.4e4, 1.4e4, 2.5e6]);
K3 = 0.1 * K4;
wave_w0 = 2*pi/wave_param.Tp;
observer_paramters = struct('M', MM1, 'D', D1, 'G', zeros(3, 3), 'T', 1000, 'lambda', [0.1, 0.1, 0.1],...
    'w', [wave_w0, wave_w0, wave_w0], 'wc', 1.2*wave_w0, 'step_size', 1, 'K3', K3, 'K4', K4);


%-------------------------------------------------------------------------------
% Definition of p_uparam
%-------------------------------------------------------------------------------
p_uparam.nu = 8 * p_ode.Nship;
p_uparam.Np = 20;
p_uparam.Ifree = [1;2;3;10;];
p_uparam.R = compute_R(p_uparam.Ifree, p_uparam.Np, p_uparam.nu);
p_uparam.np = size(p_uparam.R,2);
p_uparam.p = repmat(p_ode.u0, size(p_uparam.Ifree, 1), 1);
% p_uparam.p=zeros(p_uparam.np,1);

F_max = 400;%KN
dF_max = 100;
da_max = 20/180*pi;
% p_uparam.pmin = reshape([0, 0, 0, -2*pi -2*pi, -2*pi]'*ones(1,size(p_uparam.Ifree, 1)),p_uparam.np,1);
p_uparam.pmin=repmat(repmat([0, 0, 0, 0, -2*pi -2*pi, -2*pi, -2*pi]', p_ode.Nship, 1), size(p_uparam.Ifree, 1), 1);
p_uparam.pmax=repmat(repmat([F_max, F_max, F_max, F_max, 2*pi 2*pi, 2*pi, 2*pi]', p_ode.Nship, 1), size(p_uparam.Ifree, 1), 1);



%-------------------------------------------------------------------------------
% Definition of p_ocp
%-------------------------------------------------------------------------------
p_ocp.Q=diag(repmat([1e8;1e8;2e10;1e1;1e1;1e1], p_ode.Nship, 1));
p_ocp.R=diag(repmat([1e5; 1e5; 1e5; 1e5; 0; 0; 0; 0]*1, p_ode.Nship, 1));
p_ocp.Rdu=diag(repmat([1e4; 1e4; 1e4; 1e4; 1e4; 1e4; 1e4; 1e4]*1e-4, p_ode.Nship, 1));
p_ocp.rd=[2, 1, 30/180*pi]';
p_ocp.dF_max = dF_max;
p_ocp.da_max = da_max;

p_ocp.rd_his = zeros(p_uparam.Np, 3);
p_ocp.u_last = p_ode.u0;
% p_ocp.theta_max=0.0015;
% p_ocp.thetap_max=2*pi/30;

%-------------------------------------------------------------------------------
% Create the param strcurure 
%-------------------------------------------------------------------------------
[param,flag,message,teval]=create_solution(p_ode,p_uparam,p_ocp,0);%1表示重新编译， 0表示不编译直接运行
%-------------------------------------------------------------------------------
% Closed-loop simulation
%-------------------------------------------------------------------------------
tsim=400;param.Nev=1300;
[tt,xx,uu,tt_exec,ntsim]=initialize(tsim,param);
rrd=zeros(ntsim,3);
param=update_trust_region_parameters(param,[2,0.5]);
param.ode.rk_order=4;

[p1_his, v1_his, pd1_his, vd1_his, tau1_his,  p1_hat_his, v1_hat_his,...
    p2_his, v2_his, pd2_his, vd2_his, tau2_his, p2_hat_his, v2_hat_his,]...
    =deal(zeros(length(tt), 3));
[f1_his, a1_his, f2_his, a2_his] = deal(zeros(length(tt), 4));
[df1_his, da1_his, df2_his, da2_his] = deal(zeros(length(tt)-1, 4));
[p1, v1, p2, v2] = deal(zeros(3, 1));
p1_his(1, :) = p_ode.x0(1:3)';
v1_his(1, :) = p_ode.x0(4:6)';
p2_his(1, :) = p_ode.x0(7:9)';
v2_his(1, :) = p_ode.x0(10:12)';
f1_his(1, :) = p_ode.u0(1:4)';
a1_his(1, :) = p_ode.u0(5:8)';
f2_his(1, :) = p_ode.u0(9:12)';
a2_his(1, :) = p_ode.u0(13:16)';
p_hat_his(1, :) = p_ode.x0(1:3)';
v_hat_his(1, :) = p_ode.x0(4:6)';
T = 40;
Rr = [100, 40, 30/180*pi]';
p0 = zeros(3, 1);
rd_his = zeros(p_uparam.Np, 3);
for i=1:length(tt)-1
    disp(i/ntsim);
%     param=user_sim(tt,i,param);
    [pd, dot_pd, ddot_pd] = ref_model(T, i, p0, Rr);
    param.ocp.rd = pd;
%     for j = 1 : p_uparam.Np
%         [pd, dot_pd, ddot_pd] = ref_model(T, i+j-1, p0, Rr);
%         rd_his(j, :) = pd';           
%         param.ocp.rd_his = rd_his;
%     end
%     x_hat = [p_hat; v_hat];
    
    x = [p1; v1; p2; v2];
    [param,u,u_sol,tt_exec(i)]=pdf_mpc(x, param);
    uu(i,:)=u';
    rrd(i, :)=param.ocp.rd_his(1, :);
    param.ocp.u_last = u;
    f1_his(i+1, :) = u(1:4)';
    a1_his(i+1, :) = u(5:8)';
    f2_his(i+1, :) = u(9:12)';
    a2_his(i+1, :) = u(13:16)';
    df1_his(i, :) = f1_his(i+1, :) - f1_his(i, :);
    da1_his(i, :) = a1_his(i+1, :) - a1_his(i, :);
    df2_his(i, :) = f2_his(i+1, :) - f2_his(i, :);
    da2_his(i, :) = a2_his(i+1, :) - a2_his(i, :);
    tau1  = thrusters_configuration(u(5:8), p_ode.thrust_config) * u(1:4); %KN
    tau1_his(i+1, :) = tau1';
    tau2  = thrusters_configuration(u(13:16), p_ode.thrust_config) * u(9:12); %KN
    tau2_his(i+1, :) = tau2';
    
%     current_time = i;
%     [tau_wind_tug, tau_current_tug, tau_wave1_tug, tau_wave2_tug] = tau_env_tug_interface(p_hat_his(i, :)', current_time, wind_param,... 
%             current_param, wave_param);
%     tau_env = tau_wind_tug + tau_current_tug + tau_wave1_tug + tau_wave2_tug;
    
    [p1, v1] = ship_dynamic_solver(p1_his(i,:)', v1_his(i,:)', tau1, MM1, D1);
    [p2, v2] = ship_dynamic_solver(p2_his(i,:)', v2_his(i,:)', tau2, MM1, D1);
    p1_his(i+1, :) = p1';
    v1_his(i+1, :) = v1';
    p2_his(i+1, :) = p2';
    v2_his(i+1, :) = v2';
    
%     [p_hat, v_hat, state, output] = nonlinear_passive_observer(p, tau, state, output, observer_paramters);
%     p_hat_his(i+1, :) = p_hat';
%     v_hat_his(i+1, :) = v_hat';
    
%     pd_his(i+1, :) = rrd(i, :);
   motion_display_flag = 0;
    user_plot;
end






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