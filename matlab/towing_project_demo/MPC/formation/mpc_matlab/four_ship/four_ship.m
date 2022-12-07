clear all;  clc;   close all;
tic
%% 船舶基本参数
M1 = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
Ma1 = [7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5, 8.77e6];
MM1 = M1 + Ma1;
D1 = [MM1(1,1)/100, 0, 0; 0, MM1(2,2)/40, 0; 0, 0, MM1(3,3)/20];
L = [-5.02 0; 11.956 2.7; 11.956 -2.7];

Fmax = 400;
dFmax = 100;
damax = 20/180*pi;
% 船舶初始值
p0 = [3; 4.4; deg2rad(-20)];    v0 = [0; 0; 0];     u0 = [10 5 5 pi 0 0]';
p1 = [3; 4.4; deg2rad(-20)];    v1 = [0; 0; 0];     u1 = [10 5 5 pi 0 0]';
p2 = [3; 4.4; deg2rad(-20)];    v2 = [0; 0; 0];     u2 = [10 5 5 pi 0 0]';
p3 = [3; 4.4; deg2rad(-20)];    v3 = [0; 0; 0];     u3 = [10 5 5 pi 0 0]';
p4 = [3; 4.4; deg2rad(-20)];    v4 = [0; 0; 0];     u4 = [10 5 5 pi 0 0]';

%船舶间距
delta10 = p1 - p0;  delta20 = p2 - p0;  delta30 = p3 - p0;  delta40 = p4 - p0;
%% 生成运动方程和相应的雅可比矩阵
% 运行一次就行，符号运算偏慢；runFlag = 1;

setFourShipStateAndJacobian(MM1, D1, L , 0);


%% MPC 控制器的定义

Tsim = 50;
obstacleExist = 0;%1 存在障碍物 0 不存在障碍物

Nship = 4;

Np = 8;
Nc = 3;
Ts = 1;

nx = 6*Nship;
ny = 6*Nship;
nu = 6*Nship;

% 实例nlmpc类，其中输入量为11个，工具箱认为可测扰动MD也算输入，
% 不过计算的时候只计算操纵变量MV， 定义输入约束的时候也只是针对MV，
nlobj = nlmpc(nx, ny, nu);


nlobj.Model.StateFcn = "FourShipStateFcn";
nlobj.Jacobian.StateFcn = "FourShipStateJacobianFcn";


nlobj.Ts = Ts;
nlobj.PredictionHorizon = Np;
nlobj.ControlHorizon = Nc;

% rng(0)
% % 验证nlmpc
% validateFcns(nlobj,rand(nx,1),rand(nu,1));


nlobj.MV = struct('Min',repmat({0; 0; 0; -2*pi; -2*pi; -2*pi}, Nship, 1),...
    'Max',repmat({Fmax; Fmax; Fmax; 2*pi; 2*pi; 2*pi}, Nship, 1),...
    'RateMin', repmat({-dFmax; -dFmax; -dFmax; -damax; -damax; -damax}, Nship, 1),...
    'RateMax', repmat({dFmax; dFmax; dFmax; damax; damax; damax}, Nship, 1));



nlobj.Weights.OutputVariables = repmat([250 250 250 0 0 0]*50, 1, Nship);
% nlobj.Weights.OutputVariables = [250 250 250];

% nlobj.Weights.ManipulatedVariables =[1 1 1 5 5 5];
nlobj.Weights.ManipulatedVariables = repmat([1 1 1 0 0 0], 1, Nship);

% nlobj.Weights.ManipulatedVariablesRate = [50 50 50 300 300 300]*0;

% mpc步进计算函数
nloptions = nlmpcmoveopt;

%% 环境载荷参数
wind_param = struct('wind_speed', 5, 'wind_angle',  30/180*pi);
current_param = struct('current_speed', 0.3, 'current_angle', 30/180*pi);
wave_param = struct('Hs', 2.8, 'Tp', 13.4, 'gamma', 3.3, 'env_dir', 30/180*pi, 'nfr', 100);


%% 滤波器设置
state0 = zeros(15, 1); state0(7:9, 1) = p0; output0 = p0; p0_hat = p0; v0_hat = v0;
state1 = zeros(15, 1); state1(7:9, 1) = p1; output1 = p1; p1_hat = p1; v1_hat = v1;
state2 = zeros(15, 1); state2(7:9, 1) = p2; output2 = p2; p2_hat = p2; v2_hat = v2;
state3 = zeros(15, 1); state3(7:9, 1) = p3; output3 = p3; p3_hat = p3; v3_hat = v3;
state4 = zeros(15, 1); state4(7:9, 1) = p4; output4 = p4; p4_hat = p4; v4_hat = v4;

K4 = diag([1.4e4, 1.4e4, 2.5e6]);
K3 = 0.1 * K4;
wave_w0 = 2*pi/wave_param.Tp;
observer_paramters = struct('M', MM1, 'D', D1, 'G', zeros(3, 3), 'T', 1000, 'lambda', [0.1, 0.1, 0.1],...
    'w', [wave_w0, wave_w0, wave_w0], 'wc', 1.2*wave_w0, 'step_size', 1, 'K3', K3, 'K4', K4);


%% 参考轨迹
%situation map
load ref_map_3 %间隔0.1m 缩尺比1:500 宽75m变成30m
yy = 1300 - yy;%像素反转
xx = xx * 30 / 1900;%从像素转换到坐标 m(缩尺比1:200 宽75m）
yy = yy * (30 * 13 / 19) / 1300;
r = zeros(3, size(xx,2));
r(1, :) = xx;
r(2,:) = yy;
for i = 1 : size(r, 2)
    if i == 1
        r(3, i) = deg2rad(-20); %起点
    elseif i == size(r, 2)
        r(3,i) = deg2rad(-44.5); %终点
    else
        r(3, i) = atan( (r(2, i+1) - r(2, i-1)) / (r(1, i+1) - r(1, i-1)) );%单位 rad
    end
end


%% 船舶初始值
%船舶初始状态
% ref_map_3的初始值

x = [ p1; v1; p2; v2; p3; v3; p4; v4];
x_hat = [p1_hat; v1_hat; p2_hat; v2_hat; p3_hat; v3_hat; p4_hat; v4_hat; ];
u = [u1; u2; u3; u4];

x_his = zeros(Tsim+1, nx);  x_hat_his = x_his;
u_his = zeros(Tsim+1, nu);  du_his = zeros(Tsim, nu);
Ref_his = zeros(Tsim, nu);
x_his(1, :) = x';    u_his(1, :) = u';  
x_hat_his(1, :) = x_hat';


[coreData, onlineData] = getCodeGenerationData(nlobj, x_hat, u);


%% 主循环


for k = 1:Tsim
    k

    %找到leader前方最近的参考路径点序列
    
    

    index = findNearestPoint(p1_hat(1:2),r(1:2,:));
    index_aft = index + Np - 1;

    if index_aft > length(r(1, :))
        Yd = [r(:, index : end), repmat(r(:, end), 1, index_aft - length(r(1, :)) )];
    else
        Yd = r(:,index:index+Np-1);
    end
    

    Yd = [Yd', zeros(Np, 3)];

    
    onlineData.ref = repmat(Yd, 1, Nship);
    ref = onlineData.ref;

    Ref_his(k, :) = onlineData.ref(1, :);

%     [u, onlineData] = nlmpcmoveCodeGeneration(coreData, x_hat, u, onlineData);
%     [u, onlineData, info] = nlmpcmoveMEX(coreData, x_hat, u, onlineData);
    [u, nloptions, info] = nlmpcmove(nlobj, x_hat, u, ref, nloptions);

    u_his(k+1, :) = u';   
    du_his(k, :) = u_his(k+1, :) - u_his(k, :);

    [tau0_wind, tau0_current, tau0_wave1, tau0_wave2] = tau_env_tug_interface(p0_hat, k, wind_param,... 
            current_param, wave_param);
    [tau1_wind, tau1_current, tau1_wave1, tau1_wave2] = tau_env_tug_interface(p1_hat, k, wind_param,... 
            current_param, wave_param);
    [tau2_wind, tau2_current, tau2_wave1, tau2_wave2] = tau_env_tug_interface(p2_hat, k, wind_param,... 
            current_param, wave_param);
    [tau3_wind, tau3_current, tau3_wave1, tau3_wave2] = tau_env_tug_interface(p3_hat, k, wind_param,... 
            current_param, wave_param);
    [tau4_wind, tau4_current, tau4_wave1, tau4_wave2] = tau_env_tug_interface(p4_hat, k, wind_param,... 
            current_param, wave_param);
    tau0_env = tau0_wind + tau0_current + tau0_wave1 + tau0_wave2;
    tau1_env = tau1_wind + tau1_current + tau1_wave1 + tau1_wave2;
    tau2_env = tau2_wind + tau2_current + tau2_wave1 + tau2_wave2;
    tau3_env = tau3_wind + tau3_current + tau3_wave1 + tau3_wave2;
    tau4_env = tau4_wind + tau4_current + tau4_wave1 + tau4_wave2;

    Tenv = 30;
    if k < Tenv
        tau0_env = k / Tenv * tau0_env;
        tau1_env = k / Tenv * tau1_env;     tau2_env = k / Tenv * tau2_env;
        tau3_env = k / Tenv * tau3_env;     tau4_env = k / Tenv * tau4_env;
    end
    
    tau1 = thrusters_configuration(u(4:6), L) * u(1:3);
    tau2 = thrusters_configuration(u(10:12), L) * u(7:9);
    tau3 = thrusters_configuration(u(16:18), L) * u(13:15);
    tau4 = thrusters_configuration(u(22:24), L) * u(19:21);

    [p1, v1] = ship_dynamic_solver(p1, v1, tau1+tau1_env, MM1, D1);
    [p2, v2] = ship_dynamic_solver(p2, v2, tau2+tau2_env, MM1, D1);
    [p3, v3] = ship_dynamic_solver(p3, v3, tau3+tau3_env, MM1, D1);
    [p4, v4] = ship_dynamic_solver(p4, v4, tau4+tau4_env, MM1, D1);

    x = [ p1; v1; p2; v2; p3; v3; p4; v4];
    x_his(k+1, :) = x';

%     [p0_hat, v0_hat, state0, output0] = nonlinear_passive_observer(p0, tau0, state0, output0, observer_paramters);
    [p1_hat, v1_hat, state1, output1] = nonlinear_passive_observer(p1, tau1, state1, output1, observer_paramters);
    [p2_hat, v2_hat, state2, output2] = nonlinear_passive_observer(p2, tau2, state2, output2, observer_paramters);
    [p3_hat, v3_hat, state3, output3] = nonlinear_passive_observer(p3, tau3, state3, output3, observer_paramters);
    [p4_hat, v4_hat, state4, output4] = nonlinear_passive_observer(p4, tau4, state4, output4, observer_paramters);
    
    x_hat = [p1_hat; v1_hat; p2_hat; v2_hat; p3_hat; v3_hat; p4_hat; v4_hat; ];
    x_hat_his(k+1, :) = x_hat';



end

compute_time = toc

figure(1)
subplot(2, 2, 1)
plot(x_hat_his(:, 1), x_hat_his(:, 2))
hold on 
plot(x_his(:, 1), x_his(:, 2))
hold on 
plot(Ref_his(:, 1), Ref_his(:, 2))
hold off
legend('x_hat', 'x', 'r')
legend boxoff
title('循迹效果');
subplot(2, 2, 2)
plot(x_hat_his(:, 7), x_hat_his(:, 8))
hold on 
plot(x_his(:, 7), x_his(:, 8))
hold on 
plot(Ref_his(:, 1), Ref_his(:, 2))
hold off
subplot(2, 2, 3)
plot(x_hat_his(:, 13), x_hat_his(:, 14))
hold on 
plot(x_his(:, 13), x_his(:, 14))
hold on 
plot(Ref_his(:, 1), Ref_his(:, 2))
hold off
subplot(2, 2, 4)
plot(x_hat_his(:, 19), x_hat_his(:, 20))
hold on 
plot(x_his(:, 19), x_his(:, 20))
hold on 
plot(Ref_his(:, 1), Ref_his(:, 2))
hold off


% figure(2)
% subplot(3, 2, 1); plot(x0_his(:, 1)); hold on; plot(x0_hat_his(:, 1)); hold on; plot(pd0_his(:, 1)); hold off; legend('p0', 'p0_{hat}', 'pd'); legend boxoff
% subplot(3, 2, 3); plot(x0_his(:, 2)); hold on; plot(x0_hat_his(:, 2)); hold on; plot(pd0_his(:, 2)); hold off;
% subplot(3, 2, 5); plot(x0_his(:, 3)/pi*180); hold on; plot(x0_hat_his(:, 3)/pi*180); hold on; plot(pd0_his(:, 3)/pi*180); hold off;
% subplot(3, 2, 2); plot(x0_his(:, 4)); hold on; plot(x0_hat_his(:, 4)); hold off; legend('v0', 'v0_{hat}'); legend boxoff
% subplot(3, 2, 4); plot(x0_his(:, 5)); hold on; plot(x0_hat_his(:, 5)); hold off;
% subplot(3, 2, 6); plot(x0_his(:, 6)/pi*180); hold on; plot(x0_hat_his(:, 6)/pi*180); hold off;
% 
% 
% figure(3)
% subplot(2, 2, 1); plot(u0_his(:, 1:3)); legend("f1", 'f2', 'f3'); legend boxoff; title('f');
% subplot(2, 2, 3); plot(u0_his(:, 4:6)/pi*180); legend("a1", 'a2', 'a3'); legend boxoff; title('a');
% subplot(2, 2, 2); plot(du_his(:, 1:3)); legend("df1", 'df2', 'df3'); legend boxoff; title('df');
% subplot(2, 2, 4); plot(du_his(:, 4:6)/pi*180); legend("da1", 'da2', 'da3'); legend boxoff; title('da');





%% mex文件生成
% func = 'nlmpcmoveCodeGeneration';
% funcOutput = 'nlmpcmoveMEX';
% Cfg = coder.config('mex');
% Cfg.DynamicMemoryAllocation = 'off';
% codegen('-config',Cfg,func,'-o',funcOutput,'-args', {coder.Constant(coreData), x_hat, u, onlineData});

% CfgGPU = coder.gpuConfig('mex');
% CfgGPU.TargetLang = 'C++';
% % CfgGPU.EnableVariableSizing = false;
% % CfgCPU.ConstantInputs = 'IgnoreValues';
% codegen('-config',CfgGPU,'nlmpcmoveCodeGeneration','-o','mympcmoveGPU','-args',{coder.Constant(coreData), x_hat, u, onlineData});

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






