clear all;  clc;   close all;
tic
%% 船舶基本参数
M1 = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
Ma1 = [7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5, 8.77e6];
MM1 = M1 + Ma1;
D1 = [MM1(1,1)/100, 0, 0; 0, MM1(2,2)/40, 0; 0, 0, MM1(3,3)/20];
L = [-5.02 0; 11.956 2.7; 11.956 -2.7];

Fmax = 400;
dFmax = 50;
damax = 20/180*pi;
% 船舶初始值
p0 = [3; 4.4; deg2rad(-20)];
v0 = [0; 0; 0];
u0 = [10 5 5 pi 0 0]';
%% 生成运动方程和相应的雅可比矩阵
% 运行一次就行，符号运算偏慢；runFlag = 1;

setOneShipStateAndJacobian(MM1, D1, L , 0);


%% MPC 控制器的定义
%这里的Np Nc指的是预测和控制步数，不是时间
Np = 10;
Nc = 3;
Ts = 1;%离散间隔 discretization interval
Tsim = 50;
obstacleExist = 0;%1 存在障碍物 0 不存在障碍物

Nship = 1;

nx = 6;
ny = 6;
nu = 6;

% 实例nlmpc类，其中输入量为11个，工具箱认为可测扰动MD也算输入，
% 不过计算的时候只计算操纵变量MV， 定义输入约束的时候也只是针对MV，
nlobj = nlmpc(nx, ny, nu);


nlobj.Model.StateFcn = "OneShipStateFcn";
nlobj.Jacobian.StateFcn = "OneShipStateJacobianFcn";

% 输出函数没用，会报错，待解决
% nlobj.Model.OutputFcn = @OutputFcn;


nlobj.Ts = Ts;
nlobj.PredictionHorizon = Np;
nlobj.ControlHorizon = Nc;

% rng(0)
% % 验证nlmpc
% validateFcns(nlobj,rand(nx,1),rand(nu,1));

nlobj.MV = struct('Min',{0; 0; 0; -2*pi; -2*pi; -2*pi},...
    'Max',{Fmax; Fmax; Fmax; 2*pi; 2*pi; 2*pi},...
    'RateMin', {-dFmax; -dFmax; -dFmax; -damax; -damax; -damax},...
    'RateMax', {dFmax; dFmax; dFmax; damax; damax; damax});

nlobj.Weights.OutputVariables = [250 250 250 0 0 0]*50;
% nlobj.Weights.OutputVariables = [250 250 250];

% nlobj.Weights.ManipulatedVariables =[1 1 1 5 5 5];
nlobj.Weights.ManipulatedVariables =[1 1 1 0 0 0];

nlobj.Weights.ManipulatedVariablesRate = [50 50 50 300 300 300]*0;

% mpc步进计算函数
nloptions = nlmpcmoveopt;

%-------------------------------------------------------------------------------
% 环境载荷设置
%-------------------------------------------------------------------------------
wind_param = struct('wind_speed', 5, 'wind_angle',  30/180*pi);
current_param = struct('current_speed', 0.3, 'current_angle', 30/180*pi);
wave_param = struct('Hs', 2.8, 'Tp', 13.4, 'gamma', 3.3, 'env_dir', 30/180*pi, 'nfr', 100);


%% 滤波器设置

state0 = zeros(15, 1);
state0(7:9, 1) = p0;
output0 = p0;
p0_hat = p0;
v0_hat = v0;
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

x0 = [p0; v0];


[p0_his, v0_his, pd0_his, vd0_his, f0_his, a0_his, tau0_his, p0_hat_his, v0_hat_his] = deal(zeros(Tsim+1, 3));
[x0_his, u0_his, x0_hat_his] = deal(zeros(Tsim+1, 6));  du_his = zeros(Tsim, 6);
x0_his(1, :) = [p0; v0]';       x0_hat_his(1, :) = [p0; v0]';

u0_his(1, :) = u0';  f0_his(1, :) = u0(1:3)';   a0_his(1, :) = u0(4:6)';
p0_his(1, :) = p0';  v0_his(1, :) = v0';  pd0_his(1, :) = p0';



[coreData, onlineData] = getCodeGenerationData(nlobj, x0, u0);


%% 主循环


for k = 1:Tsim
    k

    %找到leader前方最近的参考路径点序列
    x0_hat = [p0_hat; v0_hat];
    

    index=findNearestPoint(x0_hat(1:2),r(1:2,:));
    index_aft = index + Np - 1;

    if index_aft > length(r(1, :))
        Yd = [r(:, index : end), repmat(r(:, end), 1, index_aft - length(r(1, :)) )];
    else
        Yd = r(:,index:index+Np-1);
    end
    

    Yd = [Yd', zeros(Np, 3)];

    pd0_his(k+1, :) = Yd(1, 1:3);
    
    onlineData.ref = Yd;
    [u0,onlineData] = nlmpcmoveCodeGeneration(coreData,x0_hat,u0,onlineData);
%     [u0,onlineData] = nlmpcmoveMEX(coreData, x0_hat, u0, onlineData);

%     [u0, nloptions, info] = nlmpcmove(nlobj, x0, u0, Yd, nloptions);

    u0_his(k+1, :) = u0';   f0_his(k+1, :) = u0(1:3)';  a0_his(k+1, :) = u0(4:6)';
    du_his(k, :) = u0_his(k+1, :) - u0_his(k, :);

    [tau0_wind, tau0_current, tau0_wave1, tau0_wave2] = tau_env_tug_interface(p0_hat, k, wind_param,... 
            current_param, wave_param);
    tau0_env = tau0_wind + tau0_current + tau0_wave1 + tau0_wave2;
    Tenv = 30;
    if k < Tenv
        tau0_env = k / Tenv * tau0_env;
    end
    
    tau0 = thrusters_configuration(u0(4:6), L) * u0(1:3);
    [p0, v0] = ship_dynamic_solver(p0, v0, tau0+tau0_env, MM1, D1);

    [p0_hat, v0_hat, state0, output0] = nonlinear_passive_observer(p0, tau0, state0, output0, observer_paramters);
    p0_hat_his(i+1, :) = p0_hat';           v0_hat_his(i+1, :) = v0_hat';
    

    x0_his(k+1, :) = [p0; v0]';     x0_hat_his(k+1, :) = [p0_hat; v0_hat]';

end




figure(1)
plot(x0_hat_his(:, 1), x0_hat_his(:, 2))
hold on 
plot(pd0_his(:, 1), pd0_his(:, 2))
hold off
legend('x', 'r')
legend boxoff
title('循迹效果');

figure(2)
subplot(3, 2, 1); plot(x0_his(:, 1)); hold on; plot(x0_hat_his(:, 1)); hold on; plot(pd0_his(:, 1)); hold off; legend('p0', 'p0_{hat}', 'pd'); legend boxoff
subplot(3, 2, 3); plot(x0_his(:, 2)); hold on; plot(x0_hat_his(:, 2)); hold on; plot(pd0_his(:, 2)); hold off;
subplot(3, 2, 5); plot(x0_his(:, 3)/pi*180); hold on; plot(x0_hat_his(:, 3)/pi*180); hold on; plot(pd0_his(:, 3)/pi*180); hold off;
subplot(3, 2, 2); plot(x0_his(:, 4)); hold on; plot(x0_hat_his(:, 4)); hold off; legend('v0', 'v0_{hat}'); legend boxoff
subplot(3, 2, 4); plot(x0_his(:, 5)); hold on; plot(x0_hat_his(:, 5)); hold off;
subplot(3, 2, 6); plot(x0_his(:, 6)/pi*180); hold on; plot(x0_hat_his(:, 6)/pi*180); hold off;

figure(3)
subplot(2, 2, 1); plot(u0_his(:, 1:3)); legend("f1", 'f2', 'f3'); legend boxoff; title('f');
subplot(2, 2, 3); plot(u0_his(:, 4:6)/pi*180); legend("a1", 'a2', 'a3'); legend boxoff; title('a');
subplot(2, 2, 2); plot(du_his(:, 1:3)); legend("df1", 'df2', 'df3'); legend boxoff; title('df');
subplot(2, 2, 4); plot(du_his(:, 4:6)/pi*180); legend("da1", 'da2', 'da3'); legend boxoff; title('da');



toc

%% mex文件生成
% func = 'nlmpcmoveCodeGeneration';
% funcOutput = 'nlmpcmoveMEX';
% Cfg = coder.config('mex');
% Cfg.DynamicMemoryAllocation = 'off';
% codegen('-config',Cfg,func,'-o',funcOutput,'-args', {coder.Constant(coreData), x0, u0, onlineData});



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
