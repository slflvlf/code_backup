clear all;  clc;   close all;
tic
%% 生成运动方程和相应的雅可比矩阵
% 运行一次就行，符号运算偏慢；
% 运行过后，雅可比矩阵B的大小要从11列改成8列
% getShipStateAndJacobian;

%% 全局变量
global M;global D;global N;global thrConf;
global G;global C;
global Q_err %误差权值矩阵 3x3
global Q_in  %输入权值矩阵 8x8 使得推力最小
global Q_dinleader %输入权值矩阵 8x8 使得推力、角度变化量最小
global Q_dinfo
global Q_fm  %编队权值矩阵 3x3
global Ts;global Tsim;global t;
global Np;global Nc;

global Q_err_leader
global Q_in_leader
global Q_din_leader



global terCoe  %终端代价的比例系数

N=4;%每条船的推进器数量
thrConf=[0.4 0.2;-0.4 0.2;-0.4 -0.2;0.4 -0.2];%更改坐标系后

%权值矩阵取值
global Q_error Q_input;
% global Ref;
Q_error=diag([350 350 350]);%适当把艏向角权重调大一点
Q_input=diag([1 1 1 1 50 50 50 50]);

terCoe=20;%terminal cost的权值系数
Q_in=[1*eye(4) zeros(4);zeros(4) zeros(4)];%使得推力最小的权值矩阵 只有左上角有值
Q_dinleader=diag([1 1 1 1 50 50 50 50]);
Q_dinfo=diag([50 50 50 50 300 300 300 300]);%使得推力、角度变化量最小的权值矩阵
Q_fm=diag([750 750 750]);%适当把艏向角权重调大一点



%% MPC 控制器的定义
%这里的Np Nc指的是预测和控制步数，不是时间
Np=5;
Nc=3;
Ts=1;%离散间隔 discretization interval
Tsim=50;
obstacleExist=0;%1 存在障碍物 0 不存在障碍物

nx = 6;
ny = 3;
nu = 11;

% 实例nlmpc类，其中输入量为11个，工具箱认为可测扰动MD也算输入，
% 不过计算的时候只计算操纵变量MV， 定义输入约束的时候也只是针对MV，
nlobj = nlmpc(nx, ny, 'MV', 1:8, 'MD', 9:11);


% nlobj.Model.StateFcn = @(x, u) ShipStateFcn(x, u, Yd);
% nlobj.Jacobian.StateFcn = @(x, u) ShipStateJacobianFcn(x, u, Yd);
% nlobj.Model.OutputFcn = @(x, u)OutputFcn(x, u, Yd);
% nlobj.Optimization.CustomCostFcn = @(X, U, e, data) CostFcn(X, U, e, data, Yd);
% nlobj.Optimization.ReplaceStandardCost = true;

nlobj.Model.StateFcn = "ShipStateFcn";
nlobj.Jacobian.StateFcn = "ShipStateJacobianFcn";
nlobj.Model.OutputFcn = "OutputFcn";
nlobj.Optimization.CustomCostFcn = "CostFcn";
nlobj.Optimization.ReplaceStandardCost = true;

Ref = zeros(5, 3);
param = {Ref}';
nlobj.Model.NumberOfParameters = numel(param);




nlobj.Ts = Ts;
nlobj.PredictionHorizon = Np;
nlobj.ControlHorizon = Nc;

% 验证nlmpc
rng(0)
validateFcns(nlobj,rand(nx,1),rand(nu-3,1), rand(1, 3), {zeros(5, 3)});

nlobj.MV = struct('Min',{-20; -20; -20; -20; -2*pi; -2*pi; -2*pi; -2*pi},...
    'Max',{20; 20; 20; 20; 2*pi; 2*pi; 2*pi; 2*pi},...
    'RateMin', {-4; -4; -4; -4; deg2rad(-15);deg2rad(-15);deg2rad(-15);deg2rad(-15);},...
    'RateMax', {4; 4; 4; 4; deg2rad(15);deg2rad(15);deg2rad(15);deg2rad(15);});



% nlobj.Weights.OutputVariables = [250 250 250 0 0 0];
nlobj.Weights.OutputVariables = Q_error;

nlobj.Weights.ManipulatedVariables =Q_input;

nlobj.Weights.ManipulatedVariablesRate = [50 50 50 50 300 300 300 300];


% mpc步进计算函数
nloptions = nlmpcmoveopt;

nloptions.Parameters = {Ref}';



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
C = [eye(3), zeros(3, 3); zeros(3, 3), zeros(3, 3)];
x0 = [3; 4.4; deg2rad(-20); 0; 0; 0];
y0 = C * x0;

mv = [0 0 0 0 0 0 0 0];

hbar = waitbar(0,'Simulation Progress');
xHistory = x0';
lastMV = mv;
uHistory = lastMV;
Yd_his = zeros(Np, 1);
%% 环境力计算
%根据试验数据以及海区风浪关系计算风浪流载荷
WaveDirection = 180;%顶浪航行
WindVelocity = 5;%m/s
CurrentVelocity = 0.512;%m/s
w_scale = [0.02;0.02;0];%相当于尺度比1:50
w_real_seed = EnvLoadsCal(WaveDirection, WindVelocity, CurrentVelocity)'.*w_scale;
w_real = w_real_seed;


%暂时用添加扰动的w作为RBFNN预估的w_estimated，这篇论文里面可以暂时不用神经网络预估（根据误差反馈）
w_estimated = w_real + [0; 0; 0] .* ((-1) + (1 + 1) * rand(3, 1));

%% 主循环
for k = 1:Tsim
    k

    %找到leader前方最近的参考路径点序列
    xk = xHistory(k,:);
    y0 = xk(1:3);
    index=findNearestPoint(y0(1:2),r(1:2,:));
    index_aft = index + Np - 1;
    if index_aft > length(r(1, :))
        Yd = [r(:, index : end), repmat(r(:, end), 1, index_aft - length(r(1, :)) )];
    else
        Yd = r(:,index:index+Np-1);
    end
    
    Yd = Yd';
    Ref = Yd;
    nloptions.Parameters = {Ref}';


    Yd_his = [Yd_his, Yd(:, 1)];

%     Yd = [Yd', zeros(Np, 3)];
%     Yd_his = [Yd_his, Yd(:, 1)];
    
    % Compute the control moves with reference previewing.
    w_estimated = w_real + [0; 0; 0] .* ((-1) + (1 + 1) * rand(3, 1));
    
    
    [uk, nloptions, info] = nlmpcmove(nlobj, xk, lastMV, Yd, w_estimated', nloptions);
    uHistory(k+1, :) = uk';
    lastMV = uk;
    % Update states.
    ODEFUN = @(t, xk) ShipStateFcn(xk, [uk; w_real]);
    [TOUT,YOUT] = ode45(ODEFUN, [0 Ts], xHistory(k, :)');
    xHistory(k+1, :) = YOUT(end, :);
    
    waitbar(k/Tsim, hbar);
%     waitbar(k*Ts/Duration, hbar);
end
close(hbar)
toc


figure(1)
plot(xHistory(:, 1), xHistory(:, 2))
hold on 
plot(r(1, :), r(2, :))
hold off
legend('x', 'r')
legend boxoff
title('循迹效果');




