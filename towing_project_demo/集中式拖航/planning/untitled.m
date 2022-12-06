%% 基本数据
clear all; clc; clf; close all; tic;
M0 = diag([5.3e7, 5.3e7, 7.5e10]);
Ma0 = diag([0.8e7, 3.3e7, 3.5662e9]);
MM0 = M0 + Ma0;
D0 = diag([2.44e6, 4.88e6, 3.89e9]);
L0 = [75, 50; -75, 50; -75, -50; 75, -50]; %拖轮编队的排序按照坐标系来
setOneShipStateAndJacobian(MM0, D0, L0, 0)

%% 模拟参数
max_step = 30;
% deg2rad = pi/180;
time_step = 1;

%% mpc planning obj的设置
Ts = time_step;
p = max_step;
nc = p;
nx = 6;
nu = 8;
ny = 6;
nlobj_plan = nlmpc(nx,ny,nu);
nlobj_plan.Ts = Ts;
nlobj_plan.PredictionHorizon = p;
nlobj_plan.ControlHorizon = nc;


nlobj_plan.Model.StateFcn = 'OneShipStateFcn';
nlobj_plan.Jacobian.StateFcn = 'OneShipStateJacobianFcn';
nlobj_plan.Optimization.CustomCostFcn = @(X,U,e,data) Ts*sum(sum(U(1:p,1:4)));
nlobj_plan.Optimization.ReplaceStandardCost = true;
Termination = [1, 1, 1/180*pi]';
nlobj_plan.Optimization.CustomEqConFcn = @(X,U,data) X(end,1:3)' - Termination;


Fmax = 800;  dFmax = 100;    damax = deg2rad(10);
% nlobj_plan.MV = struct('Min',{0; 0; 0; 0; deg2rad(-45); deg2rad(135); deg2rad(135); deg2rad(-45)},...
%     'Max',{Fmax; Fmax; Fmax; Fmax; deg2rad(45); deg2rad(225); deg2rad(225); deg2rad(45)},...
%     'RateMin', {-dFmax; -dFmax; -dFmax; -dFmax; -damax; -damax; -damax; -damax},...
%     'RateMax', {dFmax; dFmax; dFmax; dFmax; damax; damax; damax; damax});

nlobj_plan.MV = struct('Min',{0; 0; 0; 0; deg2rad(-45); deg2rad(135); deg2rad(135); deg2rad(-45)},...
    'Max',{Fmax; Fmax; Fmax; Fmax; deg2rad(45); deg2rad(225); deg2rad(225); deg2rad(45)});

x0 = [0, 0, 0, 0, 0, 0]';
u0 = [1 1 1 1 deg2rad(0) deg2rad(180) deg2rad(180) deg2rad(0)]';


validateFcns(nlobj_plan,x0,u0);


[~,~,info] = nlmpcmove(nlobj_plan,x0,u0);

pd = info.Xopt(:, 1:3)';    vd = info.Xopt(:, 4:6)';
f_his = info.MVopt(:, 1:4)';    a_his = info.MVopt(:, 5:8)';
plot32_1lines(pd, vd, 'pd-vd', {'pd', 'vd'}, 1, 1);
plot1_4lines(f_his, '推力', {'f1', 'f2', 'f3', 'f4'}, 2);
plot1_4lines(a_his(:, 1:end-1)/pi*180, '转角', {'a1', 'a2', 'a3', 'a4'}, 3);

toc


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
