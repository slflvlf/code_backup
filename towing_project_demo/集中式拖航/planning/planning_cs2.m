%% 基本数据
clear all; clc; clf; close all; tic;
M0 = diag([5.3e7, 5.3e7, 7.5e10]);
Ma0 = diag([0.8e7, 3.3e7, 3.5662e8]);
MM0 = M0 + Ma0;
D0 = diag([2.44e6, 4.88e6, 3.89e9]);
L0 = [75, 50; -75, 50; -75, -50; 75, -50]; %拖轮编队的排序按照坐标系来
% setOneShipStateAndJacobian(MM0, D0, L0, 1)


M1 = diag([21.67, 39.08, 14.56]);
D1 = diag([23.52, 22.32, 3.762]);
L1 = [0.4, 0.5; -0.4, 0.5; -0.4, -0.5; 0.4, -0.5];
setOneShipStateAndJacobian( M1, D1, L1, 0)


%% 模拟参数
max_step = 20;
% deg2rad = pi/180;
time_step = 0.5;

%% mpc planning obj的设置
Ts = time_step;
p = max_step;
nx = 6;
nu = 8;
nlobj_plan = nlmpcMultistage(p, nx, nu);
nlobj_plan.Ts = Ts;

nlobj_plan.Model.StateFcn = 'OneShipStateFcn';
nlobj_plan.Model.StateJacFcn = 'OneShipStateJacobianFcn';
for ct = 1:p
    nlobj_plan.Stages(ct).CostFcn = 'planning_costFcn';
%     nlobj_plan.Stages(ct).CostJacFcn = 'planning_Cost_Gradient_Fcn';
end

Termination = [5, 0, 0/180*pi]';
% nlobj_plan.Model.TerminalState = Termination;
nlobj_plan.Model.TerminalState = [Termination; inf; inf; inf];
Fmax = 800;  dFmax = 100;    damax = deg2rad(20);
% for i = 1 : 4
%     nlobj_plan.MV(i).Min = 0;
%     nlobj_plan.MV(i).Max = Fmax;
% end
% 
% 
% nlobj_plan.MV(5).Min = -2*pi;  nlobj_plan.MV(5).Max = 2*pi;
% nlobj_plan.MV(6).Min = -2*pi;  nlobj_plan.MV(6).Max = 2*pi;
% nlobj_plan.MV(7).Min = -2*pi;  nlobj_plan.MV(7).Max = 2*pi;
% nlobj_plan.MV(8).Min = -2*pi;  nlobj_plan.MV(8).Max = 2*pi;
% nlobj_plan.MV = struct('Min',{0; 0; 0; 0; deg2rad(-45); deg2rad(135); deg2rad(135); deg2rad(-45)},...
%     'Max',{Fmax; Fmax; Fmax; Fmax; deg2rad(45); deg2rad(225); deg2rad(225); deg2rad(45)},...
%     'RateMin', {-dFmax; -dFmax; -dFmax; -dFmax; -damax; -damax; -damax; -damax},...
%     'RateMax', {dFmax; dFmax; dFmax; dFmax; damax; damax; damax; damax});


nlobj_plan.MV = struct('Min',{-20; -20; -20; -20; -2*pi; -2*pi; -2*pi; -2*pi},...
    'Max',{20; 20; 20; 20; 2*pi; 2*pi; 2*pi; 2*pi},...
    'RateMin', {-4; -4; -4; -4; deg2rad(-15);deg2rad(-15);deg2rad(-15);deg2rad(-15);},...
    'RateMax', {4; 4; 4; 4; deg2rad(15);deg2rad(15);deg2rad(15);deg2rad(15);});


nlobj_plan.Optimization.SolverOptions.MaxIterations = 1000;

x0 = [0, 0, 0, 0, 0, 0]';
u0 = [0 0 0 0 deg2rad(0) deg2rad(0) deg2rad(0) deg2rad(0)]';


% validateFcns(nlobj_plan,x0,u0);


[~,~,info] = nlmpcmove(nlobj_plan, x0, u0);

pd = info.Xopt(:, 1:3)';    vd = info.Xopt(:, 4:6)';
f_his = info.MVopt(:, 1:4)';    a_his = info.MVopt(:, 5:8)';
plot32_1lines(pd, vd, 'pd-vd', {'pd', 'vd'}, 1, 1);
plot1_4lines(f_his, '推力', {'f1', 'f2', 'f3', 'f4'}, 2);
plot1_4lines(a_his(:, 1:end-1)/pi*180, '转角', {'a1', 'a2', 'a3', 'a4'}, 3);

toc


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
