clear all;  clc;   close all;

% getShipStateAndJacobian;
% p=parpool(5);
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
global Q_in_fo_firstep
global Q_din_fo_firstep
global Q_fm_fo_firstep
global Q_in_fo_substep
global Q_din_fo_substep
global Q_fm_fo_substep
global terCoe  %终端代价的比例系数

global obstacleExist

N=4;%每条船的推进器数量
thrConf=[0.4 0.2;-0.4 0.2;-0.4 -0.2;0.4 -0.2];%更改坐标系后

%权值矩阵取值
Q_err=diag([250 250 250]);%适当把艏向角权重调大一点
terCoe=20;%terminal cost的权值系数
Q_in=[1*eye(4) zeros(4);zeros(4) zeros(4)];%使得推力最小的权值矩阵 只有左上角有值
Q_dinleader=diag([1 1 1 1 50 50 50 50]);
Q_dinfo=diag([50 50 50 50 300 300 300 300]);%使得推力、角度变化量最小的权值矩阵
Q_fm=diag([750 750 750]);%适当把艏向角权重调大一点

%这里的Np Nc指的是预测和控制步数，不是时间
Np=3;
Nc=2;
Ts=2;%离散间隔 discretization interval
Tsim=300;
obstacleExist=0;%1 存在障碍物 0 不存在障碍物

%% MPC 控制器
nx = 6;
ny = 6;
nu = 8;
nlobj = nlmpc(nx, ny, nu);

nlobj.Model.StateFcn = "ShipStateFcn";
nlobj.Jacobian.StateFcn = @ShipStateJacobianFcn;
% nlobj.Model.OutputFcn = @OutputFcn;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = Np;
nlobj.ControlHorizon = Nc;

rng(0)

validateFcns(nlobj,rand(nx,1),rand(nu,1));

nlobj.MV = struct('Min',{-20; -20; -20; -20; -2*pi; -2*pi; -2*pi; -2*pi},...
    'Max',{20; 20; 20; 20; 2*pi; 2*pi; 2*pi; 2*pi},...
    'RateMin', {-4; -4; -4; -4; deg2rad(-15);deg2rad(-15);deg2rad(-15);deg2rad(-15);},...
    'RateMax', {4; 4; 4; 4; deg2rad(15);deg2rad(15);deg2rad(15);deg2rad(15);});

nlobj.Weights.OutputVariables = [250 250 250 0 0 0];
% nlobj.Weights.OutputVariables = [250 250 250];

nlobj.Weights.ManipulatedVariables =[1 1 1 1 5 5 5 5];

nlobj.Weights.ManipulatedVariablesRate = [50 50 50 50 300 300 300 300];


% Nominal control that keeps the quadrotor floating
nloptions = nlmpcmoveopt;
% nloptions.MVTarget = [0 0 0 0 0 0 0 0]; 
% mv = nloptions.MVTarget;










%% 参考轨迹
%situation map
load ref_map_3 %间隔0.1m 缩尺比1:500 宽75m变成30m
yy=1300-yy;%像素反转
xx=xx*30/1900;%从像素转换到坐标 m(缩尺比1:200 宽75m）
yy=yy*(30*13/19)/1300;
r=zeros(3,size(xx,2));
r(1,:)=xx;r(2,:)=yy;
for i=1:size(r,2)
    if i==1
        r(3,i)=deg2rad(-20); %起点
    elseif i==size(r,2)
        r(3,i)=deg2rad(-44.5); %终点
    else
    r(3,i)=atan((r(2,i+1)-r(2,i-1))/(r(1,i+1)-r(1,i-1)));%单位 rad
    end
end


%% 船舶初始值
%船舶初始状态
% ref_map_3的初始值
C = [eye(3), zeros(3, 3); zeros(3, 3), zeros(3, 3)];
x0=[3;4.4;deg2rad(-20);0;0;0];
y0 = C * x0;
% u0_leader=[0;0;0;0;deg2rad(0);deg2rad(0);deg2rad(0);deg2rad(0)];
% 
% U0_leader=repmat(u0_leader,Nc,1);
mv = [0 0 0 0 0 0 0 0];
Duration = 250;
% hbar = waitbar(0,'Simulation Progress');
xHistory = x0';
lastMV = mv;
uHistory = lastMV;
Yd_his = zeros(Np, 1);
% for k = 1:(Duration/Ts)
for k = 1:Tsim
    k
    % Set references for previewing
%     t = linspace(k*Ts, (k+p-1)*Ts, p);
%     yref = QuadrotorReferenceTrajectory(t);
%     
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
%     Yd = Yd';
%     Yd_his = [Yd_his, Yd(:, 1)];
    
    Yd = [Yd', zeros(Np, 3)];
    Yd_his = [Yd_his, Yd(:, 1)];
    
    % Compute the control moves with reference previewing.
    
    [uk,nloptions,info] = nlmpcmove(nlobj,xk,lastMV,Yd,[],nloptions);
    uHistory(k+1,:) = uk';
    lastMV = uk;
    % Update states.
    ODEFUN = @(t,xk) ShipStateFcn(xk,uk);
    [TOUT,YOUT] = ode45(ODEFUN,[0 Ts], xHistory(k,:)');
    xHistory(k+1,:) = YOUT(end,:);
%     waitbar(k*Ts/Duration,hbar);
end
% close(hbar)

plot(xHistory(:, 1), xHistory(:, 2))
hold on 
plot(r(1, :), r(2, :))
hold off
legend('x', 'r')


