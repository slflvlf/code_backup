%%%%%------Centralized MPC------%%%%%
%%%%%     2021.10.07
clear;close all;

%% 全局变量
global M;global D;global N;global thrConf;
global G;global C;
global Q_err %误差权值矩阵 3x3
global Q_in  %输入权值矩阵 8x8 使得推力最小
global Q_din %输入权值矩阵 8x8 使得推力、角度变化量最小 
global Q_fm  %编队权值矩阵 3x3
global Ts;global Tsim;global t;
global Np;global Nc;

global Q_IN
global Q_dIN
global Q_Err
global terCoe
global Q_obs_avoidance
global gamma %避障目标函数中的正数
% global pos_obstacle_static
% global pos_obstacle_dynamic
global obstacleExist

global R;%测量噪声的协方差矩阵
%% 船舶模型
%论文中的无人船
M=[21.67 0 0;0 39.08 0;0 0 14.56];%质量矩阵
D=[23.52 0 0;0 22.32 0;0 0 3.762];%阻尼矩阵

% %CyberShip II
% M=[25.8 0 0;0 33.8 1;0 1 2.8];%质量矩阵
% D=[0.72 0 0;0 0.89 0.03;0 0.03 1.9];%阻尼矩阵

N=4;%每条船的推进器数量
thrConf=[0.4 0.2;-0.4 0.2;-0.4 -0.2;0.4 -0.2];%四个推进器相对于船中心的坐标矩阵 [lx1 ly1;lx2 ly2;lx3 ly3;lx4 ly4]    

G=[zeros(3);inv(M)];
C=[eye(3) zeros(3)];
%% CMPC控制参数
% Q_err=diag([150 150 150]);%适当把艏向角权重调大一点
% terCoe=20;%terminal cost的权值系数
% Q_in=[1*eye(4) zeros(4);zeros(4) zeros(4)];%使得推力最小的权值矩阵 只有左上角有值
% Q_din=diag([1 1 1 1 150 150 150 150]);%使得推力、角度变化量最小的权值矩阵
% Q_fm=diag([150 150 150]);%适当把艏向角权重调大一点
% Q_obs_avoidance=2;
% gamma=0;

Q_err=diag([250 250 250]);%适当把艏向角权重调大一点
terCoe=20;%terminal cost的权值系数
Q_in=[1*eye(4) zeros(4);zeros(4) zeros(4)];%使得推力最小的权值矩阵 只有左上角有值
Q_din=diag([1 1 1 1 150 150 150 150]);%使得推力、角度变化量最小的权值矩阵
Q_fm=diag([750 750 750]);%适当把艏向角权重调大一点
Q_obs_avoidance=2;
gamma=0;

%Np Nc指的是预测和控制步数
Np=3;
Nc=2;
Ts=2;%离散间隔
Tsim=300;
obstacleExist=0;

%是否开启状态观测器
StateEst=0;%1---开启 0---关闭
%% 创建权值矩阵
[Q_IN,Q_dIN,Q_Err]=createWeightMat();
%% 环境干扰
%根据试验数据以及海区风浪关系计算风浪流载荷
WaveDirection=180;%顶浪航行
WindVelocity=5;%m/s
CurrentVelocity=0.512;%m/s
w_scale=[0.02;0.02;0];%相当于尺度比1:50
w_real_seed=EnvLoadsCal(WaveDirection,WindVelocity,CurrentVelocity)'.*w_scale;
w_real=repmat(w_real_seed,1,Tsim);

%暂时用添加扰动的w作为RBFNN预估的w_estimated
%这篇论文里面可以暂时不用神经网络预估（根据误差反馈）
w_estimated=w_real+[0;0;0].*((-1)+(1+1)*rand(3,Tsim));

%% 测量噪声
scale_v_position=0.01;scale_v_angle=0.0;
v=zeros(3,Tsim);
%v=normrnd(0,1,[3,Tsim]);
for i=1:Tsim
    v(1:2,i)=normrnd(0,1,[2,1]);%均值=0，方差=第二个数^2
    v(3,i)=normrnd(0,1,[1,1]);
end
v(1:2,:)=v(1:2,:)*scale_v_position;v(3,:)=v(3,:)*scale_v_angle;
R=1*eye(3);
%% 参考轨迹
%map
% load ref_map %0.2m间隔路径点
load ref_map_3 %0.1m间隔 缩尺比1:500
yy=1300-yy;%像素反转
xx=xx*30/1900;
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

%% obstacle
% load r_ori
% % static obstacle
% pos_obstacle_static=repmat(r_ori(1:2,35),1,Np);%静态障碍物位置
% 
% %动态障碍物第一种设置 两岸来回
% % pos_obstacle_dynamic=zeros(2,Np);
% % for j=1:Np
% %     pos_obstacle_dynamic(1,j)=9;
% %     pos_obstacle_dynamic(2,j)=5*sin((t+j-1)*pi/20+pi/2)+5.5;%20为河岸来回一趟的step数，需根据实际情况调整；sin前面的倍数需要和上下限一致
% % end
% % pos_obstacle_dynamic_upboundary=[9;10.5];
% % pos_obstacle_dynamic_downboundary=[9;0.5];
% 
% %动态障碍物第二种设置 沿参考轨迹
% temp=r_ori(1:2,108:118);
% temp(2,:)=temp(2,:)+0.2;%往上移0.2m
% pos_obstacle_dynamic_upboundary=temp(:,1);
% pos_obstacle_dynamic_downboundary=temp(:,end);
% pos_obstacle_dynamic_ref=repmat(cat(2,temp,fliplr(temp)),1,100);
% pos_obstacle_dynamic=pos_obstacle_dynamic_ref(:,t:t+Np-1);

%% 编队参数 用于计算follower的期望路径点
range12=1;
range13=1;
bearing12=deg2rad(0);
bearing13=deg2rad(0);
%% 船舶初始值
%situation huangpu river
% ref-_map_3的初始值 编队2m
x0_leader=[3;4.4;deg2rad(-20);0;0;0];
x0_leftfo=[3;5.4;deg2rad(-20);0;0;0];
x0_rightfo=[3;3.4;deg2rad(-20);0;0;0];

x0e_leader=x0_leader;
x0e_leftfo=x0_leftfo;
x0e_rightfo=x0_rightfo;

y0_leader=C*x0_leader;
y0_leftfo=C*x0_leftfo;
y0_rightfo=C*x0_rightfo;

x0est_leader=x0_leader;%初始状态估计值 只对leader使用状态估计
y0est_leader=y0_leader;%初始预测输出

u0_leader=[0;0;0;0;deg2rad(0);deg2rad(0);deg2rad(0);deg2rad(0)];
u0_leftfo=[0;0;0;0;deg2rad(0);deg2rad(0);deg2rad(0);deg2rad(0)];
u0_rightfo=[0;0;0;0;deg2rad(0);deg2rad(0);deg2rad(0);deg2rad(0)];
U0_leader=repmat(u0_leader,Nc,1);
U0_leftfo=repmat(u0_leftfo,Nc,1);
U0_rightfo=repmat(u0_rightfo,Nc,1);

%初始误差协方差矩阵
P0=0.5*eye(6);
%% 时域模拟
%collect information
%leader
OutputCollect_leader=y0_leader;
velocityCollect_leader=x0_leader(4:6);
uCollect_leader=u0_leader;
tauCollect_leader=[];

%follower
OutputCollect_leftfo=y0_leftfo;
OutputCollect_rightfo=y0_rightfo;
velocityCollect_leftfo=x0_leftfo(4:6);
velocityCollect_rightfo=x0_rightfo(4:6);
uCollect_leftfo=u0_leftfo;
uCollect_rightfo=u0_rightfo;
tauCollect_leftfo=[];
tauCollect_rightfo=[];

%other information
indexCollect=[];
timeCollect=zeros(1,Tsim);
posObsDynCollect=[];

%simulation
for t=1:Tsim
    tic;
    t %查看运行进度
    
    %找到leader前方最近的参考路径点序列
    index=findNearestPoint(y0_leader(1:2),r(1:2,:));
    Yd_leader=r(:,index:index+Np-1);
    indexCollect=[indexCollect index];
    
    %根据编队控制参数计算follower的期望路径点序列
    Yd_leftfo=desiredOutputCal_follower(Yd_leader,range12,bearing12,1);
    Yd_rightfo=desiredOutputCal_follower(Yd_leader,range13,bearing13,2);
    
    %Global controller
    [x_leader,xe_leader,y_leader,U_leader,tau_leader,...
    x_leftfo,xe_leftfo,y_leftfo,U_leftfo,tau_leftfo,...
    x_rightfo,xe_rightfo,y_rightfo,U_rightfo,tau_rightfo]=global_controller([U0_leader;U0_leftfo;U0_rightfo],[x0_leader;x0_leftfo;x0_rightfo],[x0e_leader;x0e_leftfo;x0e_rightfo],repmat(w_estimated(:,t),3,1),repmat(v(:,t),3,1),[Yd_leader;Yd_leftfo;Yd_rightfo]);

    
    %更新信息
    %leader
    U0_leader=U_leader;
    x0_leader=x_leader;
    x0e_leader=xe_leader;
    y0_leader=y_leader;
    
    %leftfo
    %leftfo出现信息缺失
    if t>=115 && t<=125
        U0_leftfo=zeros(16,1);
    else
        U0_leftfo=U_leftfo;
    end
    x0_leftfo=x_leftfo;
    x0e_leftfo=xe_leftfo;
%     %leftfo正常
%     U0_leftfo=U_leftfo;
%     x0_leftfo=x_leftfo;
%     x0e_leftfo=xe_leftfo;
    
    %rightfo
    U0_rightfo=U_rightfo;
    x0_rightfo=x_rightfo;
    x0e_rightfo=xe_rightfo;    
    
    timeCollect(t)=toc;
    
    %collect information
    OutputCollect_leader=[OutputCollect_leader y_leader];
    velocityCollect_leader=[velocityCollect_leader x_leader(4:6)];
    uCollect_leader=[uCollect_leader U_leader(1:8)];
    tauCollect_leader=[tauCollect_leader tau_leader];
    
    OutputCollect_leftfo=[OutputCollect_leftfo y_leftfo];
    velocityCollect_leftfo=[velocityCollect_leftfo x_leftfo(4:6)];
    uCollect_leftfo=[uCollect_leftfo U_leftfo(1:8)];
    tauCollect_leftfo=[tauCollect_leftfo tau_leftfo];
    
    OutputCollect_rightfo=[OutputCollect_rightfo y_rightfo];
    velocityCollect_rightfo=[velocityCollect_rightfo x_rightfo(4:6)];
    uCollect_rightfo=[uCollect_rightfo U_rightfo(1:8)];
    tauCollect_rightfo=[tauCollect_rightfo tau_rightfo];
   
    
    %查看信息
    y_leader
    y_leftfo
    y_rightfo
end

%% picture
%三条船的路径跟踪
figure(1)
% plot(r_ori(1,:),r_ori(2,:),'k:')
% hold on
plot(r(1,:),r(2,:),'k--')
hold on
plot(OutputCollect_leader(1,:),OutputCollect_leader(2,:),OutputCollect_leftfo(1,:),OutputCollect_leftfo(2,:),OutputCollect_rightfo(1,:),OutputCollect_rightfo(2,:),'linewidth',1)
% hold on
% plot(pos_obstacle_static(1,1),pos_obstacle_static(2,1),'sk','markerfacecolor','k')
% hold on
% plot(pos_obstacle_dynamic_upboundary(1),pos_obstacle_dynamic_upboundary(2),'sk',pos_obstacle_dynamic_downboundary(1),pos_obstacle_dynamic_downboundary(2),'sk','markerfacecolor','k')
xlabel('X/m');ylabel('Y/m');title('path following of three vessels')
legend('Desired','Leader','Left follower','Right follower')

%leader跟踪误差
figure(2)
objPath=zeros(3,Tsim);
for i=1:Tsim
objPath(:,i)=r(:,indexCollect(i));
end
subplot(2,1,1)
plot(0:Tsim,zeros(1,Tsim+1),'k--')
hold on
plot(1:Tsim,OutputCollect_leader(end-2,2:end)-objPath(1,:),'linewidth',1)
hold on
plot(1:Tsim,OutputCollect_leader(end-1,2:end)-objPath(2,:),'linewidth',1)
ylabel('Linear error/m');title('Path following error of leader')
legend('Desired error','Surge error','Sway error')
subplot(2,1,2)
plot(0:Tsim,zeros(1,Tsim+1),'k--')
hold on
plot(1:Tsim,rad2deg(OutputCollect_leader(end,2:end)-objPath(3,:)),'linewidth',1)
xlabel('Step')
ylabel('Yaw error/(deg)')
legend('Desired error','Yaw error')

%编队误差
figure(3)
formation_leader_leftfo=formationCal(OutputCollect_leader,OutputCollect_leftfo);
formation_leader_leftfo(1,:)=formation_leader_leftfo(1,:)-range12;
formation_leader_rightfo=formationCal(OutputCollect_leader,OutputCollect_rightfo);
formation_leader_rightfo(1,:)=formation_leader_rightfo(1,:)-range13;
subplot(2,1,1)%距离保持
plot(0:Tsim,zeros(1,Tsim+1),'k--')
hold on
plot(0:Tsim,formation_leader_leftfo(1,:),0:Tsim,formation_leader_rightfo(1,:),'linewidth',1)
ylabel('Distance error/m');legend('Desired error','Leader-left follower','Leader-right follower')
title('Formation error')
subplot(2,1,2)%艏向角偏差
plot(0:Tsim,zeros(1,Tsim+1),'k--')
hold on
plot(0:Tsim,formation_leader_leftfo(2,:),0:Tsim,formation_leader_rightfo(2,:),'linewidth',1)
xlabel('Time/s');ylabel('Yaw angle error/deg');legend('Desired error','Leader-left follower','Leader-right follower')

%推力、角度时历 三条船
figure(4)
uCollect_leader(5:8,:)=rad2deg(uCollect_leader(5:8,:));
uCollect_leftfo(5:8,:)=rad2deg(uCollect_leftfo(5:8,:));
uCollect_rightfo(5:8,:)=rad2deg(uCollect_rightfo(5:8,:));
subplot(3,2,1)%leader 推力
plot(0:Tsim,uCollect_leader(1,:),0:Tsim,uCollect_leader(2,:),0:Tsim,uCollect_leader(3,:),0:Tsim,uCollect_leader(4,:),'linewidth',1)
hold on
plot(0:Tsim,20*ones(Tsim+1,1),'--k',0:Tsim,-20*ones(Tsim+1,1),'--k','linewidth',1)%推力上下限
ylabel('Thrust/N');
title('Leader')
subplot(3,2,2)%leader 艏向角
plot(0:Tsim,uCollect_leader(5,:),0:Tsim,uCollect_leader(6,:),0:Tsim,uCollect_leader(7,:),0:Tsim,uCollect_leader(8,:),'linewidth',1)
ylabel('Angle/(deg)')
title('Leader')

subplot(3,2,3)%leftfo 推力
plot(0:Tsim,uCollect_leftfo(1,:),0:Tsim,uCollect_leftfo(2,:),0:Tsim,uCollect_leftfo(3,:),0:Tsim,uCollect_leftfo(4,:),'linewidth',1)
hold on
plot(0:Tsim,20*ones(Tsim+1,1),'--k',0:Tsim,-20*ones(Tsim+1,1),'--k','linewidth',1)%推力上下限
ylabel('Thrust/N');
title('Left follower')
subplot(3,2,4)%leftfo 艏向角
plot(0:Tsim,uCollect_leftfo(5,:),0:Tsim,uCollect_leftfo(6,:),0:Tsim,uCollect_leftfo(7,:),0:Tsim,uCollect_leftfo(8,:),'linewidth',1)
ylabel('Angle/(deg)')
title('Left follower')

subplot(3,2,5)%rightfo 推力
plot(0:Tsim,uCollect_rightfo(1,:),0:Tsim,uCollect_rightfo(2,:),0:Tsim,uCollect_rightfo(3,:),0:Tsim,uCollect_rightfo(4,:),'linewidth',1)
hold on
plot(0:Tsim,20*ones(Tsim+1,1),'--k',0:Tsim,-20*ones(Tsim+1,1),'--k','linewidth',1)%推力上下限
ylabel('Thrust/N');xlabel('Time/s')
title('Right follower')
subplot(3,2,6)%rightfo 艏向角
plot(0:Tsim,uCollect_rightfo(5,:),0:Tsim,uCollect_rightfo(6,:),0:Tsim,uCollect_rightfo(7,:),0:Tsim,uCollect_rightfo(8,:),'linewidth',1)
ylabel('Angle/(deg)');xlabel('Time/s');
legend('Thruster 1','Thruster 2','Thruster 3','Thruster 4');
title('Right follower')

%推力、角度变化量时历
figure(5)
duCollect_leader=zeros(8,Tsim);duCollect_leftfo=zeros(8,Tsim);duCollect_rightfo=zeros(8,Tsim);
for i=1:Tsim
duCollect_leader(:,i)=uCollect_leader(:,i+1)-uCollect_leader(:,i);
duCollect_leftfo(:,i)=uCollect_leftfo(:,i+1)-uCollect_leftfo(:,i);
duCollect_rightfo(:,i)=uCollect_rightfo(:,i+1)-uCollect_rightfo(:,i);
end
subplot(3,2,1)%leader thrust variation
plot(1:Tsim,duCollect_leader(1,:),1:Tsim,duCollect_leader(2,:),1:Tsim,duCollect_leader(3,:),1:Tsim,duCollect_leader(4,:),'linewidth',1)
hold on
plot(1:Tsim,4*ones(Tsim,1),'--k',1:Tsim,-4*ones(Tsim,1),'--k','linewidth',1)%推力变化上下限
ylabel('Thrust variation/N');
title('Leader')
subplot(3,2,2)%leader angle variation
plot(1:Tsim,duCollect_leader(5,:),1:Tsim,duCollect_leader(6,:),1:Tsim,duCollect_leader(7,:),1:Tsim,duCollect_leader(8,:),'linewidth',1)
hold on
plot(1:Tsim,15*ones(Tsim,1),'--k',1:Tsim,-15*ones(Tsim,1),'--k','linewidth',1)%推力变化上下限
ylabel('Angle variation/(deg)');
title('Leader')

subplot(3,2,3)%leftfo thrust variation
plot(1:Tsim,duCollect_leftfo(1,:),1:Tsim,duCollect_leftfo(2,:),1:Tsim,duCollect_leftfo(3,:),1:Tsim,duCollect_leftfo(4,:),'linewidth',1)
hold on
plot(1:Tsim,4*ones(Tsim,1),'--k',1:Tsim,-4*ones(Tsim,1),'--k','linewidth',1)%推力变化上下限
ylabel('Thrust variation/N');
title('Left follower')
subplot(3,2,4)%leftfo angle variation
plot(1:Tsim,duCollect_leftfo(5,:),1:Tsim,duCollect_leftfo(6,:),1:Tsim,duCollect_leftfo(7,:),1:Tsim,duCollect_leftfo(8,:),'linewidth',1)
hold on
plot(1:Tsim,15*ones(Tsim,1),'--k',1:Tsim,-15*ones(Tsim,1),'--k','linewidth',1)%推力变化上下限
ylabel('Angle variation/(deg)');
title('Left follower')

subplot(3,2,5)%rightfo thrust variation
plot(1:Tsim,duCollect_rightfo(1,:),1:Tsim,duCollect_rightfo(2,:),1:Tsim,duCollect_rightfo(3,:),1:Tsim,duCollect_rightfo(4,:),'linewidth',1)
hold on
plot(1:Tsim,4*ones(Tsim,1),'--k',1:Tsim,-4*ones(Tsim,1),'--k','linewidth',1)%推力变化上下限
ylabel('Thrust variation/N');xlabel('Time/s')
title('Right follower')
subplot(3,2,6)%rightfo angle variation
plot(1:Tsim,duCollect_rightfo(5,:),1:Tsim,duCollect_rightfo(6,:),1:Tsim,duCollect_rightfo(7,:),1:Tsim,duCollect_rightfo(8,:),'linewidth',1)
hold on
plot(1:Tsim,15*ones(Tsim,1),'--k',1:Tsim,-15*ones(Tsim,1),'--k','linewidth',1)%推力变化上下限
ylabel('Angle variation/(deg)');xlabel('Time/s')
legend('Thruster 1','Thruster 2','Thruster 3','Thruster 4')
title('Right follower')

%三条船的合力矩
figure(6)
subplot(3,1,1)
plot(1:Tsim,tauCollect_leader(1,:),1:Tsim,tauCollect_leftfo(1,:),1:Tsim,tauCollect_rightfo(1,:),'linewidth',1)
ylabel('Fx/N');title('Generalized forces (moment)')
subplot(3,1,2)
plot(1:Tsim,tauCollect_leader(2,:),1:Tsim,tauCollect_leftfo(2,:),1:Tsim,tauCollect_rightfo(2,:),'linewidth',1)
ylabel('Fy/N')
subplot(3,1,3)
plot(1:Tsim,tauCollect_leader(3,:),1:Tsim,tauCollect_leftfo(3,:),1:Tsim,tauCollect_rightfo(3,:),'linewidth',1)
ylabel('M/Nm');xlabel('Step')
legend('Leader','Left follower','Right follower')

%三条船的线速度和角速度
figure(7)
subplot(3,1,1)
plot(0:Tsim,velocityCollect_leader(1,:),0:Tsim,velocityCollect_leftfo(1,:),0:Tsim,velocityCollect_rightfo(1,:),'linewidth',1)
ylabel('u(m/s)');title('Linear and angular velocities of ASVs')
subplot(3,1,2)
plot(0:Tsim,velocityCollect_leader(2,:),0:Tsim,velocityCollect_leftfo(2,:),0:Tsim,velocityCollect_rightfo(2,:),'linewidth',1)
ylabel('v(m/s)');
subplot(3,1,3)
plot(0:Tsim,velocityCollect_leader(3,:),0:Tsim,velocityCollect_leftfo(3,:),0:Tsim,velocityCollect_rightfo(3,:),'linewidth',1)
ylabel('r(rad/s)');xlabel('Step');
legend('Leader','Left follower','Right follower')

%每一步运行时间
figure(8)
plot(1:Tsim,timeCollect(1:Tsim),'linewidth',1)
ylabel('Computation time (s)')
xlabel('Step')

% %三条船舶与静态障碍物的距离
% figure(9)
% dist_obsStatic_leader=dist_obstacle(OutputCollect_leader,pos_obstacle_static(:,1));
% dist_obsStatic_leftfo=dist_obstacle(OutputCollect_leftfo,pos_obstacle_static(:,1));
% dist_obsStatic_rightfo=dist_obstacle(OutputCollect_rightfo,pos_obstacle_static(:,1));
% plot(1:Tsim+1,0.45*ones(1,size(dist_obsStatic_leader,2)),1:Tsim+1,dist_obsStatic_leader,1:Tsim+1,dist_obsStatic_leftfo,1:Tsim+1,dist_obsStatic_rightfo)
% legend('safe distance','leader-static obstacle','leftfo-static obstacle','rightfo-static obstacle')
% %三条船舶与动态障碍物的距离
% figure(10)
% dist_obsDyn_leader=zeros(1,Tsim);dist_obsDyn_leftfo=zeros(1,Tsim);dist_obsDyn_rightfo=zeros(1,Tsim);
% for i=1:Tsim
%     dist_obsDyn_leader(i)=sqrt((OutputCollect_leader(1,i)-posObsDynCollect(1,i))^2+(OutputCollect_leader(2,i)-posObsDynCollect(2,i))^2);
%     dist_obsDyn_leftfo(i)=sqrt((OutputCollect_leftfo(1,i)-posObsDynCollect(1,i))^2+(OutputCollect_leftfo(2,i)-posObsDynCollect(2,i))^2);
%     dist_obsDyn_rightfo(i)=sqrt((OutputCollect_rightfo(1,i)-posObsDynCollect(1,i))^2+(OutputCollect_rightfo(2,i)-posObsDynCollect(2,i))^2);
% end
% plot(1:Tsim,0.45*ones(1,size(dist_obsDyn_leader,2)),1:Tsim,dist_obsDyn_leader,1:Tsim,dist_obsDyn_leftfo,1:Tsim,dist_obsDyn_rightfo)
% legend('safe distance','leader-dynamic obstacle','leftfo-dynamic obstacle','rightfo-dynamic obstacle')
%%
%save('Results_CIMPC_20211110_原参考路径_fault')