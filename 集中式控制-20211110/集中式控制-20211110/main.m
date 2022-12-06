%%%%%------Centralized MPC------%%%%%
%%%%%     2021.10.07
clear;close all;

%% ȫ�ֱ���
global M;global D;global N;global thrConf;
global G;global C;
global Q_err %���Ȩֵ���� 3x3
global Q_in  %����Ȩֵ���� 8x8 ʹ��������С
global Q_din %����Ȩֵ���� 8x8 ʹ���������Ƕȱ仯����С 
global Q_fm  %���Ȩֵ���� 3x3
global Ts;global Tsim;global t;
global Np;global Nc;

global Q_IN
global Q_dIN
global Q_Err
global terCoe
global Q_obs_avoidance
global gamma %����Ŀ�꺯���е�����
% global pos_obstacle_static
% global pos_obstacle_dynamic
global obstacleExist

global R;%����������Э�������
%% ����ģ��
%�����е����˴�
M=[21.67 0 0;0 39.08 0;0 0 14.56];%��������
D=[23.52 0 0;0 22.32 0;0 0 3.762];%�������

% %CyberShip II
% M=[25.8 0 0;0 33.8 1;0 1 2.8];%��������
% D=[0.72 0 0;0 0.89 0.03;0 0.03 1.9];%�������

N=4;%ÿ�������ƽ�������
thrConf=[0.4 0.2;-0.4 0.2;-0.4 -0.2;0.4 -0.2];%�ĸ��ƽ�������ڴ����ĵ�������� [lx1 ly1;lx2 ly2;lx3 ly3;lx4 ly4]    

G=[zeros(3);inv(M)];
C=[eye(3) zeros(3)];
%% CMPC���Ʋ���
% Q_err=diag([150 150 150]);%�ʵ��������Ȩ�ص���һ��
% terCoe=20;%terminal cost��Ȩֵϵ��
% Q_in=[1*eye(4) zeros(4);zeros(4) zeros(4)];%ʹ��������С��Ȩֵ���� ֻ�����Ͻ���ֵ
% Q_din=diag([1 1 1 1 150 150 150 150]);%ʹ���������Ƕȱ仯����С��Ȩֵ����
% Q_fm=diag([150 150 150]);%�ʵ��������Ȩ�ص���һ��
% Q_obs_avoidance=2;
% gamma=0;

Q_err=diag([250 250 250]);%�ʵ��������Ȩ�ص���һ��
terCoe=20;%terminal cost��Ȩֵϵ��
Q_in=[1*eye(4) zeros(4);zeros(4) zeros(4)];%ʹ��������С��Ȩֵ���� ֻ�����Ͻ���ֵ
Q_din=diag([1 1 1 1 150 150 150 150]);%ʹ���������Ƕȱ仯����С��Ȩֵ����
Q_fm=diag([750 750 750]);%�ʵ��������Ȩ�ص���һ��
Q_obs_avoidance=2;
gamma=0;

%Np Ncָ����Ԥ��Ϳ��Ʋ���
Np=3;
Nc=2;
Ts=2;%��ɢ���
Tsim=300;
obstacleExist=0;

%�Ƿ���״̬�۲���
StateEst=0;%1---���� 0---�ر�
%% ����Ȩֵ����
[Q_IN,Q_dIN,Q_Err]=createWeightMat();
%% ��������
%�������������Լ��������˹�ϵ����������غ�
WaveDirection=180;%���˺���
WindVelocity=5;%m/s
CurrentVelocity=0.512;%m/s
w_scale=[0.02;0.02;0];%�൱�ڳ߶ȱ�1:50
w_real_seed=EnvLoadsCal(WaveDirection,WindVelocity,CurrentVelocity)'.*w_scale;
w_real=repmat(w_real_seed,1,Tsim);

%��ʱ������Ŷ���w��ΪRBFNNԤ����w_estimated
%��ƪ�������������ʱ����������Ԥ��������������
w_estimated=w_real+[0;0;0].*((-1)+(1+1)*rand(3,Tsim));

%% ��������
scale_v_position=0.01;scale_v_angle=0.0;
v=zeros(3,Tsim);
%v=normrnd(0,1,[3,Tsim]);
for i=1:Tsim
    v(1:2,i)=normrnd(0,1,[2,1]);%��ֵ=0������=�ڶ�����^2
    v(3,i)=normrnd(0,1,[1,1]);
end
v(1:2,:)=v(1:2,:)*scale_v_position;v(3,:)=v(3,:)*scale_v_angle;
R=1*eye(3);
%% �ο��켣
%map
% load ref_map %0.2m���·����
load ref_map_3 %0.1m��� ���߱�1:500
yy=1300-yy;%���ط�ת
xx=xx*30/1900;
yy=yy*(30*13/19)/1300;
r=zeros(3,size(xx,2));
r(1,:)=xx;r(2,:)=yy;
for i=1:size(r,2)
    if i==1
        r(3,i)=deg2rad(-20); %���
    elseif i==size(r,2)
        r(3,i)=deg2rad(-44.5); %�յ�
    else
    r(3,i)=atan((r(2,i+1)-r(2,i-1))/(r(1,i+1)-r(1,i-1)));%��λ rad
    end
end

%% obstacle
% load r_ori
% % static obstacle
% pos_obstacle_static=repmat(r_ori(1:2,35),1,Np);%��̬�ϰ���λ��
% 
% %��̬�ϰ����һ������ ��������
% % pos_obstacle_dynamic=zeros(2,Np);
% % for j=1:Np
% %     pos_obstacle_dynamic(1,j)=9;
% %     pos_obstacle_dynamic(2,j)=5*sin((t+j-1)*pi/20+pi/2)+5.5;%20Ϊ�Ӱ�����һ�˵�step���������ʵ�����������sinǰ��ı�����Ҫ��������һ��
% % end
% % pos_obstacle_dynamic_upboundary=[9;10.5];
% % pos_obstacle_dynamic_downboundary=[9;0.5];
% 
% %��̬�ϰ���ڶ������� �زο��켣
% temp=r_ori(1:2,108:118);
% temp(2,:)=temp(2,:)+0.2;%������0.2m
% pos_obstacle_dynamic_upboundary=temp(:,1);
% pos_obstacle_dynamic_downboundary=temp(:,end);
% pos_obstacle_dynamic_ref=repmat(cat(2,temp,fliplr(temp)),1,100);
% pos_obstacle_dynamic=pos_obstacle_dynamic_ref(:,t:t+Np-1);

%% ��Ӳ��� ���ڼ���follower������·����
range12=1;
range13=1;
bearing12=deg2rad(0);
bearing13=deg2rad(0);
%% ������ʼֵ
%situation huangpu river
% ref-_map_3�ĳ�ʼֵ ���2m
x0_leader=[3;4.4;deg2rad(-20);0;0;0];
x0_leftfo=[3;5.4;deg2rad(-20);0;0;0];
x0_rightfo=[3;3.4;deg2rad(-20);0;0;0];

x0e_leader=x0_leader;
x0e_leftfo=x0_leftfo;
x0e_rightfo=x0_rightfo;

y0_leader=C*x0_leader;
y0_leftfo=C*x0_leftfo;
y0_rightfo=C*x0_rightfo;

x0est_leader=x0_leader;%��ʼ״̬����ֵ ֻ��leaderʹ��״̬����
y0est_leader=y0_leader;%��ʼԤ�����

u0_leader=[0;0;0;0;deg2rad(0);deg2rad(0);deg2rad(0);deg2rad(0)];
u0_leftfo=[0;0;0;0;deg2rad(0);deg2rad(0);deg2rad(0);deg2rad(0)];
u0_rightfo=[0;0;0;0;deg2rad(0);deg2rad(0);deg2rad(0);deg2rad(0)];
U0_leader=repmat(u0_leader,Nc,1);
U0_leftfo=repmat(u0_leftfo,Nc,1);
U0_rightfo=repmat(u0_rightfo,Nc,1);

%��ʼ���Э�������
P0=0.5*eye(6);
%% ʱ��ģ��
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
    t %�鿴���н���
    
    %�ҵ�leaderǰ������Ĳο�·��������
    index=findNearestPoint(y0_leader(1:2),r(1:2,:));
    Yd_leader=r(:,index:index+Np-1);
    indexCollect=[indexCollect index];
    
    %���ݱ�ӿ��Ʋ�������follower������·��������
    Yd_leftfo=desiredOutputCal_follower(Yd_leader,range12,bearing12,1);
    Yd_rightfo=desiredOutputCal_follower(Yd_leader,range13,bearing13,2);
    
    %Global controller
    [x_leader,xe_leader,y_leader,U_leader,tau_leader,...
    x_leftfo,xe_leftfo,y_leftfo,U_leftfo,tau_leftfo,...
    x_rightfo,xe_rightfo,y_rightfo,U_rightfo,tau_rightfo]=global_controller([U0_leader;U0_leftfo;U0_rightfo],[x0_leader;x0_leftfo;x0_rightfo],[x0e_leader;x0e_leftfo;x0e_rightfo],repmat(w_estimated(:,t),3,1),repmat(v(:,t),3,1),[Yd_leader;Yd_leftfo;Yd_rightfo]);

    
    %������Ϣ
    %leader
    U0_leader=U_leader;
    x0_leader=x_leader;
    x0e_leader=xe_leader;
    y0_leader=y_leader;
    
    %leftfo
    %leftfo������Ϣȱʧ
    if t>=115 && t<=125
        U0_leftfo=zeros(16,1);
    else
        U0_leftfo=U_leftfo;
    end
    x0_leftfo=x_leftfo;
    x0e_leftfo=xe_leftfo;
%     %leftfo����
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
   
    
    %�鿴��Ϣ
    y_leader
    y_leftfo
    y_rightfo
end

%% picture
%��������·������
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

%leader�������
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

%������
figure(3)
formation_leader_leftfo=formationCal(OutputCollect_leader,OutputCollect_leftfo);
formation_leader_leftfo(1,:)=formation_leader_leftfo(1,:)-range12;
formation_leader_rightfo=formationCal(OutputCollect_leader,OutputCollect_rightfo);
formation_leader_rightfo(1,:)=formation_leader_rightfo(1,:)-range13;
subplot(2,1,1)%���뱣��
plot(0:Tsim,zeros(1,Tsim+1),'k--')
hold on
plot(0:Tsim,formation_leader_leftfo(1,:),0:Tsim,formation_leader_rightfo(1,:),'linewidth',1)
ylabel('Distance error/m');legend('Desired error','Leader-left follower','Leader-right follower')
title('Formation error')
subplot(2,1,2)%�����ƫ��
plot(0:Tsim,zeros(1,Tsim+1),'k--')
hold on
plot(0:Tsim,formation_leader_leftfo(2,:),0:Tsim,formation_leader_rightfo(2,:),'linewidth',1)
xlabel('Time/s');ylabel('Yaw angle error/deg');legend('Desired error','Leader-left follower','Leader-right follower')

%�������Ƕ�ʱ�� ������
figure(4)
uCollect_leader(5:8,:)=rad2deg(uCollect_leader(5:8,:));
uCollect_leftfo(5:8,:)=rad2deg(uCollect_leftfo(5:8,:));
uCollect_rightfo(5:8,:)=rad2deg(uCollect_rightfo(5:8,:));
subplot(3,2,1)%leader ����
plot(0:Tsim,uCollect_leader(1,:),0:Tsim,uCollect_leader(2,:),0:Tsim,uCollect_leader(3,:),0:Tsim,uCollect_leader(4,:),'linewidth',1)
hold on
plot(0:Tsim,20*ones(Tsim+1,1),'--k',0:Tsim,-20*ones(Tsim+1,1),'--k','linewidth',1)%����������
ylabel('Thrust/N');
title('Leader')
subplot(3,2,2)%leader �����
plot(0:Tsim,uCollect_leader(5,:),0:Tsim,uCollect_leader(6,:),0:Tsim,uCollect_leader(7,:),0:Tsim,uCollect_leader(8,:),'linewidth',1)
ylabel('Angle/(deg)')
title('Leader')

subplot(3,2,3)%leftfo ����
plot(0:Tsim,uCollect_leftfo(1,:),0:Tsim,uCollect_leftfo(2,:),0:Tsim,uCollect_leftfo(3,:),0:Tsim,uCollect_leftfo(4,:),'linewidth',1)
hold on
plot(0:Tsim,20*ones(Tsim+1,1),'--k',0:Tsim,-20*ones(Tsim+1,1),'--k','linewidth',1)%����������
ylabel('Thrust/N');
title('Left follower')
subplot(3,2,4)%leftfo �����
plot(0:Tsim,uCollect_leftfo(5,:),0:Tsim,uCollect_leftfo(6,:),0:Tsim,uCollect_leftfo(7,:),0:Tsim,uCollect_leftfo(8,:),'linewidth',1)
ylabel('Angle/(deg)')
title('Left follower')

subplot(3,2,5)%rightfo ����
plot(0:Tsim,uCollect_rightfo(1,:),0:Tsim,uCollect_rightfo(2,:),0:Tsim,uCollect_rightfo(3,:),0:Tsim,uCollect_rightfo(4,:),'linewidth',1)
hold on
plot(0:Tsim,20*ones(Tsim+1,1),'--k',0:Tsim,-20*ones(Tsim+1,1),'--k','linewidth',1)%����������
ylabel('Thrust/N');xlabel('Time/s')
title('Right follower')
subplot(3,2,6)%rightfo �����
plot(0:Tsim,uCollect_rightfo(5,:),0:Tsim,uCollect_rightfo(6,:),0:Tsim,uCollect_rightfo(7,:),0:Tsim,uCollect_rightfo(8,:),'linewidth',1)
ylabel('Angle/(deg)');xlabel('Time/s');
legend('Thruster 1','Thruster 2','Thruster 3','Thruster 4');
title('Right follower')

%�������Ƕȱ仯��ʱ��
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
plot(1:Tsim,4*ones(Tsim,1),'--k',1:Tsim,-4*ones(Tsim,1),'--k','linewidth',1)%�����仯������
ylabel('Thrust variation/N');
title('Leader')
subplot(3,2,2)%leader angle variation
plot(1:Tsim,duCollect_leader(5,:),1:Tsim,duCollect_leader(6,:),1:Tsim,duCollect_leader(7,:),1:Tsim,duCollect_leader(8,:),'linewidth',1)
hold on
plot(1:Tsim,15*ones(Tsim,1),'--k',1:Tsim,-15*ones(Tsim,1),'--k','linewidth',1)%�����仯������
ylabel('Angle variation/(deg)');
title('Leader')

subplot(3,2,3)%leftfo thrust variation
plot(1:Tsim,duCollect_leftfo(1,:),1:Tsim,duCollect_leftfo(2,:),1:Tsim,duCollect_leftfo(3,:),1:Tsim,duCollect_leftfo(4,:),'linewidth',1)
hold on
plot(1:Tsim,4*ones(Tsim,1),'--k',1:Tsim,-4*ones(Tsim,1),'--k','linewidth',1)%�����仯������
ylabel('Thrust variation/N');
title('Left follower')
subplot(3,2,4)%leftfo angle variation
plot(1:Tsim,duCollect_leftfo(5,:),1:Tsim,duCollect_leftfo(6,:),1:Tsim,duCollect_leftfo(7,:),1:Tsim,duCollect_leftfo(8,:),'linewidth',1)
hold on
plot(1:Tsim,15*ones(Tsim,1),'--k',1:Tsim,-15*ones(Tsim,1),'--k','linewidth',1)%�����仯������
ylabel('Angle variation/(deg)');
title('Left follower')

subplot(3,2,5)%rightfo thrust variation
plot(1:Tsim,duCollect_rightfo(1,:),1:Tsim,duCollect_rightfo(2,:),1:Tsim,duCollect_rightfo(3,:),1:Tsim,duCollect_rightfo(4,:),'linewidth',1)
hold on
plot(1:Tsim,4*ones(Tsim,1),'--k',1:Tsim,-4*ones(Tsim,1),'--k','linewidth',1)%�����仯������
ylabel('Thrust variation/N');xlabel('Time/s')
title('Right follower')
subplot(3,2,6)%rightfo angle variation
plot(1:Tsim,duCollect_rightfo(5,:),1:Tsim,duCollect_rightfo(6,:),1:Tsim,duCollect_rightfo(7,:),1:Tsim,duCollect_rightfo(8,:),'linewidth',1)
hold on
plot(1:Tsim,15*ones(Tsim,1),'--k',1:Tsim,-15*ones(Tsim,1),'--k','linewidth',1)%�����仯������
ylabel('Angle variation/(deg)');xlabel('Time/s')
legend('Thruster 1','Thruster 2','Thruster 3','Thruster 4')
title('Right follower')

%�������ĺ�����
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

%�����������ٶȺͽ��ٶ�
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

%ÿһ������ʱ��
figure(8)
plot(1:Tsim,timeCollect(1:Tsim),'linewidth',1)
ylabel('Computation time (s)')
xlabel('Step')

% %���������뾲̬�ϰ���ľ���
% figure(9)
% dist_obsStatic_leader=dist_obstacle(OutputCollect_leader,pos_obstacle_static(:,1));
% dist_obsStatic_leftfo=dist_obstacle(OutputCollect_leftfo,pos_obstacle_static(:,1));
% dist_obsStatic_rightfo=dist_obstacle(OutputCollect_rightfo,pos_obstacle_static(:,1));
% plot(1:Tsim+1,0.45*ones(1,size(dist_obsStatic_leader,2)),1:Tsim+1,dist_obsStatic_leader,1:Tsim+1,dist_obsStatic_leftfo,1:Tsim+1,dist_obsStatic_rightfo)
% legend('safe distance','leader-static obstacle','leftfo-static obstacle','rightfo-static obstacle')
% %���������붯̬�ϰ���ľ���
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
%save('Results_CIMPC_20211110_ԭ�ο�·��_fault')