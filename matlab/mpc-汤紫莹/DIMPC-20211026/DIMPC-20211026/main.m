%%%%%-----Distributed MPC & control-allocation integration-----%%%%%
%%%%%     2021.9.28
clear;close all;
%% ȫ�ֱ���
global M;global D;global N;global thrConf;
global G;global C;
global Q_err %���Ȩֵ���� 3x3
global Q_in  %����Ȩֵ���� 8x8 ʹ��������С
global Q_dinleader %����Ȩֵ���� 8x8 ʹ���������Ƕȱ仯����С
global Q_dinfo
global Q_fm  %���Ȩֵ���� 3x3
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
global terCoe  %�ն˴��۵ı���ϵ��

global obstacleExist

%% ����ģ��
%�����е����˴� ��ģ �߶ȱ�1:4
M=[21.67 0 0;0 39.08 0;0 0 14.56];%��������
D=[23.52 0 0;0 22.32 0;0 0 3.762];%�������

% %CyberShip II
% M=[25.8 0 0;0 33.8 1;0 1 2.8];%��������
% D=[0.72 0 0;0 0.89 0.03;0 0.03 1.9];%�������

N=4;%ÿ�������ƽ�������
%��ģ
%thrConf=[0.4 0.2;0.4 -0.2;-0.4 -0.2;-0.4 0.2];%�ĸ��ƽ�������ڴ����ĵ�������� [lx1 ly1;lx2 ly2;lx3 ly3;lx4 ly4]  
thrConf=[0.4 0.2;-0.4 0.2;-0.4 -0.2;0.4 -0.2];%��������ϵ��
% %ʵ��
% thrConf=[1.8 0.9;1.8 -0.9;-1.8 -0.9;-1.8 0.9];

G=[zeros(3);inv(M)];
C=[eye(3) zeros(3)];
%% DMPC���Ʋ���
%Ȩֵ����ȡֵ
Q_err=diag([250 250 250]);%�ʵ��������Ȩ�ص���һ��
terCoe=20;%terminal cost��Ȩֵϵ��
Q_in=[1*eye(4) zeros(4);zeros(4) zeros(4)];%ʹ��������С��Ȩֵ���� ֻ�����Ͻ���ֵ
Q_dinleader=diag([1 1 1 1 50 50 50 50]);
Q_dinfo=diag([50 50 50 50 300 300 300 300]);%ʹ���������Ƕȱ仯����С��Ȩֵ����
Q_fm=diag([750 750 750]);%�ʵ��������Ȩ�ص���һ��

%�����Np Ncָ����Ԥ��Ϳ��Ʋ���������ʱ��
Np=3;
Nc=2;
Ts=2;%��ɢ��� discretization interval
Tsim=300;
obstacleExist=0;%1 �����ϰ��� 0 �������ϰ���

%���������
iterMax=25;%��Ϣ�����������
%errorU_Tol=[0.1;0.1;0.1;0.1;0.08;0.08;0.08;0.08];%���tolerance �����ͽǶȵ�tolerance��һ��
errorU_Tol=[0.2;0.2;0.2;0.2;0.04;0.04;0.04;0.04];

%% ����Ȩֵ����
[Q_err_leader,Q_in_leader,Q_din_leader,Q_in_fo_firstep,Q_din_fo_firstep,Q_fm_fo_firstep,Q_in_fo_substep,Q_din_fo_substep,Q_fm_fo_substep]=createWeightMat();

%% �����غɼ��� ��������ϵ��
% %������ʱ�ѻ���������������Ϊ�����ص�0.6��
% %��������Ļ����� ��ʵ�Ǹ�����/����� ���Ǹ��ݴ����˶������õ���/���߸�������ֵ�ͺ������˹�ϵ�õ���
% scale_w=0.05;%��������
% w_real=zeros(3,Tsim);
% for i=1:Tsim
% w_real(:,i)=[10+1.8*sin(0.7*i)+1.2*sin(0.05*i)+1.2*sin(0.9*i);...
%          5+0.4*sin(0.1*i)+0.2*cos(0.6*i);...
%          0]*scale_w;
% end

%�������������Լ��������˹�ϵ����������غ�
WaveDirection=180;%���˺���
WindVelocity=5;%m/s
CurrentVelocity=0.512;%m/s
w_scale=[0.02;0.02;0];%�൱�ڳ߶ȱ�1:50
w_real_seed=EnvLoadsCal(WaveDirection,WindVelocity,CurrentVelocity)'.*w_scale;
w_real=repmat(w_real_seed,1,Tsim);

%��ʱ������Ŷ���w��ΪRBFNNԤ����w_estimated����ƪ�������������ʱ����������Ԥ��������������
w_estimated=w_real+[0;0;0].*((-1)+(1+1)*rand(3,Tsim));

% %test��ͼ
% plot(1:Tsim,w,1:Tsim,w_estimated);
%% ��������
scale_v_position=0.01;scale_v_angle=0.0;
v=zeros(3,Tsim);
%v=normrnd(0,1,[3,Tsim]);
for i=1:Tsim
    v(1:2,i)=normrnd(0,1,[2,1]);%��ֵ=0������=�ڶ�����^2
    v(3,i)=normrnd(0,1,[1,1]);
end
v(1:2,:)=v(1:2,:)*scale_v_position;v(3,:)=v(3,:)*scale_v_angle;

%% �ο��켣
%situation map
load ref_map_3 %���0.1m ���߱�1:500 ��75m���30m
yy=1300-yy;%���ط�ת
xx=xx*30/1900;%������ת�������� m(���߱�1:200 ��75m��
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
% %static
% pos_obstacle_static=repmat(r(1:2,35),1,Np);
% 
% 
% %dynamic �زο��켣
% temp=r_ori(1:2,108:118);temp(2,:)=temp(2,:)+0.2;
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
%������ʼ״̬
% ref_map_3�ĳ�ʼֵ
x0_leader=[3;4.4;deg2rad(-20);0;0;0];
x0_leftfo=[3;5.4;deg2rad(-20);0;0;0];
x0_rightfo=[3;3.4;deg2rad(-20);0;0;0];

x0e_leader=x0_leader;
x0e_leftfo=x0_leftfo;
x0e_rightfo=x0_rightfo;

y0_leader=C*x0_leader;
y0_leftfo=C*x0_leftfo;
y0_rightfo=C*x0_rightfo;

%�����������ͽǶȵĳ�ʼֵ ����ĽǶ�����������ϵ�µ�
%situation 1
u0_leader=[0;0;0;0;deg2rad(0);deg2rad(0);deg2rad(0);deg2rad(0)];
u0_leftfo=[0;0;0;0;deg2rad(0);deg2rad(0);deg2rad(0);deg2rad(0)];
u0_rightfo=[0;0;0;0;deg2rad(0);deg2rad(0);deg2rad(0);deg2rad(0)];

U0_leader=repmat(u0_leader,Nc,1);
U0_leftfo=repmat(u0_leftfo,Nc,1);
U0_rightfo=repmat(u0_rightfo,Nc,1);

%% ʱ��ģ��
%collect information
%leader
OutputCollect_leader=y0_leader;
velocityCollect_leader=x0_leader(4:6);
uCollect_leader=u0_leader;
tauCollect_leader=[];
indexCollect=[];
%follower
OutputCollect_leftfo=y0_leftfo;
velocityCollect_leftfo=x0_leftfo(4:6);
uCollect_leftfo=u0_leftfo;
tauCollect_leftfo=[];

OutputCollect_rightfo=y0_rightfo;
velocityCollect_rightfo=x0_rightfo(4:6);
uCollect_rightfo=u0_rightfo;
tauCollect_rightfo=[];
%iteration
iterCollect=[];
iterTime=zeros(1,Tsim);
errorUCollect=[];

%simulation
for t=1:Tsim
    t %�鿴����ʱ�䲽
    
    %�ҵ�leaderǰ������Ĳο�·��������
    index=findNearestPoint(y0_leader(1:2),r(1:2,:));
    index_aft = index + Np - 1;
    if index_aft > length(r(1, :))
        Yd_leader = [r(:, index : end), repmat(r(:, end), 1, index_aft - length(r(1, :)) )];
    else
        Yd_leader=r(:, index:index+Np-1);
    end
    indexCollect=[indexCollect index];
    
    %���ݱ�ӿ��Ʋ�������follower������·��������
    Yd_leftfo=desiredOutputCal_follower(Yd_leader,range12,bearing12,1);
    Yd_rightfo=desiredOutputCal_follower(Yd_leader,range13,bearing13,2);

    %MPC controller for leader ship �����ڲ�ѭ��������
    [x_leader,xe_leader,y_leader,U_leaderself,tau_real_leader]=controller_leader(U0_leader,x0_leader,x0e_leader,w_real(:,t),w_estimated(:,t),v(:,t),Yd_leader);
    
    tic;%����ÿ�ε���ʱ�䣨����֮����Ϣ���䲢�ﵽһ�µ�ʱ�䣩
    %MPC controller for followers
    for iter=1:iterMax
        if iter==1 %��һ�� ȫά
            %leftfo
            [x_leftfo,xe_leftfo,y_leftfo,U_leftfoself,U_leaderbyleftfo,U_rightfobyleftfo,tau_leftfo]=controller_fo_firstep([U0_leftfo;U0_leader;U0_rightfo],[x0_leftfo;x0_leader;x0_rightfo],[x0e_leftfo;x0e_leader;x0e_rightfo],repmat(w_real(:,t),3,1),repmat(w_estimated(:,t),3,1),repmat(v(:,t),3,1),[Yd_leftfo;Yd_leader;Yd_rightfo]);
            
            %rightfo
            [x_rightfo,xe_rightfo,y_rightfo,U_rightfoself,U_leaderbyrightfo,U_leftfobyrightfo,tau_rightfo]=controller_fo_firstep([U0_rightfo;U0_leader;U0_leftfo],[x0_rightfo;x0_leader;x0_leftfo],[x0e_rightfo;x0e_leader;x0e_leftfo],repmat(w_real(:,t),3,1),repmat(w_estimated(:,t),3,1),repmat(v(:,t),3,1),[Yd_rightfo;Yd_leader;Yd_leftfo]);

            %�ж��Ƿ�����
            errorU=abs([U_leaderself-U_leaderbyleftfo;U_leaderself-U_leaderbyrightfo;U_leftfoself-U_leftfobyrightfo;U_rightfoself-U_rightfobyleftfo]);
            if t==1 %����t=1ʱ�ռ������������
            errorUCollect=[errorUCollect errorU];
            end
            if errorU<=repmat(errorU_Tol,4*Nc,1)
               break
            else
                %��Ϣ����
                U_leaderbyleftfo=U_leaderself;
                U_leaderbyrightfo=U_leaderself;
                U_leftfobyrightfo=U_leftfoself;
                U_rightfobyleftfo=U_rightfoself;
            end  
        elseif iter>=2 %�ڶ�����ʼ ��ά
            %leftfo
            [x_leftfo,xe_leftfo,y_leftfo,U_leftfoself,tau_leftfo]=controller_fo_substep([U0_leftfo;U0_leader;U0_rightfo],U_leaderbyleftfo,U_rightfobyleftfo,[x0_leftfo;x0_leader;x0_rightfo],[x0e_leftfo;x0e_leader;x0e_rightfo],repmat(w_real(:,t),3,1),repmat(w_estimated(:,t),3,1),repmat(v(:,t),3,1),[Yd_leftfo;Yd_leader;Yd_rightfo]);
            %rightfo
            [x_rightfo,xe_rightfo,y_rightfo,U_rightfoself,tau_rightfo]=controller_fo_substep([U0_rightfo;U0_leader;U0_leftfo],U_leaderbyrightfo,U_leftfobyrightfo,[x0_rightfo;x0_leader;x0_leftfo],[x0e_rightfo;x0e_leader;x0e_leftfo],repmat(w_real(:,t),3,1),repmat(w_estimated(:,t),3,1),repmat(v(:,t),3,1),[Yd_rightfo;Yd_leader;Yd_leftfo]);
             
            %�ж��Ƿ�����
            errorU=abs([U_leaderself-U_leaderbyleftfo;U_leaderself-U_leaderbyrightfo;U_leftfoself-U_leftfobyrightfo;U_rightfoself-U_rightfobyleftfo]);
            if t==1 %����t=1ʱ�ռ������������
            errorUCollect=[errorUCollect errorU];
            end
            if errorU<=repmat(errorU_Tol,4*Nc,1)
               break
            else
                %��Ϣ����
                U_leftfobyrightfo=U_leftfoself;
                U_rightfobyleftfo=U_rightfoself;
            end      
        end
    end %end iter
    iterTime(t)=toc*0.5;%ʵ�ʹ����п���������follower���м��� ���*0.5
    iterCollect=[iterCollect iter];

    %����leader��ʼֵ
    %leader
    U0_leader=U_leaderself;
    x0_leader=x_leader;
    x0e_leader=xe_leader;
    y0_leader=y_leader;
 
    %����leftfo û�г��ֹ���
    U0_leftfo=U_leftfoself;
    x0_leftfo=x_leftfo;
    x0e_leftfo=xe_leftfo;

%     %����leftfo ���ֹ���
%     if t>=115 && t<=125
%         U0_leftfo=U_leftfobyrightfo;
%     else
%         U0_leftfo=U_leftfoself;
%     end
%     x0_leftfo=x_leftfo;
%     x0e_leftfo=xe_leftfo;
    
    %����rightfo
    U0_rightfo=U_rightfoself;
    x0_rightfo=x_rightfo;
    x0e_rightfo=xe_rightfo;
    
    %collect information
    %leader
    OutputCollect_leader=[OutputCollect_leader y_leader];
    velocityCollect_leader=[velocityCollect_leader x_leader(4:6)];
    uCollect_leader=[uCollect_leader U_leaderself(1:8)];
    tauCollect_leader=[tauCollect_leader tau_real_leader];
    %follower
    OutputCollect_leftfo=[OutputCollect_leftfo y_leftfo];
    velocityCollect_leftfo=[velocityCollect_leftfo x_leftfo(4:6)];
    uCollect_leftfo=[uCollect_leftfo U_leftfoself(1:8)];
    tauCollect_leftfo=[tauCollect_leftfo,tau_leftfo];
    
    OutputCollect_rightfo=[OutputCollect_rightfo y_rightfo];
    velocityCollect_rightfo=[velocityCollect_rightfo x_rightfo(4:6)];
    uCollect_rightfo=[uCollect_rightfo U_rightfoself(1:8)];
    tauCollect_rightfo=[tauCollect_rightfo,tau_rightfo];
    
    %ʵʱ�鿴���
    leader=[y_leader(1) y_leader(2) rad2deg(y_leader(3))]
    leftfo=[y_leftfo(1) y_leftfo(2) rad2deg(y_leftfo(3))]
    rightfo=[y_rightfo(1) y_rightfo(2) rad2deg(y_rightfo(3))]

    dist_leader_leftfo=sqrt((y_leader(1)-y_leftfo(1))^2+(y_leader(2)-y_leftfo(2))^2)
    dist_leader_rightfo=sqrt((y_leader(1)-y_rightfo(1))^2+(y_leader(2)-y_rightfo(2))^2)
    dist_leftfo_rightfo=sqrt((y_leftfo(1)-y_rightfo(1))^2+(y_leftfo(2)-y_rightfo(2))^2)

end

%% ��ͼ
%��������·������
figure(1)
plot(r(1,:),r(2,:),'k--')
hold on
plot(OutputCollect_leader(1,:),OutputCollect_leader(2,:),OutputCollect_leftfo(1,:),OutputCollect_leftfo(2,:),OutputCollect_rightfo(1,:),OutputCollect_rightfo(2,:),'linewidth',1)
xlabel('X/m');ylabel('Y/m');
title('path following of three vessels')

%����֮��ı�ӱ���(�������봬����Ծ��룬�����ƫ��)
figure(2)
formation_leader_leftfo=formationCal(OutputCollect_leader(1:3,:),OutputCollect_leftfo);
formation_leader_leftfo(1,:)=formation_leader_leftfo(1,:)-range12;

formation_leader_rightfo=formationCal(OutputCollect_leader(1:3,:),OutputCollect_rightfo);
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
xlabel('Step');ylabel('Yaw angle error/deg');legend('Desired error','Leader-left follower','Leader-right follower')

%leader�������ʱ��  
figure(3)
objPath=zeros(3,Tsim);
% indexCollect(1)=1;%�ѵ�һ��ֵ���ɵ�һ��·����
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

%����֮����Ϣ������� 
figure(4)
yyaxis left
bar(1:Tsim,iterCollect,'facecolor',[0.8,0.8,0.8],'edgecolor','none');
ylabel('Iteration number')
yyaxis right
plot(1:Tsim,iterTime(1:Tsim),'linewidth',1)
ylabel('Iteration time')
xlabel('Step')

figure(5)
errorUCollect_norm=mapminmax(errorUCollect,0,1);%��һ��
errorUCollect_mean=mean(errorUCollect_norm);
for k=1:size(errorUCollect_mean,2)
    scatter(k*ones(2*N*Nc*4,1),errorUCollect_norm(:,k),20,'b')
    hold on
end
line=plot(1:k,errorUCollect_mean,'r','linewidth',1);
xlabel('Iteration');ylabel('Normed error');legend(line,'Mean error')

%�������Ƕ�ʱ������������
figure(6)
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
ylabel('Thrust/N');xlabel('Step')
title('Right follower')
subplot(3,2,6)%rightfo �����
plot(0:Tsim,uCollect_rightfo(5,:),0:Tsim,uCollect_rightfo(6,:),0:Tsim,uCollect_rightfo(7,:),0:Tsim,uCollect_rightfo(8,:),'linewidth',1)
ylabel('Angle/(deg)');xlabel('Step');
legend('Thruster 1','Thruster 2','Thruster 3','Thruster 4');
title('Right follower')

%�������Ƕȱ仯ʱ�� ������
figure(7)
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
ylabel('Thrust variation/N');xlabel('Step')
title('Right follower')
subplot(3,2,6)%rightfo angle variation
plot(1:Tsim,duCollect_rightfo(5,:),1:Tsim,duCollect_rightfo(6,:),1:Tsim,duCollect_rightfo(7,:),1:Tsim,duCollect_rightfo(8,:),'linewidth',1)
hold on
plot(1:Tsim,15*ones(Tsim,1),'--k',1:Tsim,-15*ones(Tsim,1),'--k','linewidth',1)%�����仯������
ylabel('Angle variation/(deg)');xlabel('Step')
legend('Thruster 1','Thruster 2','Thruster 3','Thruster 4')
title('Right follower')

%�������أ�ʱ��
figure(8)
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

%���������ٶȺͽ��ٶȣ���������
figure(9)
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

%% ������
%save('results')