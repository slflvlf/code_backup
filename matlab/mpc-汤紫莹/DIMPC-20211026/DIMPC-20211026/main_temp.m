clear all;  close all;
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

%% 船舶模型
%论文中的无人船 船模 尺度比1:4
M=[21.67 0 0;0 39.08 0;0 0 14.56];%质量矩阵
D=[23.52 0 0;0 22.32 0;0 0 3.762];%阻尼矩阵

% %CyberShip II
% M=[25.8 0 0;0 33.8 1;0 1 2.8];%质量矩阵
% D=[0.72 0 0;0 0.89 0.03;0 0.03 1.9];%阻尼矩阵

N=4;%每条船的推进器数量
%船模
%thrConf=[0.4 0.2;0.4 -0.2;-0.4 -0.2;-0.4 0.2];%四个推进器相对于船中心的坐标矩阵 [lx1 ly1;lx2 ly2;lx3 ly3;lx4 ly4]  
thrConf=[0.4 0.2;-0.4 0.2;-0.4 -0.2;0.4 -0.2];%更改坐标系后
% %实船
% thrConf=[1.8 0.9;1.8 -0.9;-1.8 -0.9;-1.8 0.9];

G=[zeros(3);inv(M)];
C=[eye(3) zeros(3)];
%% DMPC控制参数
%权值矩阵取值
Q_err=diag([250 250 250]);%适当把艏向角权重调大一点
terCoe=2;%terminal cost的权值系数
Q_in=[1*eye(4) zeros(4);zeros(4) zeros(4)];%使得推力最小的权值矩阵 只有左上角有值
Q_dinleader=diag([1 1 1 1 50 50 50 50]);
Q_dinfo=diag([50 50 50 50 300 300 300 300]);%使得推力、角度变化量最小的权值矩阵
Q_fm=diag([750 750 750]);%适当把艏向角权重调大一点

%这里的Np Nc指的是预测和控制步数，不是时间
Np=3;
Nc=2;
Ts=2;%离散间隔 discretization interval
Tsim=200;
obstacleExist=0;%1 存在障碍物 0 不存在障碍物

%迭代及误差
iterMax=25;%信息传输迭代上限
%errorU_Tol=[0.1;0.1;0.1;0.1;0.08;0.08;0.08;0.08];%误差tolerance 推力和角度的tolerance不一样
errorU_Tol=[0.2;0.2;0.2;0.2;0.04;0.04;0.04;0.04];

%% 创建权值矩阵
[Q_err_leader,Q_in_leader,Q_din_leader,Q_in_fo_firstep,Q_din_fo_firstep,Q_fm_fo_firstep,Q_in_fo_substep,Q_din_fo_substep,Q_fm_fo_substep]=createWeightMat();

%% 环境载荷计算 船体坐标系下
% %可以暂时把环境力的量级设置为合力矩的0.6倍
% %这样计算的环境力 其实是给定的/随机的 不是根据船舶运动反馈得到的/或者根据试验值和海区风浪关系得到的
% scale_w=0.05;%量级设置
% w_real=zeros(3,Tsim);
% for i=1:Tsim
% w_real(:,i)=[10+1.8*sin(0.7*i)+1.2*sin(0.05*i)+1.2*sin(0.9*i);...
%          5+0.4*sin(0.1*i)+0.2*cos(0.6*i);...
%          0]*scale_w;
% end

%根据试验数据以及海区风浪关系计算风浪流载荷
WaveDirection=180;%顶浪航行
WindVelocity=5;%m/s
CurrentVelocity=0.512;%m/s
w_scale=[0.02;0.02;0];%相当于尺度比1:50
w_real_seed=EnvLoadsCal(WaveDirection,WindVelocity,CurrentVelocity)'.*w_scale;
w_real=repmat(w_real_seed,1,Tsim);

%暂时用添加扰动的w作为RBFNN预估的w_estimated，这篇论文里面可以暂时不用神经网络预估（根据误差反馈）
w_estimated=w_real+[0;0;0].*((-1)+(1+1)*rand(3,Tsim));

% %test画图
% plot(1:Tsim,w,1:Tsim,w_estimated);
%% 测量噪声
scale_v_position=0.01;scale_v_angle=0.0;
v=zeros(3,Tsim);
%v=normrnd(0,1,[3,Tsim]);
for i=1:Tsim
    v(1:2,i)=normrnd(0,1,[2,1]);%均值=0，方差=第二个数^2
    v(3,i)=normrnd(0,1,[1,1]);
end
v(1:2,:)=v(1:2,:)*scale_v_position;v(3,:)=v(3,:)*scale_v_angle;

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

%% obstacle
% %static
% pos_obstacle_static=repmat(r(1:2,35),1,Np);
% 
% 
% %dynamic 沿参考轨迹
% temp=r_ori(1:2,108:118);temp(2,:)=temp(2,:)+0.2;
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
%船舶初始状态
% ref_map_3的初始值
x0_leader=[3;4.4;deg2rad(-20);0;0;0];
x0_leftfo=[3;5.4;deg2rad(-20);0;0;0];
x0_rightfo=[3;3.4;deg2rad(-20);0;0;0];

x0e_leader=x0_leader;
x0e_leftfo=x0_leftfo;
x0e_rightfo=x0_rightfo;

y0_leader=C*x0_leader;
y0_leftfo=C*x0_leftfo;
y0_rightfo=C*x0_rightfo;

%推力器推力和角度的初始值 这里的角度是随体坐标系下的
%situation 1
u0_leader=[0;0;0;0;deg2rad(0);deg2rad(0);deg2rad(0);deg2rad(0)];
u0_leftfo=[0;0;0;0;deg2rad(0);deg2rad(0);deg2rad(0);deg2rad(0)];
u0_rightfo=[0;0;0;0;deg2rad(0);deg2rad(0);deg2rad(0);deg2rad(0)];

U0_leader=repmat(u0_leader,Nc,1);
U0_leftfo=repmat(u0_leftfo,Nc,1);
U0_rightfo=repmat(u0_rightfo,Nc,1);

%% 时域模拟
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
    t; %查看迭代时间步
    
    %找到leader前方最近的参考路径点序列
    index=findNearestPoint(y0_leader(1:2),r(1:2,:));
    index_aft = index + Np - 1;
    if index_aft > length(r(1, :))
        Yd_leader = [r(:, index : end), repmat(r(:, end), 1, index_aft - length(r(1, :)) )];
    else
        Yd_leader=r(:,index:index+Np-1);
    end

        

    
    indexCollect=[indexCollect index];
    
    %根据编队控制参数计算follower的期望路径点序列
    Yd_leftfo=desiredOutputCal_follower(Yd_leader,range12,bearing12,1);
    Yd_rightfo=desiredOutputCal_follower(Yd_leader,range13,bearing13,2);

    %MPC controller for leader ship 放在内层循环的外面
    [x_leader,xe_leader,y_leader,U_leaderself,tau_real_leader]=controller_leader(U0_leader,x0_leader,x0e_leader,w_real(:,t),w_estimated(:,t),v(:,t),Yd_leader);
    
    tic;%计算每次迭代时间（船舶之间信息传输并达到一致的时间
    
    iterTime(t)=toc*0.5;%实际工程中可以让两条follower并行计算 因此*0.5
%     iterCollect=[iterCollect iter];

    %更新leader初始值
    %leader
    U0_leader=U_leaderself;
    x0_leader=x_leader;
    x0e_leader=xe_leader;
    y0_leader=y_leader;
 
%     %更新leftfo 没有出现故障
%     U0_leftfo=U_leftfoself;
%     x0_leftfo=x_leftfo;
%     x0e_leftfo=xe_leftfo;

%     %更新leftfo 出现故障
%     if t>=115 && t<=125
%         U0_leftfo=U_leftfobyrightfo;
%     else
%         U0_leftfo=U_leftfoself;
%     end
%     x0_leftfo=x_leftfo;
%     x0e_leftfo=xe_leftfo;
    
%     %更新rightfo
%     U0_rightfo=U_rightfoself;
%     x0_rightfo=x_rightfo;
%     x0e_rightfo=xe_rightfo;
    
    %collect information
    %leader
    OutputCollect_leader=[OutputCollect_leader y_leader];
    velocityCollect_leader=[velocityCollect_leader x_leader(4:6)];
    uCollect_leader=[uCollect_leader U_leaderself(1:8)];
    tauCollect_leader=[tauCollect_leader tau_real_leader];
%     %follower
%     OutputCollect_leftfo=[OutputCollect_leftfo y_leftfo];
%     velocityCollect_leftfo=[velocityCollect_leftfo x_leftfo(4:6)];
%     uCollect_leftfo=[uCollect_leftfo U_leftfoself(1:8)];
%     tauCollect_leftfo=[tauCollect_leftfo,tau_leftfo];
%     
%     OutputCollect_rightfo=[OutputCollect_rightfo y_rightfo];
%     velocityCollect_rightfo=[velocityCollect_rightfo x_rightfo(4:6)];
%     uCollect_rightfo=[uCollect_rightfo U_rightfoself(1:8)];
%     tauCollect_rightfo=[tauCollect_rightfo,tau_rightfo];
    
    %实时查看输出
    leader=[y_leader(1) y_leader(2) rad2deg(y_leader(3))]
%     leftfo=[y_leftfo(1) y_leftfo(2) rad2deg(y_leftfo(3))]
%     rightfo=[y_rightfo(1) y_rightfo(2) rad2deg(y_rightfo(3))]

%     dist_leader_leftfo=sqrt((y_leader(1)-y_leftfo(1))^2+(y_leader(2)-y_leftfo(2))^2)
%     dist_leader_rightfo=sqrt((y_leader(1)-y_rightfo(1))^2+(y_leader(2)-y_rightfo(2))^2)
%     dist_leftfo_rightfo=sqrt((y_leftfo(1)-y_rightfo(1))^2+(y_leftfo(2)-y_rightfo(2))^2)

end
figure(1)
plot(OutputCollect_leader(1, :), OutputCollect_leader(2, :))
hold on 
plot(r(1, :), r(2, :))
legend('y', 'yd')

%leader跟踪误差时历  
figure(3)
objPath=zeros(3,Tsim);
% indexCollect(1)=1;%把第一个值换成第一个路径点
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




