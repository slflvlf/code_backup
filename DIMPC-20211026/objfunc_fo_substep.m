function [J]=objfunc_fo_substep(U_self,U0_self,U_neighbor1,U_neighbor2,U_e,x0,x0_e,X_e,S,T,I,w_estimated,r)
% This function is used to create the objective function of the follower at
% sub steps.

% Input
% U_self         待求的自己的控制序列
% U0_self        上一步的自己的控制序列
% U_neighbor1    上一步求得的neighbor1的控制序列
% U_neighbor2    上一步求得的neighbor1的控制序列
% U_e            三条船的seed控制序列
% x0             上一步的三条船的状态变量
% x0_e           上一步的三条船的seed状态变量
% X_e            三条船的seed状态变量 从这一步到未来Np步
% S              三条船的预测矩阵
% T
% I
% w_estimated    三条船的预估的环境力
% r              三条船的期望位置和艏向角

% Output
% J              性能指标

global Np
global Q_fm
global terCoe
global Q_obs_avoidance
global pos_obstacle_static
global pos_obstacle_dynamic
global gamma
global obstacleExist

%% 准备矩阵
global Q_in_fo_substep
global Q_din_fo_substep
global Q_fm_fo_substep

%Q_fm_fo_substep_addi
%三条船的实际位置
U=[U_self;U_neighbor1;U_neighbor2];%U_self是待求量
Y=S*(U-U_e)+T*(x0-x0_e)+I*X_e;
Y_neighbor1_=Y(3*Np+1:6*Np);Y_neighbor1=reshape(Y_neighbor1_,[3,Np]);%转换成与期望位置矩阵相同的形式 3xNp
Y_neighbor2_=Y(6*Np+1:end);Y_neighbor2=reshape(Y_neighbor2_,[3,Np]);
%三条船的期望位置 3xNp
Yd_self=r(1:3,:);
Yd_neighbor1=r(4:6,:);
Yd_neighbor2=r(7:9,:);

Q_fm_fo_substep_addi_cell=cell(Np,1);
for i=1:Np
    if i==Np
        Q_fm_fo_substep_addi_cell{i,1}=-2*terCoe*Q_fm*((Y_neighbor1(:,i)+Yd_self(:,i)-Yd_neighbor1(:,i))+(Y_neighbor2(:,i)+Yd_self(:,i)-Yd_neighbor2(:,i)));
    else
        Q_fm_fo_substep_addi_cell{i,1}=-2*Q_fm*((Y_neighbor1(:,i)+Yd_self(:,i)-Yd_neighbor1(:,i))+(Y_neighbor2(:,i)+Yd_self(:,i)-Yd_neighbor2(:,i)));
    end
end
Q_fm_fo_substep_addi=cell2mat(Q_fm_fo_substep_addi_cell);

%其他矩阵
x0_e_self=x0_e(1:6);
[U_e_self,X_e_self,S_self,T_self,I_self,~,~]=vesselPredModel_MatPre(U0_self,x0_e_self,w_estimated(1:3),1);%flag==1 self预测矩阵

%% 目标函数
dU=abs(U_self-U0_self);%控制序列的变化量
x0_self=x0(1:6);
Y=S_self*(U_self-U_e_self)+T_self*(x0_self-x0_e_self)+I_self*X_e_self;%目标船的坐标
if obstacleExist==1
    %去掉艏向角，只保留横纵坐标，用于计算避障目标函数
    Y_xy=zeros(2*Np,1);
    for i=1:Np
        Y_xy(2*i-1)=Y(3*i-2);
        Y_xy(2*i)=Y(3*i-1);
    end
    J_obs_static=0;J_obs_dynamic=0;
    for i=1:Np
        J_obs_static=J_obs_static+Q_obs_avoidance/((Y_xy(2*i-1)-pos_obstacle_static(2*i-1))^2+(Y_xy(2*i)-pos_obstacle_static(2*i))^2+gamma);
        J_obs_dynamic=J_obs_dynamic+Q_obs_avoidance/((Y_xy(2*i-1)-pos_obstacle_dynamic(2*i-1))^2+(Y_xy(2*i)-pos_obstacle_dynamic(2*i))^2+gamma);
    end
    J_obs_avoidance=J_obs_static+J_obs_dynamic;

    J=U_self'*Q_in_fo_substep*U_self+dU'*Q_din_fo_substep*dU+Y'*Q_fm_fo_substep*Y+Q_fm_fo_substep_addi'*Y+J_obs_avoidance;
elseif obstacleExist==0
    J=U_self'*Q_in_fo_substep*U_self+dU'*Q_din_fo_substep*dU+Y'*Q_fm_fo_substep*Y+Q_fm_fo_substep_addi'*Y;
end
end