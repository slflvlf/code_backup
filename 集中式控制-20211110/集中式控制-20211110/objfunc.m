function J=objfunc(U,U0,x0,x0_e,w_estimated,r)
%This function is used to create the objective function of the global
%controller.

% Input
% U               待求的自变量 最优的控制序列  3x8xNc 顺序是leader-leftfo-rightfo
% U0              上一步的控制序列
% x0              上一步的状态变量
% x0_e            上一步的seed state
% w_estimated     RBFNN预估的环境力 3x3x1
% r               未来Np步的参考轨迹 3x3xNp

% Output
% J               性能指标

global Np
global Q_err
global Q_fm
global terCoe
global Q_obs_avoidance
global pos_obstacle_static
global pos_obstacle_dynamic
global gamma
global obstacleExist

%% 准备矩阵
global Q_IN
global Q_dIN
global Q_Err

%Q_Err_addi 第四个矩阵
Yd_leader=r(1:3,:);%三条船的期望位置
Yd_leftfo=r(4:6,:);
Yd_rightfo=r(7:9,:);

Q_Err_addi_cell_1=cell(Np,1);
Q_Err_addi_cell_2=cell(Np,1);
Q_Err_addi_cell_3=cell(Np,1);
for i=1:Np
    if i==Np
        Q_Err_addi_cell_1{i,1}=-2*terCoe*Q_err*Yd_leader(:,i);
        Q_Err_addi_cell_2{i,1}=-2*terCoe*Q_fm*Yd_leftfo(:,i);
        Q_Err_addi_cell_3{i,1}=-2*terCoe*Q_fm*Yd_rightfo(:,i);
    else
        Q_Err_addi_cell_1{i,1}=-2*Q_err*Yd_leader(:,i);
        Q_Err_addi_cell_2{i,1}=-2*Q_fm*Yd_leftfo(:,i);
        Q_Err_addi_cell_3{i,1}=-2*Q_fm*Yd_rightfo(:,i);
    end
end
Q_Err_addi_1=cell2mat(Q_Err_addi_cell_1);
Q_Err_addi_2=cell2mat(Q_Err_addi_cell_2);
Q_Err_addi_3=cell2mat(Q_Err_addi_cell_3);
Q_Err_addi=[Q_Err_addi_1;Q_Err_addi_2;Q_Err_addi_3];

%其他矩阵
[U_e,X_e,S,T,I,~,~]=vesselPredModel_MatPre(U0,x0_e,w_estimated,2);%flag==2 三条船
%% 目标函数
dU=abs(U-U0);%控制序列的变化量
Y=S*(U-U_e)+T*(x0-x0_e)+I*X_e;
if obstacleExist==1
    Y_leader=Y(1:3*Np);Y_leftfo=Y(3*Np+1:6*Np);Y_rightfo=Y(6*Np+1:end);
    Y_leader_xy=zeros(2*Np,1);Y_leftfo_xy=zeros(2*Np,1);Y_rightfo_xy=zeros(2*Np,1);
    for i=1:Np
        Y_leader_xy(2*i-1)=Y_leader(3*i-2);
        Y_leader_xy(2*i)=Y_leader(3*i-1);
        Y_leftfo_xy(2*i-1)=Y_leftfo(3*i-2);
        Y_leftfo_xy(2*i)=Y_leftfo(3*i-1);
        Y_rightfo_xy(2*i-1)=Y_rightfo(3*i-2);
        Y_rightfo_xy(2*i)=Y_rightfo(3*i-1);
    end
    J1_obs_static=0;J1_obs_dynamic=0;
    J2_obs_static=0;J2_obs_dynamic=0;
    J3_obs_static=0;J3_obs_dynamic=0;
    for i=1:Np
        if i==Np %终端系数
        J1_obs_static=J1_obs_static+terCoe*Q_obs_avoidance/((Y_leader_xy(2*i-1)-pos_obstacle_static(2*i-1))^2+(Y_leader_xy(2*i)-pos_obstacle_static(2*i))^2+gamma);
        J1_obs_dynamic=J1_obs_dynamic+terCoe*Q_obs_avoidance/((Y_leader_xy(2*i-1)-pos_obstacle_dynamic(2*i-1))^2+(Y_leader_xy(2*i)-pos_obstacle_dynamic(2*i))^2+gamma);
        J2_obs_static=J2_obs_static+terCoe*Q_obs_avoidance/((Y_leftfo_xy(2*i-1)-pos_obstacle_static(2*i-1))^2+(Y_leftfo_xy(2*i)-pos_obstacle_static(2*i))^2+gamma);
        J2_obs_dynamic=J2_obs_dynamic+terCoe*Q_obs_avoidance/((Y_leftfo_xy(2*i-1)-pos_obstacle_dynamic(2*i-1))^2+(Y_leftfo_xy(2*i)-pos_obstacle_dynamic(2*i))^2+gamma);
        J3_obs_static=J3_obs_static+terCoe*Q_obs_avoidance/((Y_rightfo_xy(2*i-1)-pos_obstacle_static(2*i-1))^2+(Y_rightfo_xy(2*i)-pos_obstacle_static(2*i))^2+gamma);
        J3_obs_dynamic=J3_obs_dynamic+terCoe*Q_obs_avoidance/((Y_rightfo_xy(2*i-1)-pos_obstacle_dynamic(2*i-1))^2+(Y_rightfo_xy(2*i)-pos_obstacle_dynamic(2*i))^2+gamma);
        else
        J1_obs_static=J1_obs_static+Q_obs_avoidance/((Y_leader_xy(2*i-1)-pos_obstacle_static(2*i-1))^2+(Y_leader_xy(2*i)-pos_obstacle_static(2*i))^2+gamma);
        J1_obs_dynamic=J1_obs_dynamic+Q_obs_avoidance/((Y_leader_xy(2*i-1)-pos_obstacle_dynamic(2*i-1))^2+(Y_leader_xy(2*i)-pos_obstacle_dynamic(2*i))^2+gamma);
        J2_obs_static=J2_obs_static+Q_obs_avoidance/((Y_leftfo_xy(2*i-1)-pos_obstacle_static(2*i-1))^2+(Y_leftfo_xy(2*i)-pos_obstacle_static(2*i))^2+gamma);
        J2_obs_dynamic=J2_obs_dynamic+Q_obs_avoidance/((Y_leftfo_xy(2*i-1)-pos_obstacle_dynamic(2*i-1))^2+(Y_leftfo_xy(2*i)-pos_obstacle_dynamic(2*i))^2+gamma);
        J3_obs_static=J3_obs_static+Q_obs_avoidance/((Y_rightfo_xy(2*i-1)-pos_obstacle_static(2*i-1))^2+(Y_rightfo_xy(2*i)-pos_obstacle_static(2*i))^2+gamma);
        J3_obs_dynamic=J3_obs_dynamic+Q_obs_avoidance/((Y_rightfo_xy(2*i-1)-pos_obstacle_dynamic(2*i-1))^2+(Y_rightfo_xy(2*i)-pos_obstacle_dynamic(2*i))^2+gamma);
    
        end
    end
    J_obs_avoidance=J1_obs_static+J1_obs_dynamic+J2_obs_static+J2_obs_dynamic+J3_obs_static+J3_obs_dynamic;
    J=U'*Q_IN*U+dU'*Q_dIN*dU+Y'*Q_Err*Y+Q_Err_addi'*Y+J_obs_avoidance;
elseif obstacleExist==0
    J=U'*Q_IN*U+dU'*Q_dIN*dU+Y'*Q_Err*Y+Q_Err_addi'*Y;
end
end