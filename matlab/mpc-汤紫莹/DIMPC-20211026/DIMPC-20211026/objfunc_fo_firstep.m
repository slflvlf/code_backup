function [J]=objfunc_fo_firstep(U,U0,x0,x0_e,w_estimated,r) %�����U�Ǵ�����
% This function is used to create the objective function of the follower at
% the first step.

% Input
% U               ������Ա��� ���ŵĿ�������  3x8xNc
% U0              ��һ���Ŀ�������
% x0              ��һ����״̬����
% x0_e            ��һ����seed state
% w_estimated     RBFNNԤ���Ļ����� 3x3x1
% r               δ��Np���Ĳο��켣 3x3xNp

% Output
% J               ����ָ��

global Np
global Q_fm
global terCoe
global Q_obs_avoidance
global pos_obstacle_static
global pos_obstacle_dynamic
global gamma
global obstacleExist

%% ׼������
%Ȩֵ����
global Q_in_fo_firstep
global Q_din_fo_firstep
global Q_fm_fo_firstep

%Q_fm_fo_firstep_addi
Yd_self=r(1:3,:);%������������λ��
Yd_neighbor1=r(4:6,:);
Yd_neighbor2=r(7:9,:);

Q_fm_fo_firstep_addi_cell=cell(3*Np,1);
for i=1:Np
    if i==Np
        Q_fm_fo_firstep_addi_cell{i,1}=-2*terCoe*Q_fm*((Yd_self(:,i)-Yd_neighbor1(:,i))+(Yd_self(:,i)-Yd_neighbor2(:,i)));
    else
        Q_fm_fo_firstep_addi_cell{i,1}=-2*Q_fm*((Yd_self(:,i)-Yd_neighbor1(:,i))+(Yd_self(:,i)-Yd_neighbor2(:,i)));
    end
end
for i=Np+1:2*Np
    if i==2*Np
        Q_fm_fo_firstep_addi_cell{i,1}=2*terCoe*Q_fm*(Yd_self(:,i-Np)-Yd_neighbor1(:,i-Np));
    else
        Q_fm_fo_firstep_addi_cell{i,1}=2*Q_fm*(Yd_self(:,i-Np)-Yd_neighbor1(:,i-Np));
    end
end
for i=2*Np+1:3*Np
    if i==3*Np
        Q_fm_fo_firstep_addi_cell{i,1}=2*terCoe*Q_fm*(Yd_self(:,i-2*Np)-Yd_neighbor2(:,i-2*Np));
    else
        Q_fm_fo_firstep_addi_cell{i,1}=2*Q_fm*(Yd_self(:,i-2*Np)-Yd_neighbor2(:,i-2*Np));
    end
end
Q_fm_fo_firstep_addi=cell2mat(Q_fm_fo_firstep_addi_cell);

%��������
[U_e,X_e,S,T,I,~,~]=vesselPredModel_MatPre(U0,x0_e,w_estimated,2);%flag==2 ������

%% Ŀ�꺯��
dU=abs(U-U0);%�������еı仯��
Y=S*(U-U_e)+T*(x0-x0_e)+I*X_e;%������������
Y_self=Y(1:3*Np);%Ŀ�괬������
if obstacleExist==1
    %ȥ������ǣ�ֻ�����������꣬���ڼ������Ŀ�꺯��
    Y_xy=zeros(2*Np,1);
    for i=1:Np
        Y_xy(2*i-1)=Y_self(3*i-2);
        Y_xy(2*i)=Y_self(3*i-1);
    end
    J_obs_static=0;J_obs_dynamic=0;
    for i=1:Np
        J_obs_static=J_obs_static+Q_obs_avoidance/((Y_xy(2*i-1)-pos_obstacle_static(2*i-1))^2+(Y_xy(2*i)-pos_obstacle_static(2*i))^2+gamma);
        J_obs_dynamic=J_obs_dynamic+Q_obs_avoidance/((Y_xy(2*i-1)-pos_obstacle_dynamic(2*i-1))^2+(Y_xy(2*i)-pos_obstacle_dynamic(2*i))^2+gamma);
    end
    J_obs_avoidance=J_obs_static+J_obs_dynamic;

    J=U'*Q_in_fo_firstep*U+dU'*Q_din_fo_firstep*dU+Y'*Q_fm_fo_firstep*Y+Q_fm_fo_firstep_addi'*Y+J_obs_avoidance;
elseif obstacleExist==0
    J=U'*Q_in_fo_firstep*U+dU'*Q_din_fo_firstep*dU+Y'*Q_fm_fo_firstep*Y+Q_fm_fo_firstep_addi'*Y;
end
end