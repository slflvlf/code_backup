function [J]=objfunc_fo_substep(U_self,U0_self,U_neighbor1,U_neighbor2,U_e,x0,x0_e,X_e,S,T,I,w_estimated,r)
% This function is used to create the objective function of the follower at
% sub steps.

% Input
% U_self         ������Լ��Ŀ�������
% U0_self        ��һ�����Լ��Ŀ�������
% U_neighbor1    ��һ����õ�neighbor1�Ŀ�������
% U_neighbor2    ��һ����õ�neighbor1�Ŀ�������
% U_e            ��������seed��������
% x0             ��һ������������״̬����
% x0_e           ��һ������������seed״̬����
% X_e            ��������seed״̬���� ����һ����δ��Np��
% S              ��������Ԥ�����
% T
% I
% w_estimated    ��������Ԥ���Ļ�����
% r              ������������λ�ú������

% Output
% J              ����ָ��

global Np
global Q_fm
global terCoe
global Q_obs_avoidance
global pos_obstacle_static
global pos_obstacle_dynamic
global gamma
global obstacleExist

%% ׼������
global Q_in_fo_substep
global Q_din_fo_substep
global Q_fm_fo_substep

%Q_fm_fo_substep_addi
%��������ʵ��λ��
U=[U_self;U_neighbor1;U_neighbor2];%U_self�Ǵ�����
Y=S*(U-U_e)+T*(x0-x0_e)+I*X_e;
Y_neighbor1_=Y(3*Np+1:6*Np);Y_neighbor1=reshape(Y_neighbor1_,[3,Np]);%ת����������λ�þ�����ͬ����ʽ 3xNp
Y_neighbor2_=Y(6*Np+1:end);Y_neighbor2=reshape(Y_neighbor2_,[3,Np]);
%������������λ�� 3xNp
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

%��������
x0_e_self=x0_e(1:6);
[U_e_self,X_e_self,S_self,T_self,I_self,~,~]=vesselPredModel_MatPre(U0_self,x0_e_self,w_estimated(1:3),1);%flag==1 selfԤ�����

%% Ŀ�꺯��
dU=abs(U_self-U0_self);%�������еı仯��
x0_self=x0(1:6);
Y=S_self*(U_self-U_e_self)+T_self*(x0_self-x0_e_self)+I_self*X_e_self;%Ŀ�괬������
if obstacleExist==1
    %ȥ������ǣ�ֻ�����������꣬���ڼ������Ŀ�꺯��
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