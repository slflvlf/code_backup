function J=objfunc_leader(U,U0,x0,x0_e,w_estimated,r)
% This function is used to set the objective function of the leader ship.
% Input
% U               ������Ա��� ���ŵĿ�������
% U0              ��һ���Ŀ������� 
% x0              ��һ����״̬����
% x0_e            ��һ����seed state
% w_estimated     RBFNNԤ���Ļ����� 3x1
% r               δ��Np���Ĳο��켣 3xNp

% Output
% J               ����ָ��

global Np
global Q_err
global terCoe

%% ׼������
%Ȩֵ����
global Q_err_leader
global Q_in_leader
global Q_din_leader
global Q_obs_avoidance
global pos_obstacle_static
global pos_obstacle_dynamic
global gamma
global obstacleExist

Q_err_leader_addi_cell=cell(Np,1);
for i=1:Np
    if i==Np
        Q_err_leader_addi_cell{i,1}=-2*terCoe*Q_err*r(:,i);
    else
        Q_err_leader_addi_cell{i,1}=-2*Q_err*r(:,i);
    end
end
Q_err_leader_addi=cell2mat(Q_err_leader_addi_cell);

%��������
[U_e,X_e,S,T,I,~,~]=vesselPredModel_MatPre(U0,x0_e,w_estimated,1);%flag==1 ����

%% Ŀ�꺯��
dU=abs(U-U0);%�������еı仯��
Y=S*(U-U_e)+T*(x0-x0_e)+I*X_e;

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
    J=Y'*Q_err_leader*Y+Q_err_leader_addi'*Y+U'*Q_in_leader*U+dU'*Q_din_leader*dU+J_obs_avoidance;
elseif obstacleExist==0
    J=Y'*Q_err_leader*Y+Q_err_leader_addi'*Y+U'*Q_in_leader*U+dU'*Q_din_leader*dU;
end
end



