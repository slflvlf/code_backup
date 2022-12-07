function [x_self,xe_self,y_self,U_self,U_neighbor1,U_neighbor2,tau_self]=controller_fo_firstep(U0,x0,x0_e,w_real,w_estimated,v,r)
% This function is used to create the local controller of the follower at the first step.
% fmincon solver is used.


% Input
% U0    ��һ���Ŀ�������       �ബ 3*8*Ncx1 
% x0    ��һ����״̬����       �ബ 3*6x1
% x0_e  ��һ����seed state     �ബ 3*6x1
% w_real          ��ǰʱ����ʵ�Ļ�����    �ബ 3x3x1
% w_estimated     RBFNNԤ���Ļ�����      �ബ 3*3x1
% v     ��������               �ബ 3x3x1
% r     δ��Np��������λ�ã�����leader�ǲο��켣 ����follower�Ǹ��ݱ����Ϣ����õ�������λ�ú�����ǣ�   �ബ 9xNp

% Output 
% x_self         ��һ����״̬���� [x,y,psi,u,v,r]  6x1
% xe_self        ��һ����seed״̬����              6x1
% y_self         ��һ�������     [x,y,psi]        3x1
% U_self         ��õ��Լ������ſ�������                          8*Ncx1 
% U_neighbor1    ��õ�neighbor1��leader�������ſ�������           8*Ncx1 
% U_neighbor2    ��õ�neighbor2����һ��follower�������ſ�������   8*Ncx1 

global N
global Nc
global C
global G
%% ��������������
[A_ineq,b_ineq,lb,ub]=phylimofthr(U0,2);%flag==2 ������

%% Ԥ�����
[U_e,X_e,S,T,I,AJ_cal,BJ_cal]=vesselPredModel_MatPre(U0,x0_e,w_estimated,2);%�����������AJ_cal BJ_cal���������� ��Ҫ���
AJ_cal=AJ_cal(:,1:6);%AJ 6x6
BJ_cal=BJ_cal(:,1:8);%BJ 6x8
xe_self=X_e(1:6);
%% ���
options=optimoptions('fmincon','Algorithm','SQP','MaxFunctionEvaluations', 100000,'ConstraintTolerance',1e-2,'StepTolerance',1e-2);
U=fmincon(@(U)objfunc_fo_firstep(U,U0,x0,x0_e,w_estimated,r),U0,A_ineq,b_ineq,[],[],lb,ub,@(U)safedistConstraint_firstep(U,U_e,x0,x0_e,X_e,S,T,I),options);
%���
U_self=U(1:2*N*Nc);
%Ue_self=U_e(1:2*N*Nc);
U_neighbor1=U(2*N*Nc+1:2*2*N*Nc);
U_neighbor2=U(2*2*N*Nc+1:end);

%��ʵ�ʵĿ��ƺ������أ� tau_self
tau_self=thrust_configuration(U_self(1:2*N))*U_self(1:4);

x_self=X_e(1:6)+AJ_cal*(x0(1:6)-x0_e(1:6))+BJ_cal*(U_self(1:2*N)-U_e(1:2*N))+G*(w_real(1:3)-w_estimated(1:3));%��ʵ�Ļ�����������
%y_self=C*x_self+v(1:3);%�����˲�������
y_self=C*x_self;%�����ǲ������� ��Ϊfollowerû�����۲���

end


