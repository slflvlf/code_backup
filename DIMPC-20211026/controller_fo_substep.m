function [x_self,xe_self,y_self,U_self,tau_self]=controller_fo_substep(U0,U_neighbor1,U_neighbor2,x0,x0_e,w_real,w_estimated,v,r)
% This function is used to create the local controller of the follower at sub steps.
% fmincon solver is used.

% Input
% U0=[U0_self,U0_neighbor1,U0_neighbor2]         ��һ���Ŀ������� ������
% U_neighbor1
% U_neighbor2
% x0=[x0_self;x0_neighbor1;x0_neighbor2]         ��һ����״̬����       ������ 6x1
% x0_e=[x0e_self;x0e_neighbor1;x0e_neighbor2]    ��һ����seed state     ������ 3x6x1 
% w_real          ��ǰʱ����ʵ�Ļ�����    �ബ 3x3x1
% w_estimated     RBFNNԤ���Ļ�����      �ബ 3*3x1
% v          ��������                             ������ 3x3x1
% r=[Yd_self,Yd_neighbor1,Yd_neighbor2]           ������

% Output 
% x_self         ��һ����״̬���� [x,y,psi,u,v,r]  6x1
% xe_self        ��һ����seed״̬����              6x1
% y_self         ��һ�������     [x,y,psi]        3x1
% U_self         ��õ��Լ������ſ�������           8*Ncx1 

global N
global Nc
global C
global G

U0_self=U0(1:8*Nc);
x0_self=x0(1:6);
x0_e_self=x0_e(1:6);
%% ��������������
[A_ineq,b_ineq,lb,ub]=phylimofthr(U0_self,1);%flag==1 self����������

%% Ԥ�����
[U_e,X_e,S,T,I,~,~]=vesselPredModel_MatPre(U0,x0_e,w_estimated,2);%flag==2 ������Ҫ�����������ģ���ΪҪ�����������������Ȼ����㰲ȫ������������

%% ���
options=optimoptions('fmincon','Algorithm','SQP','MaxFunctionEvaluations', 10000,'ConstraintTolerance',1e-2,'StepTolerance',1e-2);
U_self=fmincon(@(U_self)objfunc_fo_substep(U_self,U0_self,U_neighbor1,U_neighbor2,U_e,x0,x0_e,X_e,S,T,I,w_estimated,r),U0_self,A_ineq,b_ineq,[],[],lb,ub,@(U_self)safedistConstraint_substep(U_self,U_neighbor1,U_neighbor2,U_e,x0,x0_e,X_e,S,T,I),options);

%���
[U_e_self,X_e_self,~,~,~,AJ_self,BJ_self]=vesselPredModel_MatPre(U0_self,x0_e_self,w_estimated(1:3),1);%self��Ԥ�����
xe_self=X_e_self(1:6);
x_self=xe_self+AJ_self*(x0_self-x0_e_self)+BJ_self*(U_self(1:2*N)-U_e_self(1:2*N))+G*(w_real(1:3)-w_estimated(1:3));%��ʵ�Ļ�����������;
y_self=C*x_self;%�����ǲ������� ��Ϊfollowerû�����۲���
%y_self=C*x_self+v(1:3);%�����˲�������

%��ʵ�ʵĿ��ƺ������أ� tau_self
tau_self=thrust_configuration(U_self(1:2*N))*U_self(1:4);

end