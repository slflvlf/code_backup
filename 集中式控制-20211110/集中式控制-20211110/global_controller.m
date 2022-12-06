function [x_leader,xe_leader,y_leader,U_leader,tau_leader,...
    x_leftfo,xe_leftfo,y_leftfo,U_leftfo,tau_leftfo,...
    x_rightfo,xe_rightfo,y_rightfo,U_rightfo,tau_rightfo]=global_controller(U0,x0,x0_e,w_estimated,v,r)
% This function is used to create the global controller for the
% multi-vessel system.

% Input
% U0    ��һ���Ŀ�������       �ബ 3*8*Ncx1 ˳��leader-leftfo-rightfo
% x0    ��һ����״̬����       �ബ 3*6x1
% x0_e  ��һ����seed state     �ബ 3*6x1
% w     RBFNNԤ���Ļ�����      �ബ 3*3x1
% v     ��������               �ബ 3x3x1
% r     δ��Np��������λ�ã�����leader�ǲο��켣 ����follower�Ǹ��ݱ����Ϣ����õ�������λ�ú�����ǣ�   �ബ 3x3xNp

% Output 
% x_leader       ��һ����״̬���� [x,y,psi,u,v,r]  6x1
% xe_leader      ��һ����seed״̬����              6x1
% y_leader       ��һ�������     [x,y,psi]        3x1
% U_leader       ��õ�leader�����ſ�������    8*Ncx1 

% x_leftfo
% xe_leftfo
% y_leftfo
% U_leftfo       ��õ�leftfo�����ſ�������    8*Ncx1 

% x_rightfo
% xe_rightfo
% y_rightfo
% U_rightfo      ��õ�rightfo�����ſ�������   8*Ncx1 

global Nc
global N
global C
%% ��������������
[A_ineq,b_ineq,lb,ub]=phylimofthr(U0,2);%flag==2 ������

%% Ԥ�����
%leader
U0_leader=U0(1:2*N*Nc);
x0_leader=x0(1:6);
x0_e_leader=x0_e(1:6);
w_estimated_leader=w_estimated(1:3);
[U_e_leader,X_e_leader,S_leader,T_leader,I_leader,AJ_cal_leader,BJ_cal_leader]=vesselPredModel_MatPre(U0_leader,x0_e_leader,w_estimated_leader,1);

xe_leader=X_e_leader(1:6);

%leftfo
U0_leftfo=U0(2*N*Nc+1:2*2*N*Nc);
x0_leftfo=x0(7:12);
x0_e_leftfo=x0_e(7:12);
w_estimated_leftfo=w_estimated(4:6);
[U_e_leftfo,X_e_leftfo,S_leftfo,T_leftfo,I_leftfo,AJ_cal_leftfo,BJ_cal_leftfo]=vesselPredModel_MatPre(U0_leftfo,x0_e_leftfo,w_estimated_leftfo,1);

xe_leftfo=X_e_leftfo(1:6);

%rightfo
U0_rightfo=U0(2*2*N*Nc+1:end);
x0_rightfo=x0(13:18);
x0_e_rightfo=x0_e(13:18);
w_estimated_rightfo=w_estimated(7:9);
[U_e_rightfo,X_e_rightfo,S_rightfo,T_rightfo,I_rightfo,AJ_cal_rightfo,BJ_cal_rightfo]=vesselPredModel_MatPre(U0_rightfo,x0_e_rightfo,w_estimated_rightfo,1);

xe_rightfo=X_e_rightfo(1:6);

%�ϲ�������������Ԥ�����
U_e=[U_e_leader;U_e_leftfo;U_e_rightfo];
X_e=[X_e_leader;X_e_leftfo;X_e_rightfo];
zerosS=zeros(size(S_leader));zerosT=zeros(size(T_leader));zerosI=zeros(size(I_leader));
S=[S_leader zerosS zerosS;zerosS S_leftfo zerosS;zerosS zerosS S_rightfo];
T=[T_leader zerosT zerosT;zerosT T_leftfo zerosT;zerosT zerosT T_rightfo];
I=[I_leader zerosI zerosI;zerosI I_leftfo zerosI;zerosI zerosI I_rightfo];

%% ���
options=optimoptions('fmincon','Algorithm','SQP','MaxFunctionEvaluations', 100000,'ConstraintTolerance',1e-3,'StepTolerance',1e-3);
U=fmincon(@(U)objfunc(U,U0,x0,x0_e,w_estimated,r),U0,A_ineq,b_ineq,[],[],lb,ub,@(U)safedistConstraint(U,U_e,x0,x0_e,X_e,S,T,I),options);

%���
U_leader=U(1:2*N*Nc);
U_leftfo=U(2*N*Nc+1:2*2*N*Nc);
U_rightfo=U(2*2*N*Nc+1:end);

x_leader=xe_leader+AJ_cal_leader*(x0_leader-x0_e_leader)+BJ_cal_leader*(U_leader(1:2*N)-U_e_leader(1:2*N));
y_leader=C*x_leader;%leader��ʱ�����ǲ�����������Ϊû�����۲���

x_leftfo=xe_leftfo+AJ_cal_leftfo*(x0_leftfo-x0_e_leftfo)+BJ_cal_leftfo*(U_leftfo(1:2*N)-U_e_leftfo(1:2*N));
y_leftfo=C*x_leftfo;%fo��ʱ�����ǲ�������

x_rightfo=xe_rightfo+AJ_cal_rightfo*(x0_rightfo-x0_e_rightfo)+BJ_cal_rightfo*(U_rightfo(1:2*N)-U_e_rightfo(1:2*N));
y_rightfo=C*x_rightfo;%fo��ʱ�����ǲ�������

tau_leader=thrust_configuration(U_leader(1:2*N))*U_leader(1:4);
tau_leftfo=thrust_configuration(U_leftfo(1:2*N))*U_leftfo(1:4);
tau_rightfo=thrust_configuration(U_rightfo(1:2*N))*U_rightfo(1:4);

end