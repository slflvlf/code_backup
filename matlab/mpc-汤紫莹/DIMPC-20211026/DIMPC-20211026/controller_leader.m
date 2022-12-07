function [x,x_e,y,U,tau_real]=controller_leader(U0,x0,x0_e,w_real,w_estimated,v,r)
% This function is used to create the local controller of leader.
% SQP algorithm is used.

% Input
% U0              ��һ���Ŀ������� 8*Ncx1
% x0              ��һ����״̬���� 6x1
% x0_e            ��һ����seed state 6x1
% w_real          ��ǰʱ����ʵ�Ļ����� 3x1
% w_estimated     RBFNNԤ���Ļ����� 3x1
% v               ��������
% r               δ��Np���Ĳο��켣 3xNp

% Output
% x               ��һ����״̬����    [x,y,psi,u,v,r]
% x_e             ��һ����seed state  [x,y,psi,u,v,r]
% y               ��һ�������        [x,y,psi]
% U               ��õ����ſ�������
global C
global G
global N

%% Ԥ�����
[U_e,X_e,~,~,~,AJ_cal,BJ_cal]=vesselPredModel_MatPre(U0,x0_e,w_estimated,1);%flag==1 ����

%% ���
%����ʽԼ����������
[A_ineq,b_ineq,lb,ub]=phylimofthr(U0,1);%flag==1 ����

% %QP���
% options = optimset('Algorithm','interior-point-convex','Display','off');
% U=quadprog(H,f,A_ineq,b_ineq,[],[],lb,ub,U0,options);

%SQP
options=optimoptions('fmincon','Algorithm','SQP','MaxFunctionEvaluations', 10000,'ConstraintTolerance',1e-3,'StepTolerance',1e-3);
%U=fmincon(@(U)objfunc_leader(U,U0,x0,x0_e,w_estimated,r),U0,A_ineq,b_ineq,[],[],lb,ub,@(U)noncon_leader(U,U0),options);
U=fmincon(@(U)objfunc_leader(U,U0,x0,x0_e,w_estimated,r),U0,A_ineq,b_ineq,[],[],lb,ub,[],options);


%���
u0=U(1:2*N);%�õ����������������ͽǶ�
u0_e=U_e(1:2*N);
x_e=X_e(1:6);

%��ʵ�ʵĿ��ƺ������أ� tau_real
tau_real=thrust_configuration(u0)*u0(1:4);

% %���ܵĿ��ƺ������أ� tau_tol
% [R,~]=create_RotMat_CvMat(x0_e);
% tau_tol=tau_real+R'*w;

%x=x_e+AJ_cal*(x0-x0_e)+BJ_cal*(u0-u0_e);%ʵ��λ��
x=x_e+AJ_cal*(x0-x0_e)+BJ_cal*(u0-u0_e)+G*(w_real-w_estimated);%��ǰʱ����ʵ�Ļ�����Ӧ�ü����⣬G��Jacobian������������
%y=C*x+v;%����λ�ã������˲�������������seed state����𲻴�
y=C*x;%����λ�ã������ǲ�������

end