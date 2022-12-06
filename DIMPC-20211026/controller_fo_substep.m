function [x_self,xe_self,y_self,U_self,tau_self]=controller_fo_substep(U0,U_neighbor1,U_neighbor2,x0,x0_e,w_real,w_estimated,v,r)
% This function is used to create the local controller of the follower at sub steps.
% fmincon solver is used.

% Input
% U0=[U0_self,U0_neighbor1,U0_neighbor2]         上一步的控制序列 三条船
% U_neighbor1
% U_neighbor2
% x0=[x0_self;x0_neighbor1;x0_neighbor2]         上一步的状态变量       三条船 6x1
% x0_e=[x0e_self;x0e_neighbor1;x0e_neighbor2]    上一步的seed state     三条船 3x6x1 
% w_real          当前时刻真实的环境力    多船 3x3x1
% w_estimated     RBFNN预估的环境力      多船 3*3x1
% v          测量噪声                             三条船 3x3x1
% r=[Yd_self,Yd_neighbor1,Yd_neighbor2]           三条船

% Output 
% x_self         下一步的状态变量 [x,y,psi,u,v,r]  6x1
% xe_self        下一步的seed状态变量              6x1
% y_self         下一步的输出     [x,y,psi]        3x1
% U_self         求得的自己的最优控制序列           8*Ncx1 

global N
global Nc
global C
global G

U0_self=U0(1:8*Nc);
x0_self=x0(1:6);
x0_e_self=x0_e(1:6);
%% 推力器物理限制
[A_ineq,b_ineq,lb,ub]=phylimofthr(U0_self,1);%flag==1 self的限制条件

%% 预测矩阵
[U_e,X_e,S,T,I,~,~]=vesselPredModel_MatPre(U0,x0_e,w_estimated,2);%flag==2 还是需要计算三条船的，因为要计算三条船的输出，然后计算安全距离限制条件

%% 求解
options=optimoptions('fmincon','Algorithm','SQP','MaxFunctionEvaluations', 10000,'ConstraintTolerance',1e-2,'StepTolerance',1e-2);
U_self=fmincon(@(U_self)objfunc_fo_substep(U_self,U0_self,U_neighbor1,U_neighbor2,U_e,x0,x0_e,X_e,S,T,I,w_estimated,r),U0_self,A_ineq,b_ineq,[],[],lb,ub,@(U_self)safedistConstraint_substep(U_self,U_neighbor1,U_neighbor2,U_e,x0,x0_e,X_e,S,T,I),options);

%输出
[U_e_self,X_e_self,~,~,~,AJ_self,BJ_self]=vesselPredModel_MatPre(U0_self,x0_e_self,w_estimated(1:3),1);%self的预测矩阵
xe_self=X_e_self(1:6);
x_self=xe_self+AJ_self*(x0_self-x0_e_self)+BJ_self*(U_self(1:2*N)-U_e_self(1:2*N))+G*(w_real(1:3)-w_estimated(1:3));%真实的环境力加在这;
y_self=C*x_self;%不考虑测量噪声 因为follower没开启观测器
%y_self=C*x_self+v(1:3);%考虑了测量噪声

%求实际的控制合力（矩） tau_self
tau_self=thrust_configuration(U_self(1:2*N))*U_self(1:4);

end