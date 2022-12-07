function [x_self,xe_self,y_self,U_self,U_neighbor1,U_neighbor2,tau_self]=controller_fo_firstep(U0,x0,x0_e,w_real,w_estimated,v,r)
% This function is used to create the local controller of the follower at the first step.
% fmincon solver is used.


% Input
% U0    上一步的控制序列       多船 3*8*Ncx1 
% x0    上一步的状态变量       多船 3*6x1
% x0_e  上一步的seed state     多船 3*6x1
% w_real          当前时刻真实的环境力    多船 3x3x1
% w_estimated     RBFNN预估的环境力      多船 3*3x1
% v     测量噪声               多船 3x3x1
% r     未来Np步的期望位置（对于leader是参考轨迹 对于follower是根据编队信息计算得到的期望位置和艏向角）   多船 9xNp

% Output 
% x_self         下一步的状态变量 [x,y,psi,u,v,r]  6x1
% xe_self        下一步的seed状态变量              6x1
% y_self         下一步的输出     [x,y,psi]        3x1
% U_self         求得的自己的最优控制序列                          8*Ncx1 
% U_neighbor1    求得的neighbor1（leader）的最优控制序列           8*Ncx1 
% U_neighbor2    求得的neighbor2（另一个follower）的最优控制序列   8*Ncx1 

global N
global Nc
global C
global G
%% 推力器物理限制
[A_ineq,b_ineq,lb,ub]=phylimofthr(U0,2);%flag==2 三条船

%% 预测矩阵
[U_e,X_e,S,T,I,AJ_cal,BJ_cal]=vesselPredModel_MatPre(U0,x0_e,w_estimated,2);%这里算出来的AJ_cal BJ_cal是三条船的 需要拆分
AJ_cal=AJ_cal(:,1:6);%AJ 6x6
BJ_cal=BJ_cal(:,1:8);%BJ 6x8
xe_self=X_e(1:6);
%% 求解
options=optimoptions('fmincon','Algorithm','SQP','MaxFunctionEvaluations', 100000,'ConstraintTolerance',1e-2,'StepTolerance',1e-2);
U=fmincon(@(U)objfunc_fo_firstep(U,U0,x0,x0_e,w_estimated,r),U0,A_ineq,b_ineq,[],[],lb,ub,@(U)safedistConstraint_firstep(U,U_e,x0,x0_e,X_e,S,T,I),options);
%输出
U_self=U(1:2*N*Nc);
%Ue_self=U_e(1:2*N*Nc);
U_neighbor1=U(2*N*Nc+1:2*2*N*Nc);
U_neighbor2=U(2*2*N*Nc+1:end);

%求实际的控制合力（矩） tau_self
tau_self=thrust_configuration(U_self(1:2*N))*U_self(1:4);

x_self=X_e(1:6)+AJ_cal*(x0(1:6)-x0_e(1:6))+BJ_cal*(U_self(1:2*N)-U_e(1:2*N))+G*(w_real(1:3)-w_estimated(1:3));%真实的环境力加在这
%y_self=C*x_self+v(1:3);%考虑了测量噪声
y_self=C*x_self;%不考虑测量噪声 因为follower没开启观测器

end


