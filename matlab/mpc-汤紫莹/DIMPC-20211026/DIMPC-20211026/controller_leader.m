function [x,x_e,y,U,tau_real]=controller_leader(U0,x0,x0_e,w_real,w_estimated,v,r)
% This function is used to create the local controller of leader.
% SQP algorithm is used.

% Input
% U0              上一步的控制序列 8*Ncx1
% x0              上一步的状态变量 6x1
% x0_e            上一步的seed state 6x1
% w_real          当前时刻真实的环境力 3x1
% w_estimated     RBFNN预估的环境力 3x1
% v               测量噪声
% r               未来Np步的参考轨迹 3xNp

% Output
% x               下一步的状态变量    [x,y,psi,u,v,r]
% x_e             下一步的seed state  [x,y,psi,u,v,r]
% y               下一步的输出        [x,y,psi]
% U               求得的最优控制序列
global C
global G
global N

%% 预测矩阵
[U_e,X_e,~,~,~,AJ_cal,BJ_cal]=vesselPredModel_MatPre(U0,x0_e,w_estimated,1);%flag==1 单船

%% 求解
%不等式约束和上下限
[A_ineq,b_ineq,lb,ub]=phylimofthr(U0,1);%flag==1 单船

% %QP求解
% options = optimset('Algorithm','interior-point-convex','Display','off');
% U=quadprog(H,f,A_ineq,b_ineq,[],[],lb,ub,U0,options);

%SQP
options=optimoptions('fmincon','Algorithm','SQP','MaxFunctionEvaluations', 10000,'ConstraintTolerance',1e-3,'StepTolerance',1e-3);
%U=fmincon(@(U)objfunc_leader(U,U0,x0,x0_e,w_estimated,r),U0,A_ineq,b_ineq,[],[],lb,ub,@(U)noncon_leader(U,U0),options);
U=fmincon(@(U)objfunc_leader(U,U0,x0,x0_e,w_estimated,r),U0,A_ineq,b_ineq,[],[],lb,ub,[],options);


%输出
u0=U(1:2*N);%得到各推力器的推力和角度
u0_e=U_e(1:2*N);
x_e=X_e(1:6);

%求实际的控制合力（矩） tau_real
tau_real=thrust_configuration(u0)*u0(1:4);

% %求总的控制合力（矩） tau_tol
% [R,~]=create_RotMat_CvMat(x0_e);
% tau_tol=tau_real+R'*w;

%x=x_e+AJ_cal*(x0-x0_e)+BJ_cal*(u0-u0_e);%实际位置
x=x_e+AJ_cal*(x0-x0_e)+BJ_cal*(u0-u0_e)+G*(w_real-w_estimated);%当前时刻真实的环境力应该加在这，G的Jacobian矩阵是它本身
%y=C*x+v;%测量位置，考虑了测量噪声；不用seed state，差别不大
y=C*x;%测量位置，不考虑测量噪声

end