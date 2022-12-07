function [x_leader,xe_leader,y_leader,U_leader,tau_leader,...
    x_leftfo,xe_leftfo,y_leftfo,U_leftfo,tau_leftfo,...
    x_rightfo,xe_rightfo,y_rightfo,U_rightfo,tau_rightfo]=global_controller(U0,x0,x0_e,w_estimated,v,r)
% This function is used to create the global controller for the
% multi-vessel system.

% Input
% U0    上一步的控制序列       多船 3*8*Ncx1 顺序leader-leftfo-rightfo
% x0    上一步的状态变量       多船 3*6x1
% x0_e  上一步的seed state     多船 3*6x1
% w     RBFNN预估的环境力      多船 3*3x1
% v     测量噪声               多船 3x3x1
% r     未来Np步的期望位置（对于leader是参考轨迹 对于follower是根据编队信息计算得到的期望位置和艏向角）   多船 3x3xNp

% Output 
% x_leader       下一步的状态变量 [x,y,psi,u,v,r]  6x1
% xe_leader      下一步的seed状态变量              6x1
% y_leader       下一步的输出     [x,y,psi]        3x1
% U_leader       求得的leader的最优控制序列    8*Ncx1 

% x_leftfo
% xe_leftfo
% y_leftfo
% U_leftfo       求得的leftfo的最优控制序列    8*Ncx1 

% x_rightfo
% xe_rightfo
% y_rightfo
% U_rightfo      求得的rightfo的最优控制序列   8*Ncx1 

global Nc
global N
global C
%% 推力器物理限制
[A_ineq,b_ineq,lb,ub]=phylimofthr(U0,2);%flag==2 三条船

%% 预测矩阵
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

%合并生成三条船的预测矩阵
U_e=[U_e_leader;U_e_leftfo;U_e_rightfo];
X_e=[X_e_leader;X_e_leftfo;X_e_rightfo];
zerosS=zeros(size(S_leader));zerosT=zeros(size(T_leader));zerosI=zeros(size(I_leader));
S=[S_leader zerosS zerosS;zerosS S_leftfo zerosS;zerosS zerosS S_rightfo];
T=[T_leader zerosT zerosT;zerosT T_leftfo zerosT;zerosT zerosT T_rightfo];
I=[I_leader zerosI zerosI;zerosI I_leftfo zerosI;zerosI zerosI I_rightfo];

%% 求解
options=optimoptions('fmincon','Algorithm','SQP','MaxFunctionEvaluations', 100000,'ConstraintTolerance',1e-3,'StepTolerance',1e-3);
U=fmincon(@(U)objfunc(U,U0,x0,x0_e,w_estimated,r),U0,A_ineq,b_ineq,[],[],lb,ub,@(U)safedistConstraint(U,U_e,x0,x0_e,X_e,S,T,I),options);

%输出
U_leader=U(1:2*N*Nc);
U_leftfo=U(2*N*Nc+1:2*2*N*Nc);
U_rightfo=U(2*2*N*Nc+1:end);

x_leader=xe_leader+AJ_cal_leader*(x0_leader-x0_e_leader)+BJ_cal_leader*(U_leader(1:2*N)-U_e_leader(1:2*N));
y_leader=C*x_leader;%leader暂时不考虑测量噪声，因为没开启观测器

x_leftfo=xe_leftfo+AJ_cal_leftfo*(x0_leftfo-x0_e_leftfo)+BJ_cal_leftfo*(U_leftfo(1:2*N)-U_e_leftfo(1:2*N));
y_leftfo=C*x_leftfo;%fo暂时不考虑测量噪声

x_rightfo=xe_rightfo+AJ_cal_rightfo*(x0_rightfo-x0_e_rightfo)+BJ_cal_rightfo*(U_rightfo(1:2*N)-U_e_rightfo(1:2*N));
y_rightfo=C*x_rightfo;%fo暂时不考虑测量噪声

tau_leader=thrust_configuration(U_leader(1:2*N))*U_leader(1:4);
tau_leftfo=thrust_configuration(U_leftfo(1:2*N))*U_leftfo(1:4);
tau_rightfo=thrust_configuration(U_rightfo(1:2*N))*U_rightfo(1:4);

end