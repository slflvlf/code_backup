function [x_e_next,AJ,BJ]=RungeKutta(x_e,U_e,w_estimated)
% This function is used to calculate the x_e at time k+1 and Jacobian matrices AJ(k) and BJ(k).
% RungeKutta只计算norminal values!!!

% Input
% x_e             k时刻的seed状态变量          [x y psi u v r]'
% U_e             k时刻的seed控制输入序列      Nc x [f1 f2 f3 f4 alpha1 alpha2 alpha3 alpha4]'
% w_estimated     k时刻的RBFNN估计的环境力     [wx wy wpsi]'  3x1
% 
%
% Output
% x_e_next        k+1时刻的seed状态变量   [x y psi u v r]
% AJ              k时刻的Jacobian矩阵 6x6 对状态变量求偏导
% BJ              k时刻的Jacobian矩阵 6x8 对控制输入求偏导

global M;global D;
global G;
global N;  %推进器的数量
global thrConf;
global Ts;

%% RungeKutta计算下一时刻的seed state x_e_next
% Rotation matrix and coriolis matrix
[R,Cv]=create_RotMat_CvMat(x_e);            %时变

% 非线性模型的系数矩阵
u_e=U_e(1:2*N);%k时刻的控制输入
u_e_next=U_e(2*N+1:2*2*N);%k+1时刻的控制输入

% 推力配置矩阵
T_conf=thrust_configuration(u_e);
T_conf_next=thrust_configuration(u_e_next);

% % 推力损失矩阵
% T_efficiency=thrust_efficiency(u_e);
% T_efficiency_next=thrust_efficiency(u_e_next);
% 默认推力损失系数=1
T_efficiency=[eye(4) zeros(4)];
T_efficiency_next=[eye(4) zeros(4)];

A=[zeros(3) R;zeros(3) -inv(M)*(Cv+D)];     %时变
B=[zeros(3);inv(M)]*T_conf*T_efficiency;               %时变 B(k)=B*T_conf*T_efficiency，其中B=[0;inv(M)]
B_next=[zeros(3);inv(M)]*T_conf_next*T_efficiency_next;

% 计算RungeKutta系数
k1=A*x_e+B*u_e+G*w_estimated;
k2=A*(x_e+0.5*Ts*k1)+B*u_e+G*w_estimated;
k3=A*(x_e+0.5*Ts*k2)+B*u_e+G*w_estimated;
k4=A*(x_e+Ts*k3)+B_next*u_e_next+G*w_estimated;

% 计算下一时刻的seed状态变量
x_e_next=x_e+Ts*(k1+2*k2+2*k3+k4)*0.1667;

%% Jacobian matrices 用于MPC预测模型
%连续的 下标c=continuous
% AJ_c=[0 0 -x_e(4)*sin(x_e(3))-x_e(5)*cos(x_e(3))  cos(x_e(3))  -sin(x_e(3)) 0;...
%     0 0  x_e(4)*cos(x_e(3))-x_e(5)*sin(x_e(3))  sin(x_e(3))  cos(x_e(3))  0;...
%     0 0 0 0 0 1;...
%     0 0 0 -D(1,1)/M(1,1)                         M(2,2)*x_e(6)/M(1,1)                   M(2,2)*x_e(5)/M(1,1);...
%     0 0 0 -M(1,1)*x_e(6)/M(2,2)                  -D(2,2)/M(2,2)                         -M(1,1)*x_e(4)/M(2,2);...
%     0 0 0 (-M(2,2)*x_e(5)+M(1,1)*x_e(5))/M(3,3)  (-M(2,2)*x_e(4)+M(1,1)*x_e(4))/M(3,3)  -D(3,3)/M(3,3)];
% 
% BJ_c=[zeros(3,8);...
%     L(1,1)*T_conf(1,1)/M(1,1) L(2,2)*T_conf(1,2)/M(1,1) L(3,3)*T_conf(1,3)/M(1,1) L(4,4)*T_conf(1,4)/M(1,1) -L(1,1)*u_e(1)*T_conf(2,1)/M(1,1)                                         -L(2,2)*u_e(2)*T_conf(2,2)/M(1,1)                                         -L(3,3)*u_e(3)*T_conf(2,3)/M(1,1)                                         -L(4,4)*u_e(4)*T_conf(2,4)/M(1,1);...
%     L(1,1)*T_conf(2,1)/M(2,2) L(2,2)*T_conf(2,2)/M(2,2) L(3,3)*T_conf(2,3)/M(2,2) L(4,4)*T_conf(2,4)/M(2,2)  L(1,1)*u_e(1)*T_conf(1,1)/M(2,2)                                          L(2,2)*u_e(2)*T_conf(1,2)/M(2,2)                                          L(3,3)*u_e(3)*T_conf(1,3)/M(2,2)                                          L(4,4)*u_e(4)*T_conf(1,4)/M(2,2);...
%     L(1,1)*T_conf(3,1)/M(3,3) L(2,2)*T_conf(3,2)/M(3,3) L(3,3)*T_conf(3,3)/M(3,3) L(4,4)*T_conf(3,4)/M(3,3)  L(1,1)*u_e(1)*(thrConf(1,1)*T_conf(1,1)+thrConf(1,2)*T_conf(2,1))/M(3,3)  L(2,2)*u_e(2)*(thrConf(2,1)*T_conf(1,2)+thrConf(2,2)*T_conf(2,2))/M(3,3)  L(3,3)*u_e(3)*(thrConf(3,1)*T_conf(1,3)+thrConf(3,2)*T_conf(2,3))/M(3,3)  L(4,4)*u_e(4)*(thrConf(4,1)*T_conf(1,4)+thrConf(4,2)*T_conf(2,4))/M(3,3)];

%减少除法运算 提高程序运行速度
AJ_c=[0 0 -x_e(4)*sin(x_e(3))-x_e(5)*cos(x_e(3))  cos(x_e(3))  -sin(x_e(3)) 0;...
    0 0  x_e(4)*cos(x_e(3))-x_e(5)*sin(x_e(3))  sin(x_e(3))  cos(x_e(3))  0;...
    0 0 0 0 0 1;...
    0 0 0 -D(1,1)                         M(2,2)*x_e(6)                   M(2,2)*x_e(5);...
    0 0 0 -M(1,1)*x_e(6)                  -D(2,2)                         -M(1,1)*x_e(4);...
    0 0 0 (-M(2,2)*x_e(5)+M(1,1)*x_e(5))  (-M(2,2)*x_e(4)+M(1,1)*x_e(4))  -D(3,3)]./[1;1;1;M(1,1);M(2,2);M(3,3)];

BJ_c=[zeros(3,8);...
    T_efficiency(1,1)*T_conf(1,1) T_efficiency(2,2)*T_conf(1,2) T_efficiency(3,3)*T_conf(1,3) T_efficiency(4,4)*T_conf(1,4) -T_efficiency(1,1)*u_e(1)*T_conf(2,1)                                         -T_efficiency(2,2)*u_e(2)*T_conf(2,2)                                         -T_efficiency(3,3)*u_e(3)*T_conf(2,3)                                         -T_efficiency(4,4)*u_e(4)*T_conf(2,4);...
    T_efficiency(1,1)*T_conf(2,1) T_efficiency(2,2)*T_conf(2,2) T_efficiency(3,3)*T_conf(2,3) T_efficiency(4,4)*T_conf(2,4)  T_efficiency(1,1)*u_e(1)*T_conf(1,1)                                          T_efficiency(2,2)*u_e(2)*T_conf(1,2)                                          T_efficiency(3,3)*u_e(3)*T_conf(1,3)                                          T_efficiency(4,4)*u_e(4)*T_conf(1,4);...
    T_efficiency(1,1)*T_conf(3,1) T_efficiency(2,2)*T_conf(3,2) T_efficiency(3,3)*T_conf(3,3) T_efficiency(4,4)*T_conf(3,4)  T_efficiency(1,1)*u_e(1)*(thrConf(1,1)*T_conf(1,1)+thrConf(1,2)*T_conf(2,1))  T_efficiency(2,2)*u_e(2)*(thrConf(2,1)*T_conf(1,2)+thrConf(2,2)*T_conf(2,2))  T_efficiency(3,3)*u_e(3)*(thrConf(3,1)*T_conf(1,3)+thrConf(3,2)*T_conf(2,3))  T_efficiency(4,4)*u_e(4)*(thrConf(4,1)*T_conf(1,4)+thrConf(4,2)*T_conf(2,4))]./[1;1;1;M(1,1);M(2,2);M(3,3)];


%离散的Jacobian matrices 必须要离散 不然计算结果不对
[AJ,BJ]=c2d(AJ_c,BJ_c,Ts);

% AJ=eye(6,6)+Ts*AJ_c;%前向欧拉算法离散
% BJ=Ts*BJ_c;

end