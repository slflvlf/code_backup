function [x_est,y_est,P]=stateEstimator(x_e,x0_e,x0_est,u0,u0_e,y0,y0_est,P0,A0_J,B0_J)
% This function is used to estimate all states x(k) through measurable
% states y(k)(postion and yaw angle).
% Kalman Filter is used.

% Input
% x_e      这一步的nominal state
% x0_e     上一步的nominal state
% x0_est   上一步的估计值
% u0       MPC控制器求得的控制输入
% u0_e     nominal input
% y0       上一步的实际输出
% y0_est   上一步的预测输出
% P0       上一步的误差协方差矩阵
% A0_J     上一步的系统矩阵 Jacobian
% B0_J     上一步的控制矩阵 Jacobian

% Output
% x_est    这一步的状态估计值
% y_est    这一步的输出估计值 [x y psi]
% P        这一步的误差协方差矩阵，用于下一步计算

global C
global R

%% 计算Kalman Gain
K0=A0_J*P0*C'*(inv(C*P0*C'+R));

%% 计算这一步的状态估计值和输出
x_est=x_e+A0_J*(x0_est-x0_e)+B0_J*(u0-u0_e)+K0*(y0-y0_est);
y_est=C*x_est;

%% 计算这一步的误差协方差矩阵 用于下一步计算
P=(A0_J-K0*C)*P0*(A0_J-K0*C)'+K0*R*K0';

end