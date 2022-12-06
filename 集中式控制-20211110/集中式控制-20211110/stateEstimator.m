function [x_est,y_est,P]=stateEstimator(x_e,x0_e,x0_est,u0,u0_e,y0,y0_est,P0,A0_J,B0_J)
% This function is used to estimate all states x(k) through measurable
% states y(k)(postion and yaw angle).
% Kalman Filter is used.

% Input
% x_e      ��һ����nominal state
% x0_e     ��һ����nominal state
% x0_est   ��һ���Ĺ���ֵ
% u0       MPC��������õĿ�������
% u0_e     nominal input
% y0       ��һ����ʵ�����
% y0_est   ��һ����Ԥ�����
% P0       ��һ�������Э�������
% A0_J     ��һ����ϵͳ���� Jacobian
% B0_J     ��һ���Ŀ��ƾ��� Jacobian

% Output
% x_est    ��һ����״̬����ֵ
% y_est    ��һ�����������ֵ [x y psi]
% P        ��һ�������Э�������������һ������

global C
global R

%% ����Kalman Gain
K0=A0_J*P0*C'*(inv(C*P0*C'+R));

%% ������һ����״̬����ֵ�����
x_est=x_e+A0_J*(x0_est-x0_e)+B0_J*(u0-u0_e)+K0*(y0-y0_est);
y_est=C*x_est;

%% ������һ�������Э������� ������һ������
P=(A0_J-K0*C)*P0*(A0_J-K0*C)'+K0*R*K0';

end