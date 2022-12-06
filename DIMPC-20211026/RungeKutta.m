function [x_e_next,AJ,BJ]=RungeKutta(x_e,U_e,w_estimated)
% This function is used to calculate the x_e at time k+1 and Jacobian matrices AJ(k) and BJ(k).
% RungeKuttaֻ����norminal values!!!

% Input
% x_e             kʱ�̵�seed״̬����          [x y psi u v r]'
% U_e             kʱ�̵�seed������������      Nc x [f1 f2 f3 f4 alpha1 alpha2 alpha3 alpha4]'
% w_estimated     kʱ�̵�RBFNN���ƵĻ�����     [wx wy wpsi]'  3x1
% 
%
% Output
% x_e_next        k+1ʱ�̵�seed״̬����   [x y psi u v r]
% AJ              kʱ�̵�Jacobian���� 6x6 ��״̬������ƫ��
% BJ              kʱ�̵�Jacobian���� 6x8 �Կ���������ƫ��

global M;global D;
global G;
global N;  %�ƽ���������
global thrConf;
global Ts;

%% RungeKutta������һʱ�̵�seed state x_e_next
% Rotation matrix and coriolis matrix
[R,Cv]=create_RotMat_CvMat(x_e);            %ʱ��

% ������ģ�͵�ϵ������
u_e=U_e(1:2*N);%kʱ�̵Ŀ�������
u_e_next=U_e(2*N+1:2*2*N);%k+1ʱ�̵Ŀ�������

% �������þ���
T_conf=thrust_configuration(u_e);
T_conf_next=thrust_configuration(u_e_next);

% % ������ʧ����
% T_efficiency=thrust_efficiency(u_e);
% T_efficiency_next=thrust_efficiency(u_e_next);
% Ĭ��������ʧϵ��=1
T_efficiency=[eye(4) zeros(4)];
T_efficiency_next=[eye(4) zeros(4)];

A=[zeros(3) R;zeros(3) -inv(M)*(Cv+D)];     %ʱ��
B=[zeros(3);inv(M)]*T_conf*T_efficiency;               %ʱ�� B(k)=B*T_conf*T_efficiency������B=[0;inv(M)]
B_next=[zeros(3);inv(M)]*T_conf_next*T_efficiency_next;

% ����RungeKuttaϵ��
k1=A*x_e+B*u_e+G*w_estimated;
k2=A*(x_e+0.5*Ts*k1)+B*u_e+G*w_estimated;
k3=A*(x_e+0.5*Ts*k2)+B*u_e+G*w_estimated;
k4=A*(x_e+Ts*k3)+B_next*u_e_next+G*w_estimated;

% ������һʱ�̵�seed״̬����
x_e_next=x_e+Ts*(k1+2*k2+2*k3+k4)*0.1667;

%% Jacobian matrices ����MPCԤ��ģ��
%������ �±�c=continuous
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

%���ٳ������� ��߳��������ٶ�
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


%��ɢ��Jacobian matrices ����Ҫ��ɢ ��Ȼ����������
[AJ,BJ]=c2d(AJ_c,BJ_c,Ts);

% AJ=eye(6,6)+Ts*AJ_c;%ǰ��ŷ���㷨��ɢ
% BJ=Ts*BJ_c;

end