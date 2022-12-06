function [T_efficiency]=thrust_efficiency(u_e)
% This function is used to create thrust efficiency matrix.

% Input
% u_e      state control input [f1 f2 f3 f4 alpha1 alpha2 alpha3 alpha4]
% 
% Output
% T_efficiency   thrust efficiency matrix

global N;

% %test
% u_e=[-0.304689016424228;2.27506864133137;2.36727855422385;-0.333203150873476;0.0263669687031542;-0.00290397005082865;-0.204742912769201;0.0565881653235052];

%% �������Ƕ�ת����������ѵ����Χ [0 360] ˳ʱ��Ϊ��
%u_e�������Ƕ����ڴ�������ϵ�� ��ת��������������ϵ
alpha=rad2deg(u_e(N+1:2*N))-90;
%��ת����������ѵ����Χ [0 360]
for i=1:4
    if alpha(i)<0
        alpha(i)=alpha(i)+360;
    end
end

% %% ����ѵ����ɵ�rbf�������������ʧϵ��
% load net_rb_2
% eta=[0;0;0;0];
% %thr 1
% eta(1)=sim(net_rb_2,[alpha(2);alpha(1)]);
% %thr 2
% eta(2)=sim(net_rb_2,[alpha(1)-180;alpha(2)-180]);
% %thr 3
% eta(3)=sim(net_rb_2,[alpha(4)-180;alpha(3)-180]);
% %thr 4
% eta(4)=sim(net_rb_2,[alpha(3);alpha(4)]);

%% ��ֵ���� �ٶȿ�һ��
load alpha_interval
load eta_value

Alpha_forward=abs([alpha(2);alpha(1)-180;alpha(4)-180;alpha(3)]);
Alpha_rear=abs([alpha(1);alpha(2)-180;alpha(3)-180;alpha(4)]);
eta=interp2(alpha_interval,alpha_interval,eta_value,Alpha_forward,Alpha_rear);%����� ǰ�����[2 1 4 3] �����[1 2 3 4]

%% ����thrust efficiency matrix
T_efficiency=[diag(eta) zeros(N)];
end