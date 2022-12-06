function [N,W,EA,D]=element_parameter(mooring_param)
L1_n=mooring_param.element_number;
L2_n=0;
L3_n=0;

% L1_n=3;
% L2_n=53;
% L3_n=29;
N=L1_n+L2_n+L3_n;
%% ��λ����ˮ������
w1=mooring_param.density;     %kg/m
w2=w1;
w3=w1;

% w1=139.2;     %kg/m
% w2=4.2;
% w3=139.2;
W(1:L1_n,1)=w1;
W(L1_n+1:L1_n+L2_n,1)=w2;
W(L2_n+1:N,1)=w3;
%% EA
E = mooring_param.E;
A = mooring_param.cross_area;
EA = E * A;

EA1=EA;   %��λN
EA2=EA;                                     
EA3=EA;

% EA1=700e6;   %��λN
% EA2=700e6;                                     
% EA3=700e6;

% EA1=633000000;
% EA2=264000000;                                     
% EA3=633000000;
EA(1:L1_n,1)=EA1;
EA(L1_n+1:L1_n+L2_n,1)=EA2;
EA(L2_n+1:N,1)=EA3;
%% ����ֱ��
diameter = mooring_param.diameter; %��λm
D1=diameter;             %��λm
D2=diameter;                                           %3�ε��⾶
D3=diameter;
% D1=0.084;             %��λm
% D2=0.084;                                           %3�ε��⾶
% D3=0.084;

% D1=0.084;
% D2=0.16;                                           %3�ε��⾶
% D3=0.084;
D=zeros(N,1);
D(1:L1_n,1)=D1;
D(L1_n+1:L1_n+L2_n,1)=D2;
D(L1_n+L2_n+1:N,1)=D3;

end