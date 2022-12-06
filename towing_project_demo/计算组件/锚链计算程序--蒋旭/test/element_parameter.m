function [N,W,EA,D]=element_parameter(mooring_param)

% mooring_param = struct('length', 300, 'element_length', 10, 'element_number', 30,... 
%                 'node_number', 31, 'E', 9.16e10, 'diameter', 0.09, 'cross_area', 6.362e-3, ...
%                 'density', 126, 'time_step', 0.1, 'coef_drag', 1, 'coef_addmass', 1,...
%                 'current', [0; 0; 0]);

n = mooring_param.element_number; %锚链分为多少段
L1_n=n;
L2_n=0;
L3_n=0;

% L1_n=3;
% L2_n=53;
% L3_n=29;
N=L1_n+L2_n+L3_n;
%% 单位长度水中质量
density = mooring_param.density;
w1=density;     %kg/m
w2=density;
w3=density;

% w1=139.2;     %kg/m
% w2=4.2;
% w3=139.2;
W = zeros(N, 1);
W(1:L1_n,1)=w1;
W(L1_n+1:L1_n+L2_n,1)=w2;
W(L2_n+1:N,1)=w3;
%% EA
ea = mooring_param.E * mooring_param.cross_area;
EA1=ea;   %单位N
EA2=ea;                                     
EA3=ea;

% EA1=633000000;
% EA2=264000000;                                     
% EA3=633000000;
EA = zeros(N, 1);
EA(1:L1_n,1)=EA1;
EA(L1_n+1:L1_n+L2_n,1)=EA2;
EA(L2_n+1:N,1)=EA3;
%% 定义直径
d = mooring_param.diameter;
D1=d;             %单位m
D2=d;                                           %3段的外径
D3=d;

% D1=0.084;
% D2=0.16;                                           %3段的外径
% D3=0.084;
D=zeros(N,1);
D(1:L1_n,1)=D1;
D(L1_n+1:L1_n+L2_n,1)=D2;
D(L1_n+L2_n+1:N,1)=D3;

end