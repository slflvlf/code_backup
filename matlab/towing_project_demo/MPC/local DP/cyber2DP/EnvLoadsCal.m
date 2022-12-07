function [EnvLoads]=EnvLoadsCal(WaveDirection,WindVelocity,CurrentVelocity)
%This function is used to calculate the environmental loads according to
%test data.

%Input
%WaveDirection       浪向角（风浪流同向）
%WindVelocity        风速 m/s
%CurrentVelocity     流速 m/s
%
%Output
%EnvLoads            环境载荷（风、浪、流）

% %% test
% WaveDirection=180;
% WindVelocity=5;%m/s
% CurrentVelocity=0.512;%m/s
%% Wind Loads
% [diretion xloads yloads yawloads]
% [degree KN KN KN*m]
windloads = ...
[0	56.34	7.39	28.39
30	63.06	49.92	-754.16
60	18.01	94.78	-764.72
90	-21.36	117.40	304.67
120	-12.18	127.39	1244.42
150	-24.93	84.47	1909.98
180	-33.67	-8.18	-212.28];%试验数据

WLoads=zeros(1,3);%Initialize,3dof
for j = 1:3 
    WLoads(1,j) = interp1(windloads(:,1),windloads(:,j+1),WaveDirection,'spline');
end
WindLoads=WLoads/15^2*WindVelocity^2;

%% Current Loads
% [diretion xloads yloads yawloads]
% [degree KN KN KN*m]
currentloads = ...
[0	22.75	-1.48	65.78
30	16.76	99.72	-2041.04
60	-7.51	192.64	-3070.72
90	-27.07	225.91	-1320.59
120	-11.65	195.87	514.40
150	-42.44	94.66	1077.06
180	-34.32	4.70	-35.92];

CLoads=zeros(1,3);%Initialize,3dof
for j = 1:3
    CLoads(1,j) = interp1(currentloads(:,1),currentloads(:,j+1),WaveDirection,'spline');
end
CurrentLoads=CLoads/1^2*CurrentVelocity^2;

%% Wave Loads
WaveLoads=MeanWaveDrift(WaveDirection,WindVelocity);

%% Total
EnvLoads=WindLoads+CurrentLoads+WaveLoads;

end