%% 动态计算， 调用

clear all; clc;
tic
mooring_param = struct('length', 300, 'element_length', 10, 'element_number', 30,... 
                'node_number', 31, 'E', 9.16e10, 'diameter', 0.09, 'cross_area', 6.362e-3, ...
                'density', 126, 'time_step', 0.1, 'coef_drag', 1, 'coef_addmass', 1,...
                'current', [0; 0; 0]);
p0 = [0, 0, 0]';
p1 = [300, 0, 0]';
H=1000;          %水深
g=9.80665;
L = mooring_param.element_length;            %分段长m
N = mooring_param.element_number;
Time=100;        %时间 s

% p0_next=[0;0;2];      %坐标m
% p1_next=[302;0;2];

Nhis = 500;
p0_his = zeros(3, Nhis);
p1_his = zeros(3, Nhis);
YN_his = zeros(3*N+3, Nhis);

for i = 1 : Nhis
    p0_his(1, i) = p0(1);
    p0_his(1, i) = p0(1) + i * 0.1;
    p0_his(2, i) = p0(2);
    p0_his(3, i) = p0(3);
    
    p1_his(1, i) = p1(1) + i * 0.1;
    p1_his(2, i) = p1(2);
    p1_his(3, i) = p1(3);
end


aa=zeros(3*N+3,2);
vv=zeros(3*N+3,2);
TR1_his = zeros(3, Nhis);
TRN_his = zeros(3, Nhis);
TE_begin_his = zeros(3, Nhis);
TE_end_his = zeros(3, Nhis);
figure(3)
[TE_begin, TE_end, Y_N, p_static_final, p_init0] = mooring_line_static_fcn(p0, p1, mooring_param);

for i = 1 : Nhis
    TE_begin_his(:, i) = TE_begin;
    TE_end_his(:, i) = TE_end;
%     [TE_begin, TE_end, ~, ~, ~] = mooring_line_static_fcn(p0_next, p1_next, mooring_param);
    i
    p0_next =  p0_his(:, i);
    
    p1_next =  p1_his(:, i);
    [TE_begin, TE_end, ~, ~, ~] = mooring_line_static_fcn(p0_next, p1_next, mooring_param);

    [TR1, TRN, Y_N, p_dynamic_final, vv, aa] = mooring_line_dynamic_fcn(p0_next, p1_next, Y_N, vv, aa, mooring_param);
    YN_his(:, i) = Y_N;
    TR1_his(:,i) = TR1;
    TRN_his(:, i) = TRN;
    
%     if rem(i, 2)==1
%         plot(p_dynamic_final(:, 1), p_dynamic_final(:, 3))
%         hold on
%     end
        
end
title('动态计算锚链形态');
% hold off

figure(5)
plot(1:Nhis, TR1_his(1, :));
hold on
plot(1:Nhis, -TRN_his(1, :));
hold on
plot(1:Nhis, TE_begin_his(1, :));
hold on
plot(1:Nhis, -TE_end_his(1, :));
hold off
legend('TR1_{dynamic}', 'TRN_{dynamic}', 'TR1_{static}', 'TRN_{static}');
legend boxoff

toc

%% 动态和静态对比
clear all; clc;
mooring_param = struct('length', 300, 'element_length', 3, 'element_number', 100,... 
                'node_number', 101, 'E', 9.16e10, 'diameter', 0.09, 'cross_area', 6.362e-3, ...
                'density', 126, 'time_step', 1, 'coef_drag', 1, 'coef_addmass', 1,...
                'current', [0; 0; 0]);
p0 = [0, 0, 0]';
p1 = [304, 0, 0]';
H=1500;          %水深
g=9.80665;
L = mooring_param.element_length;            %分段长m
N = mooring_param.element_number;
Time=100;        %时间 s

p0_next=[0;0;2];      %坐标m
p1_next=[302;0;2];

Nhis = 10;
p0_his = zeros(3, Nhis);
p1_his = zeros(3, Nhis);
YN_his = zeros(3*N+3, Nhis);

for i = 1 : Nhis
    p0_his(1, i) = p0(1);
%     p0_his(1, i) = p0(1) + i * 0.5;
    p0_his(2, i) = p0(2);
    p0_his(3, i) = p0(3);
    
    p1_his(1, i) = p1(1) + i * 1;
    p1_his(2, i) = p1(2);
    p1_his(3, i) = p1(3);
end



aa=zeros(3*N+3,2);
vv=zeros(3*N+3,2);
TR1_his = zeros(3, Nhis);
TRN_his = zeros(3, Nhis);
TE_begin_his = zeros(3, Nhis);
TE_end_his = zeros(3, Nhis);
figure(3)
[TE_begin, TE_end, Y_N, p_static_final, p_init0] = mooring_line_static_fcn(p0, p1, mooring_param);

for i = 1 : Nhis
    TE_begin_his(:, i) = TE_begin;
    TE_end_his(:, i) = TE_end;
%     [TE_begin, TE_end, ~, ~, ~] = mooring_line_static_fcn(p0_next, p1_next, mooring_param);
    i
    p0_next =  p0_his(:, i);
    p1_next =  p1_his(:, i);
    [TE_begin, TE_end, ~, ~, ~] = mooring_line_static_fcn(p0_next, p1_next, mooring_param);

    [TR1, TRN, Y_N, p_dynamic_final, vv, aa] = mooring_line_dynamic_fcn(p0_next, p1_next, Y_N, vv, aa, mooring_param);
    YN_his(:, i) = Y_N;
    TR1_his(:,i) = TR1;
    TRN_his(:, i) = TRN;
    
    if rem(i, 2)==1
        plot(p_dynamic_final(:, 1), p_dynamic_final(:, 3))
        hold on
    end
        
end
title('动态计算锚链形态');
hold off

figure(5)
plot(1:Nhis, TR1_his(1, :));
hold on
plot(1:Nhis, -TRN_his(1, :));
hold on
plot(1:Nhis, TE_begin_his(1, :));
hold on
plot(1:Nhis, -TE_end_his(1, :));
hold off
legend('TR1_{dynamic}', 'TRN_{dynamic}', 'TR1_{static}', 'TRN_{static}');
legend boxoff



% figure(2)
% % plot(p_dynamic_init(:, 1), p_dynamic_init(:, 3))
% % hold on
% plot(p_dynamic_final(:, 1), p_dynamic_final(:, 3))
% % hold off
% % legend('初始形状', '最终形状'); legend boxoff;

%% 螺旋桨推力和转速
rho = 1025; %密度
D = 0.140; %直径
Kt = 0.6; %推力系数
coef = Kt * rho * D^4;
force = 1;
n = sqrt(force/coef)

%% pid 增益调节
clear all; clc;
T = 120
M = 1.3*100
Iz = M*0.9^2
MM = diag([M, M, Iz]);
D = diag([MM(1, 1)/40, MM(2, 2)/20, MM(3, 3)/10]);
Kp = (2*pi/T)^2*MM
temp = Kp * MM;
Kd = 1.4 * diag([sqrt(temp(1, 1)),sqrt(temp(2, 2)),sqrt(temp(3, 3)) ])-D


