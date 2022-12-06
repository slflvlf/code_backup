clear all; clc; clf;

mooring_param = struct('length', 300, 'element_length', 10, 'element_number', 30,... 
                'node_number', 31, 'E', 9.16e10, 'diameter', 0.09, 'cross_area', 6.362e-3, ...
                'density', 126, 'time_step', 0.1, 'coef_drag', 1, 'coef_addmass', 1,...
                'current', [0; 0; 0]);
p0=[0;0;2];      %坐标m
p1=[300;0;2];
p0_next=[0;0;2];      %坐标m
p1_next=[315;0;2];

Nhis = 100;
p0_his = zeros(3, Nhis);
p1_his = zeros(3, Nhis);
% YN_his = zeros(3*N+3, Nhis)
TR1_his = zeros(3, Nhis);
TRN_his = zeros(3, Nhis);

vel = 5;
dt = mooring_param.time_step;

for i = 1 : Nhis
    
    p0_his(1, i) = p0(1);
    p0_his(1, i) = p0(1) + i * 0;
    p0_his(2, i) = p0(2);
    p0_his(3, i) = p0(3);
    
    p1_his(1, i) = p1(1) + (i-1) * vel * dt;
    p1_his(2, i) = p1(2);
    p1_his(3, i) = p1(3);
end
for i = 1 : Nhis
    i
    p0_next = p0_his(:, i);
    p1_next = p1_his(:, i);


    [TR1, TRN] = mooringline_dynamic_force_fcn(p0, p1, p0_next, p1_next, mooring_param);
    p0 = p0_next;
    p1 = p1_next;

    TR1_his(:, i) = TR1;
    TRN_his(:, i) = TRN;

end


figure(1)
plot(p1_his(1, :), -TRN_his(1, :));
hold on
plot(p1_his(1, :), TR1_his(1, :));
hold off;
legend('TRN', 'TR1'); legend boxoff;


%% 
p0=[0;0;2];      %坐标m
p1=[303;0;2];
% p0_next=[0;0;2];      %坐标m
% p1_next=[315;0;2];

mooring_param = struct('length', 300, 'element_length', 10, 'element_number', 30,... 
                'node_number', 31, 'E', 9.16e10, 'diameter', 0.09, 'cross_area', 6.362e-3, ...
                'density', 126, 'time_step', 0.1, 'coef_drag', 1, 'coef_addmass', 1,...
                'current', [0; 0; 0]);
max_step = 20
TR1_his = zeros(3, max_step);
TRN_his = zeros(3, max_step);
for i = 1 : max_step
    p0_next = p0;
    p1_next = p1 + [1, 0, 0]';
    [TR1, TRN] = dynamic(p0, p1, p0_next, p1_next, mooring_param);
    TR1_his(:, i) = TR1;
    TRN_his(:, i) = TRN;
end

%% 
clear all; clc;
p0=[0;0;2];      %坐标m
p1=[310;0;2];
p0_next=[0;0;2];      %坐标m
p1_next=[315;0;2];
mooring_param = struct('length', 300, 'element_length', 3, 'element_number', 100,... 
                'node_number', 101, 'E', 9.16e10, 'diameter', 0.09, 'cross_area', 6.362e-3, ...
                'density', 126, 'time_step', 1, 'coef_drag', 1, 'coef_addmass', 1,...
                'current', [0; 0; 0]);
N = mooring_param.element_number;
aa=zeros(3*N+3,2);
vv=zeros(3*N+3,2);
% [TE_begin, TE_end, Y_N, p_static_final, p_init0] = mooring_line_static_fcn(p0, p1, mooring_param)
% 
% [TE_begin, TE_end, ~, ~, ~] = mooring_line_static_fcn(p0_next, p1_next, mooring_param)

[TR1, TRN] = mooringline_dynamic_force_fcn(p0, p1, p0_next, p1_next, mooring_param)

% [TR1, TRN, Y_N, p_dynamic_final, vv, aa] = mooring_line_dynamic_fcn(p0_next, p1_next, Y_N, vv, aa, mooring_param);
