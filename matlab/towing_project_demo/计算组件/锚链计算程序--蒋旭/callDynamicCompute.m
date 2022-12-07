clear all; clc;
g=9.80665;
L=20;            %分段长m
H=1500;          %水深
N = 50;
Time=100;        %时间 s
p0=[0;0;2];      %坐标m
p1=[1050;0;2];
p0_next=[2;0;2];      %坐标m
p1_next=[1302;0;2];

Nhis = 100;
p0_his = zeros(3, Nhis);
p1_his = zeros(3, Nhis);
YN_his = zeros(3*N+3, Nhis);

for i = 1 : Nhis
    p0_his(1, i) = p0(1) + i * 0.5;
    p0_his(2, i) = p0(2);
    p0_his(3, i) = p0(3);
    
    p1_his(1, i) = p1(1) + i * 1.5;
    p1_his(2, i) = p1(2);
    p1_his(3, i) = p1(3);
end


aa=zeros(3*N+3,2);
vv=zeros(3*N+3,2);
TR1_his = zeros(3, Nhis);
TRN_his = zeros(3, Nhis);
figure(3)
[TE, Y_N, p_static_final] = mooring_line_static_fcn(p0, p1);
for i = 1 : Nhis
    p0_next =  p0_his(:, i);
    p1_next =  p1_his(:, i);
    [TR1, TRN, Y_N, p_dynamic_final, vv, aa] = mooring_line_dynamic_fcn(p0_next, p1_next, Y_N, vv, aa);
    YN_his(:, i) = Y_N;
    TR1_his(:,i) = TR1;
    TRN_his(:, i) = TRN;
    
    if rem(i, 20)==1
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
hold off
legend('TR1', 'TRN');
legend boxoff



% figure(2)
% % plot(p_dynamic_init(:, 1), p_dynamic_init(:, 3))
% % hold on
% plot(p_dynamic_final(:, 1), p_dynamic_final(:, 3))
% % hold off
% % legend('初始形状', '最终形状'); legend boxoff;



