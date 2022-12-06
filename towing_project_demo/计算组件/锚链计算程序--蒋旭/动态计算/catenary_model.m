% 悬链线理论
clear all; clc;
L=300;
d=0.09;
A=6.362e-3;
w=126;
E=9.16e10;
l_his = 290:0.02:310;
T_init = 1e4;
tic
for i = 1 : length(l_his)
    i
    l = l_his(i);
    eq = @(T)2*T/w*asinh(w*L/2/T)+T*L/E/A-l;
    T = fsolve(eq, T_init);
    T_init = T;
    T_his(i) = T/1e3;
    
    
    % 阻力计算
    s = l;
    v=0.2;
    Rt = 1.224e0*s*d/1e5*v^2*(1+1.122e0/T*d/10*(s/1e3)^2);
    Rt_his(i) = Rt/1e3;
end
figure(1)
plot(l_his, T_his);
% hold on 
% plot(l_his, Rt_his+T_his);
% hold off
% legend('T', 'Rt'); legend boxoff;


fsolve_time = toc



% %%
% tic
% syms Ts
% l_his = 300:0.01:310;
% for i = 1 : length(l_his)
%     l = l_his(i);
%     Tvpa = vpasolve(2*Ts/w*asinh(w*L/2/Ts)+Ts*L/E/A-l, Ts)/1e3;
%     Tvpa_his(i) = Tvpa;
% end
% plot(Tvpa_his)
% legend('T','Tvpa')
% vpasolve_time = toc

%% 静态计算测试
clear all;
mooring_param = struct('length', 300, 'element_length', 10, 'element_number', 30,... 
                'node_number', 31, 'E', 9.16e10, 'diameter', 0.09, 'cross_area', 6.362e-3, ...
                'density', 126);
p0 = [0, 0, 0]';
p1 = [300, 0, 0]';
max_step = 50;
p_his = 300.1:0.1:305;
f_his = zeros(max_step, 1);
% figure
for i  = 1 : length(p_his)
    [TE_begin, TE_end, Y_N, p_final, p_init0] = mooring_line_static_fcn(p0, p1, mooring_param);
    p1(1) = f_his(i)
    f_his(i, 1) = TE_begin(1);
    figure(1)
    plot(p_final(:, 1), p_final(:, 3));
    hold on
    drawnow;
    pause(0.001);
end

figure
plot(p_his, f_his(:, 1))
    
    
% figure(1)
% plot(p_init0(:, 1), p_init0(:, 3))
% hold on
% plot(p_final(:, 1), p_final(:, 3))
% hold off
% legend('初始形状', '最终形状');
% title('锚链形状变化');
% TE_begin
