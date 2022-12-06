%% 静态计算测试，周立师兄文章里的参数
clear all;
mooring_param = struct('length', 300, 'element_length', 10, 'element_number', 30,... 
                'node_number', 31, 'E', 9.16e9/2, 'diameter', 0.09, 'cross_area', 6.362e-3, ...
                'density', 126);
p0 = [0, 0, 0]';
p1 = [300, 0, 0]';
max_step = 30;
f_his = zeros(max_step, 1);
% figure
for i  = 1 : max_step
    [TE_begin, TE_end, Y_N, p_final, p_init0] = mooring_line_static_fcn(p0, p1, mooring_param);
    p1(1) = 300 + 0.1 * i;
    f_his(i, 1) = TE_begin(1);
    figure(1)
    plot(p_final(:, 1), p_final(:, 3));
    hold on
    drawnow;
    pause(0.001);
end

figure
plot(f_his(:, 1))
    
    
% figure(1)
% plot(p_init0(:, 1), p_init0(:, 3))
% hold on
% plot(p_final(:, 1), p_final(:, 3))
% hold off
% legend('初始形状', '最终形状');
% title('锚链形状变化');
% TE_begin


%% 静态计算测试,换一组参数，巴西文章里的参数
clear all;
mooring_param = struct('length', 400, 'element_length', 10, 'element_number', 40,... 
                'node_number', 41, 'E', 4.10e10, 'diameter', 0.0857, 'cross_area', 5.8e-3, ...
                'density', 25.5);
p0 = [0, 0, 0]';
p1 = [400, 0, 0]';
max_step = 1;
f_his = zeros(max_step, 1);
K_his = zeros(max_step, 1);
% figure
for i  = 1 : max_step
    [TE_begin, TE_end, Y_N, p_final, p_init0] = mooring_line_static_fcn(p0, p1, mooring_param);
    p1(1) = 400 + 0.01 * i;
    f_his(i, 1) = TE_begin(1);
%     K_his(i+1, 1) = 
    figure(1)
    plot(p_final(:, 1), p_final(:, 3));
    hold on
    drawnow;
    pause(0.001);
end

figure
plot(f_his(:, 1)/10)