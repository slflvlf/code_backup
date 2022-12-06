%% 静态计算测试
%% 悬链线方法和静态计算的比较
clear all; clc; clf;
mooring_param = struct('length', 300, 'element_length', 10, 'element_number', 30,... 
                'node_number', 31, 'E', 9.16e10, 'diameter', 0.09, 'cross_area', 6.362e-3, ...
                'density', 126);
p0 = [0, 0, 0]';
p1 = [300, 0, 0]';
p_his = 299:0.1:320;
max_step = length(p_his);
f_his = zeros(max_step, 1);
f1_his = zeros(max_step, 1);

f1= 1e4;

% figure
for i  = 1 : max_step
    p1(1) = p_his(i);
    [TE_begin, TE_end, Y_N, p_final, p_init0] = mooring_line_static_fcn(p0, p1, mooring_param);
    i
    l_init = get_length(p_init0);
    l_final = get_length(p_final);

    
    f_his(i, 1) = TE_begin(1);
    
    l = p_his(i);
    f1 = mooring_line_catenary_fcn(l, f1, mooring_param);   %求解需要初值
    f1_his(i, 1) = f1;
    
    

    figure(1)
    if rem(i, 30) == 0
    plot(p_final(:, 1), p_final(:, 3));
    hold on
    drawnow;
    end

end
% 
% figure
% plot(f_his(:, 1))

    
% figure(2)
% plot(p_init0(:, 1), p_init0(:, 3))
% hold on
% plot(p_final(:, 1), p_final(:, 3))
% hold off
% legend('初始形状', '最终形状');
% title('锚链形状变化');
% TE_begin

figure(2)
plot(p_his, f_his(:, 1))
hold on
plot(p_his, f1_his(:, 1));
hold off
legend('static', 'catenary');
legend boxoff

function l = get_length(p_final)
    l_ele = 0;  l = 0;
    for i = 1 : length(p_final)-1
       p1 = p_final(i, :)';
       p2 = p_final(i+1, :)';
       l_ele = norm(p1-p2);
       l = l+l_ele;
    end

end 