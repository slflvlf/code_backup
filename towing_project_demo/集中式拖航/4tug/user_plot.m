% 画图
i=k;
if i == Tsim
    plot_pos_vel(p0_hat_his, pd0_his, v0_hat_his, vd0_his, 2);
%     plot_pos_vel(p_hat_his, pd_his, v_hat_his, vd_his, 2);
%     plot_thruster(tau_his, taur_his, f_his, a_his, 3);
end

%% 实时运动显示
if motion_display_flag == 1
if i == 1
        hull_x1 = [-16, 12, 16, 12, -16, -16];
        hull_y1 = [6, 6, 0, -6, -6, 6];
        figure(1)
        title('Motion of the ships');
        axis([-50, 50, -50, 50]*0.5);
        axis equal;
        grid on;
    
        hgtrans = initial_motion_fig(hull_x1, hull_y1, p_hat, 0, [5, 3], 'y');
        update_motion_fig(hgtrans, p_hat);
        
        pause(0.0001);
    
else
        update_motion_fig(hgtrans, p_hat);
        hold on 
        plot(p(1), p(2), '.');
        pause(0.0001);
    
end
end





function hgtrans = initial_motion_fig(hull_x, hull_y, p, id, id_position, ship_color)
    hgtrans = hgtransform;
    fill(hull_x, hull_y, ship_color, 'parent',hgtrans);
    line(0,0,'marker','o','markeredgecolor','k','markerfacecolor','k','parent',hgtrans); 
    text(p(1)+id_position(1), p(2)+id_position(2), num2str(id), 'parent', hgtrans);   
end

function [] = update_motion_fig(hgtrans, p)
    makehg = makehgtform('translate',[p(1) p(2) 0]) * makehgtform('zrotate',p(3));
    set(hgtrans, 'Matrix',makehg); 
end




%% 位置和速度画图， i是图的编号
function [] = plot_pos_vel(p_his, pd_his, v_his, vd_his, p_hat_his, v_hat_his, i)
    % 船舶id 对应着图的id
    vd_his = dotpd_to_vd(vd_his, pd_his);
    figure(i)
    title('位置和速度');

    subplot(3, 2, 1);
    plot(p_his(1, :))
    hold on
    plot(pd_his(1, :))
    hold off
    legend('p', 'pd')
    legend boxoff

subplot(3, 2, 3);
plot(p_his(2, :))
hold on
plot(pd_his(2, :))
hold off

subplot(3, 2, 5);
plot(p_his(3, :)/pi*180)
hold on
plot(pd_his(3, :)/pi*180)
hold off

% velocity

subplot(3, 2, 2);
plot(v_his(1, :))
hold on
plot(vd_his(1, :))
hold off
legend('v', 'vd')
legend boxoff

subplot(3, 2, 4);
plot(v_his(2, :))
hold on
plot(vd_his(2, :))
hold off

subplot(3, 2, 6);
plot(v_his(2, :))
hold on
plot(vd_his(2, :))
hold off

    
    
end

%% 画图 推力分配， i是图的编号
function [] = plot_thruster(tau_his, taur_his, f_his, a_his, fig_id)
%     figure
    figure(fig_id)
title('推力分配');

subplot(3, 3, 1);
plot(tau_his(1, :));
hold on
plot(taur_his(1, :));
hold off
legend('tau', 'tau_r')
legend boxoff

subplot(3, 3, 4)
plot(tau_his(2, :));
hold on
plot(taur_his(2, :));
hold off

subplot(3, 3, 7)
plot(tau_his(3, :));
hold on
plot(taur_his(3, :));
hold off


subplot(3, 3, 2)
plot(f_his(1,:));

subplot(3, 3, 5)
plot(f_his(2,:));

subplot(3, 3, 8)
plot(f_his(3,:));

subplot(3, 3, 3)
plot(a_his(1,:));

subplot(3, 3, 6)
plot(a_his(2,:));

subplot(3, 3, 9)
plot(a_his(3,:));
    
end



%% 将规划轨迹速(dot_pd)度转换为体坐标上的真实速度
function vd_his = dotpd_to_vd(vd_his, pd_his)
    
    for i = length(vd_his(1, :))
        R = rotate_matrix(pd_his(3, i));
        vd_his(:, i) = R' * vd_his(:, i);
    end
end


function R = rotate_matrix(fai)
    
    R = [cos(fai),  -sin(fai),  0;
         sin(fai),  cos(fai),   0;
         0,          0,         1];   
end


