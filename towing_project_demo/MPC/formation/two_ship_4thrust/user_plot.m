% 画图
if i == tsim
    plot_pos_vel1(p1_his, pd1_his, p1_hat_his, v1_his, vd1_his,  v1_hat_his, 1);
    plot_pos_vel1(p2_his, pd2_his, p2_hat_his, v2_his, vd2_his,  v2_hat_his, 2);
    plot_thruster_fa(f1_his, a1_his, df1_his, da1_his, 3);
    plot_thruster_fa(f2_his, a2_his, df2_his, da2_his, 4);
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

%% 位置和速度
function [] = plot_pos_vel1(p1_his, pd1_his,p1_hat_his, v1_his, vd1_his,  v1_hat_his, i)
figure(i)
subplot(3, 2, 1)
plot(p1_his(:, 1));
hold on
plot(p1_hat_his(:, 1));
hold on
plot(pd1_his(:, 1))
hold off
legend("p", "p_{hat}", "pd")
legend boxoff
subplot(3, 2, 3)
plot(p1_his(:, 2));
hold on
plot(p1_hat_his(:, 2));
hold on
plot(pd1_his(:, 2))
hold off

subplot(3, 2, 5)
plot(p1_his(:, 3)/pi*180);
hold on
plot(p1_hat_his(:, 3)/pi*180);
hold on
plot(pd1_his(:, 3)/pi*180)
hold off
subplot(3, 2, 2)
plot(v1_his(:, 1))
hold on 
plot(v1_hat_his(:, 1));
hold on 
plot(vd1_his(:, 1));
hold off
legend("v", "v_{hat}", "v_d"); legend boxoff;
subplot(3, 2, 4)
plot(v1_his(:, 2));
hold on
plot(v1_hat_his(:, 2));
hold on 
plot(vd1_his(:, 2));
hold off
subplot(3, 2, 6)
plot(v1_his(:, 3)/pi*180);
hold on
plot(v1_hat_his(:, 3)/pi*180);
hold on 
plot(vd1_his(:, 3)/pi*180);
hold off
end

%% 推力显示
function [] = plot_thruster_fa(f_his, a_his, df_his, da_his, fig_id)
figure(fig_id)
title("fig_id")
subplot(2, 2, 1)
plot(f_his(:, 1));
hold on
plot(f_his(:, 2));
hold on
plot(f_his(:, 3));
hold on
plot(f_his(:, 4));
hold off
legend("f_1", "f_2", "f_3", "f_4");
legend boxoff

subplot(2, 2, 3)
plot(a_his(:, 1)/pi*180);
hold on
plot(a_his(:, 2)/pi*180);
hold on
plot(a_his(:, 3)/pi*180);
hold on
plot(a_his(:, 4)/pi*180);
hold off
legend("a_1", "a_2", "a_3", "a_4");
legend boxoff

subplot(2, 2, 2)
plot(df_his(:, 1));
hold on
plot(df_his(:, 2));
hold on
plot(df_his(:, 3));
hold on
plot(df_his(:, 4));
hold off
legend("df_1", "df_2", "df_3", "df_4");
legend boxoff

subplot(2, 2, 4)
plot(da_his(:, 1)/pi*180);
hold on
plot(da_his(:, 2)/pi*180);
hold on
plot(da_his(:, 3)/pi*180);
hold on
plot(da_his(:, 4)/pi*180);
hold off
legend("da_1", "da_2", "da_3", "da_4");
legend boxoff

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


