% 画图


%% local DP 结果展示
if i == max_step
    % 各个船的位置和速度曲线
    plot_pos_vel(p1_his, pd1_his, v1_his, vd1_his, 2);
    plot_pos_vel(p2_his, pd2_his, v2_his, vd2_his, 3);
    plot_pos_vel(p3_his, pd3_his, v3_his, vd3_his, 4);
    plot_pos_vel(p4_his, pd4_his, v4_his, vd4_his, 5);
    
%     plot_length_and_error(length_his, length_error_his,  6, 7, 8);

    
%     plot_controller_force(tau1_his, tauc1_his, tau2_his, tauc2_his,...
%                                 tau3_his, tauc3_his, tau4_his, tauc4_his, 9, 10);
                            
%     plot_coop_error(s1_his, s2_his, s3_his, s4_his, 11);

%     plot_controller_force(tau1_his, tauc1_his, tau2_his, tauc2_his, 6);
%     plot_controller_force(tau1_his, tau2_his, 6);
%     plot_thruster(tau1_his, taur1_his, f1_his, a1_his, 10);
%     plot_thruster(tau2_his, taur2_his, f2_his, a2_his, 11);
%     plot_thruster(tau3_his, taur3_his, f3_his, a3_his, 12);
%     plot_thruster(tau4_his, taur4_his, f4_his, a4_his, 13);

end

%% 实时运动显示
if display_motion_flag == 1
if i == 1
        hull_x1 = [-16, 12, 16, 12, -16, -16];
        hull_y1 = [6, 6, 0, -6, -6, 6];
        figure(1)
        title('Motion of the ships');
        axis([-50, 50, -50, 50]*2);
        axis equal;
    
        hgtrans0 = initial_motion_fig(hull_x1, hull_y1, p0, 0, [5, 3], 'y');
        hgtrans1 = initial_motion_fig(hull_x1, hull_y1, p1, 1, [25,-20], 'm');
        hgtrans2 = initial_motion_fig(hull_x1, hull_y1, p2, 2, [25, 20], 'm');
        hgtrans3 = initial_motion_fig(hull_x1, hull_y1, p3, 3, [-25,20], 'm');
        hgtrans4 = initial_motion_fig(hull_x1, hull_y1, p4, 4, [-25, -20], 'm');
        update_motion_fig(hgtrans0, p0);
        update_motion_fig(hgtrans1, p1);
        update_motion_fig(hgtrans2, p2);
        update_motion_fig(hgtrans3, p3);
        update_motion_fig(hgtrans4, p4);
        pause(0.0001);
    
else i>1&&i<max_step
        update_motion_fig(hgtrans0, p0);
        hold on 
        plot(p0(1), p0(2), '.');
        update_motion_fig(hgtrans1, p1);
        update_motion_fig(hgtrans2, p2);
        update_motion_fig(hgtrans3, p3);
        update_motion_fig(hgtrans4, p4);
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

%% 位置和速度画图， i是图的编号
function [] = plot_pos_vel(p_his, pd_his, v_his, vd_his, i)
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

%% 将规划轨迹速度转换为体坐标上的真实速度
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


%% 船舶间距误差
function [] = plot_length_and_error(length_his, length_error_his,  ...
                                    fig_id_length1, fig_id_length2, fig_id_error)
    figure(fig_id_length1);
    title('和leader的间距');
%     subplot(1, 2, 1)
    plot(length_his(1, :))
    hold on
    plot(length_his(2, :))
    hold on
    plot(length_his(3, :))
    hold on
    plot(length_his(4, :))
    hold off
    legend('length01', 'length02', 'length03', 'length04')
    legend boxoff  
    
    figure(fig_id_length2)
    title('follower之间的间距');
    plot(length_his(5, :))
    hold on
    plot(length_his(6, :))
    hold on
    plot(length_his(7, :))
    hold on
    plot(length_his(8, :))
    hold off
    legend( 'length12', 'length13', 'length23', 'length34');
    legend boxoff
    
    
    figure(fig_id_error)
    title('多船间距误差')
    plot(length_error_his(1, :))
    hold on
    plot(length_error_his(2, :))
    hold on
    plot(length_error_his(3, :))
    hold on
    plot(length_error_his(4, :))
    hold on
    plot(length_error_his(5, :))
    hold on
    plot(length_error_his(6, :))
    hold on
    plot(length_error_his(7, :))
    hold on
    plot(length_error_his(8, :))
    hold off
    legend('length01error', 'length02error', 'length03error', 'length04error',...
        'length12error', 'length13error', 'length23error', 'length34error');
    legend boxoff
    
    
end

%% 船舶控制力加入一致性协议后的对比
function [] = plot_controller_force(tau1_his, tauc1_his, tau2_his, tauc2_his,...
                                tau3_his, tauc3_his, tau4_his, tauc4_his, fig_id1, fig_id2)
% function [] = plot_controller_force(tau1_his,  tau2_his, tau3  fig_id)
    figure(fig_id1);
    title('船1和船2：协同控制控制力');
    
    subplot(3, 2, 1)
    plot(tau1_his(1, :));
    hold on 
    plot(tauc1_his(1, :));
    hold off
    legend('tau1', 'tauc1');  legend boxoff;
    
    subplot(3, 2, 3)
    plot(tau1_his(2, :));
    hold on 
    plot(tauc1_his(2, :));
    hold off
    
    subplot(3, 2, 5)
    plot(tau1_his(3, :));
    hold on 
    plot(tauc1_his(3, :));
    hold off
    
    
    subplot(3, 2, 2)
    plot(tau2_his(1, :));
    hold on 
    plot(tauc2_his(1, :));
    hold off
    legend('tau2', 'tauc2');  legend boxoff;
    
    subplot(3, 2, 4)
    plot(tau2_his(2, :));
    hold on 
    plot(tauc2_his(2, :));
    hold off
    
    subplot(3, 2, 6)
    plot(tau2_his(3, :));
    hold on 
    plot(tauc2_his(3, :));
    hold off
    
    figure(fig_id2);
    title('船3和船4：协同控制控制力');
    
    subplot(3, 2, 1)
    plot(tau3_his(1, :));
    hold on 
    plot(tauc3_his(1, :));
    hold off
    legend('tau3', 'tauc3');  legend boxoff;
    
    subplot(3, 2, 3)
    plot(tau3_his(2, :));
    hold on 
    plot(tauc3_his(2, :));
    hold off
    
    subplot(3, 2, 5)
    plot(tau3_his(3, :));
    hold on 
    plot(tauc3_his(3, :));
    hold off
    
    
    subplot(3, 2, 2)
    plot(tau4_his(1, :));
    hold on 
    plot(tauc4_his(1, :));
    hold off
    legend('tau4', 'tauc4');  legend boxoff;
    
    subplot(3, 2, 4)
    plot(tau4_his(2, :));
    hold on 
    plot(tauc4_his(2, :));
    hold off
    
    subplot(3, 2, 6)
    plot(tau4_his(3, :));
    hold on 
    plot(tauc4_his(3, :));
    hold off
    
    
    
    

end


%% 协同变量的误差(李博文章的算法）
function  [] = plot_coop_error(coop_error1_his, coop_error2_his, coop_error3_his, coop_error4_his, fig_id )
    figure(fig_id);
    title('协同变量误差');
    subplot(3, 1, 1)
%     title('协同变量误差');
    plot(coop_error1_his(1, :));
    hold on
    plot(coop_error2_his(1, :));
    hold on
    plot(coop_error3_his(1, :));
    hold on
    plot(coop_error4_his(1, :));
    hold off
    legend('ship1', 'sihp2', 'ship3', 'ship4');
    legend boxoff
    
    subplot(3, 1, 2)
    plot(coop_error1_his(2, :));
    hold on
    plot(coop_error2_his(2, :));
    hold on
    plot(coop_error3_his(2, :));
    hold on
    plot(coop_error4_his(2, :));
    hold off
    legend('ship1', 'sihp2', 'ship3', 'ship4');
    legend boxoff
    
    subplot(3, 1, 3)
    plot(coop_error1_his(3, :)/pi*180);
    hold on
    plot(coop_error2_his(3, :)/pi*180);
    hold on
    plot(coop_error3_his(3, :)/pi*180);
    hold on
    plot(coop_error4_his(3, :)/pi*180);
    hold off
    legend('ship1', 'sihp2', 'ship3', 'ship4');
    legend boxoff
end


    




