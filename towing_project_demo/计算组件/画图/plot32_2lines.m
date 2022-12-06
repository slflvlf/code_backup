function plot32_2lines(pd_his, p_his,  vd_his, v_his, fig_title, fig_legend, fig_id, rad2deg_flag)

% % 获取变量名称，并去掉_his(pd_his ----> pd) 这个想法没用
% legend11 = getVarName(pd_his);  legend12 = getVarName(p_his); 
% legend21 = getVarName(vd_his);  legend22 = getVarName(v_his);   

fig = figure(fig_id);

title(fig_title);
subplot(3, 2, 1)
plot(pd_his(1, :)); hold on; plot(p_his(1, :));  hold off; 
legend(fig_legend{1}, fig_legend{2});  title(fig_title); legend boxoff;
subplot(3, 2, 3);
plot(pd_his(2, :)); hold on; plot(p_his(2, :)); hold off;
subplot(3, 2, 5);
if rad2deg_flag
    plot(pd_his(3, :)/pi*180); hold on; plot(p_his(3, :)/pi*180);  hold off;
else
    plot(pd_his(3, :)); hold on; plot(p_his(3, :));  hold off;
end

subplot(3, 2, 2)
plot(vd_his(1, :)); hold on; plot(v_his(1, :)); hold off; legend(fig_legend{3}, fig_legend{4}); legend boxoff;
subplot(3, 2, 4);
plot(vd_his(2, :)); hold on; plot(v_his(2, :));  hold off;
subplot(3, 2, 6);
if rad2deg_flag
    plot(vd_his(3, :)/pi*180); hold on; plot(v_his(3, :)/pi*180);  hold off;
else
    plot(vd_his(3, :)); hold on; plot(v_his(3, :));  hold off;
end



