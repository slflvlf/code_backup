function plot32_1lines(pd_his,  vd_his, fig_title, fig_legend, fig_id, rad2deg_flag)

% % 获取变量名称，并去掉_his(pd_his ----> pd) 这个想法没用
% legend11 = getVarName(pd_his);  legend12 = getVarName(p_his); 
% legend21 = getVarName(vd_his);  legend22 = getVarName(v_his);   

fig = figure(fig_id);

title(fig_title);
subplot(3, 2, 1)
plot(pd_his(1, :));; 
legend(fig_legend{1});  title(fig_title); legend boxoff;
subplot(3, 2, 3);
plot(pd_his(2, :));;
subplot(3, 2, 5);
if rad2deg_flag
    plot(pd_his(3, :)/pi*180);
else
    plot(pd_his(3, :));
end

subplot(3, 2, 2)
plot(vd_his(1, :)); legend(fig_legend{2}); legend boxoff;
subplot(3, 2, 4);
plot(vd_his(2, :));
subplot(3, 2, 6);
if rad2deg_flag
    plot(vd_his(3, :)/pi*180);
else
    plot(vd_his(3, :));
end