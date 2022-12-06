function plot32_3lines(pd_his, p_his, p_hat_his, vd_his, v_his, v_hat_his, fig_title, fig_legend, fig_id, rad_to_deg_flag)

% 获取变量名称，并去掉_his(pd_his ----> pd)
legend11 = getVarName(pd_his);  legend12 = getVarName(p_his); legend13 = getVarName(p_hat_his);
legend21 = getVarName(vd_his);  legend22 = getVarName(v_his);   legend23 = getVarName(v_hat_his);

fig = figure(fig_id);


subplot(3, 2, 1)
plot(pd_his(1, :)); hold on; plot(p_his(1, :)); hold on; plot(p_hat_his(1, :)); hold off; 
legend(fig_legend{1}, fig_legend{2}, fig_legend{3});   title(fig_title);
subplot(3, 2, 3);
plot(pd_his(2, :)); hold on; plot(p_his(2, :)); hold on; plot(p_hat_his(2, :)); hold off;
subplot(3, 2, 5);
if rad_to_deg_flag
    plot(pd_his(3, :)/pi*180); hold on; plot(p_his(3, :)/pi*180); hold on; plot(p_hat_his(3, :)/pi*180); hold off;
else
    plot(pd_his(3, :)); hold on; plot(p_his(3, :)); hold on; plot(p_hat_his(3, :)); hold off;
end

subplot(3, 2, 2)
plot(vd_his(1, :)); hold on; plot(v_his(1, :)); hold on; plot(v_hat_his(1, :)); hold off; 
legend(fig_legend{4}, fig_legend{5}, fig_legend{6});
subplot(3, 2, 3);
plot(vd_his(2, :)); hold on; plot(v_his(2, :)); hold on; plot(v_hat_his(2, :)); hold off;
subplot(3, 2, 5);
if rad_to_deg_flag
    plot(vd_his(3, :)/pi*180); hold on; plot(v_his(3, :)/pi*180); hold on; plot(v_hat_his(3, :)/pi*180); hold off;
else
    plot(vd_his(3, :)); hold on; plot(v_his(3, :)); hold on; plot(v_hat_his(3, :)); hold off;
end
end



