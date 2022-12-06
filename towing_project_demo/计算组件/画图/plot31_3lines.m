function plot31_3lines(pd_his, p_his, p_hat_his, fig_title, fig_legend, fig_id, rad_to_deg_flag)

% rad_to_deg_flag: 决定是否转换角度

fig = figure(fig_id);


subplot(3, 1, 1)

plot(pd_his(1, :)); hold on; plot(p_his(1, :)); hold on; plot(p_hat_his(1, :)); hold off; 
legend(fig_legend{1}, fig_legend{2}, fig_legend{3});    title(fig_title);
subplot(3, 1, 2);
plot(pd_his(2, :)); hold on; plot(p_his(2, :)); hold on; plot(p_hat_his(2, :)); hold off;
subplot(3, 1, 3);
if rad_to_deg_flag
    plot(pd_his(3, :)/pi*180); hold on; plot(p_his(3, :)/pi*180); hold on; plot(p_hat_his(3, :)/pi*180); hold off;
else
    plot(pd_his(3, :)); hold on; plot(p_his(3, :)); hold on; plot(p_hat_his(3, :)); hold off;
end

end



