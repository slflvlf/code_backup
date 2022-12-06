function plot1_3lines(x_his, fig_title, fig_legend, fig_id)

    figure(fig_id);
    title(fig_title);

    plot(x_his(1,:)); hold on; plot(x_his(2,:)); hold on;  plot(x_his(3, :)); hold off;
    legend(fig_legend{1}, fig_legend{2}, fig_legend{3}); legend boxoff;

end

