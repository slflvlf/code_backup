function plot1_2lines(x_his, fig_title, fig_legend, fig_id)

    figure(fig_id);
    title(fig_title);

    plot(x_his(1,:)); hold on; plot(x_his(2,:)); hold off;
    legend(fig_legend{1}, fig_legend{2}); legend boxoff;

end

