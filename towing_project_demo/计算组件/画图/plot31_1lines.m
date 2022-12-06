function plot31_1lines(pd_his,  fig_title, fig_legend, fig_id, rad_to_deg_flag)



fig = figure(fig_id);


subplot(3, 1, 1)

plot(pd_his(1, :)); 
legend(fig_legend{1});   legend boxoff;    title(fig_title);
subplot(3, 1, 2);
plot(pd_his(2, :));
subplot(3, 1, 3);
if rad_to_deg_flag
    plot(pd_his(3, :)/pi*180);
else
    plot(pd_his(3, :));
end


end



