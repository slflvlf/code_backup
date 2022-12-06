function plot_thrust3(tau_his, taur_his, f_his, a_his, fig_title, fig_id)

    figure(fig_id)
title(fig_title);

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
plot(a_his(1,:)/pi*180);

subplot(3, 3, 6)
plot(a_his(2,:)/pi*180);

subplot(3, 3, 9)
plot(a_his(3,:)/pi*180);

end


