

plotMotion(x1_his, x1_hat_his, pd1_his, 2)

function [] = plotMotion(x0_his, x0_hat_his, pd0_his, fig_id)
figure(fig_id)
subplot(3, 2, 1); plot(x0_his(:, 1)); hold on; plot(x0_hat_his(:, 1)); hold on; plot(pd0_his(:, 1)); hold off; legend('p0', 'p0_{hat}', 'pd'); legend boxoff
subplot(3, 2, 3); plot(x0_his(:, 2)); hold on; plot(x0_hat_his(:, 2)); hold on; plot(pd0_his(:, 2)); hold off;
subplot(3, 2, 5); plot(x0_his(:, 3)/pi*180); hold on; plot(x0_hat_his(:, 3)/pi*180); hold on; plot(pd0_his(:, 3)/pi*180); hold off;
subplot(3, 2, 2); plot(x0_his(:, 4)); hold on; plot(x0_hat_his(:, 4)); hold off; legend('v0', 'v0_{hat}'); legend boxoff
subplot(3, 2, 4); plot(x0_his(:, 5)); hold on; plot(x0_hat_his(:, 5)); hold off;
subplot(3, 2, 6); plot(x0_his(:, 6)/pi*180); hold on; plot(x0_hat_his(:, 6)/pi*180); hold off;
end