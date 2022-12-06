% 画图

%% position and velocity
figure(1)
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

%%  推力分配
%一列是总的，一列是f，一列是a
figure(2)
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


% figure(10)
% for i = 1 : 15
%     subplot(3, 5, i)
%     plot(sin(0:0.1:2*pi));
% end







