figure(1)
subplot(3, 2, 1)
plot(xx(:, 1));
hold on
plot(rrd(:, 1))
hold off
legend("p", "pd")
legend boxoff
subplot(3, 2, 3)
plot(xx(:, 2));
hold on
plot(rrd(:, 2))
hold off

subplot(3, 2, 5)
plot(xx(:, 3)/pi*180);
hold on
plot(rrd(:, 3)/pi*180)
hold off
subplot(3, 2, 2)
plot(xx(:, 4))
subplot(3, 2, 4)
plot(xx(:, 5))
subplot(3, 2, 6)
plot(xx(:, 6)/pi*180);

figure(2)
subplot(2, 2, 1)
plot(uu(:, 1));
hold on
plot(uu(:, 2));
hold on
plot(uu(:, 3));
hold off
legend("f_1", "f_2", "f_3");
legend boxoff

subplot(2, 2, 3)
plot(uu(:, 4)/pi*180);
hold on
plot(uu(:, 5)/pi*180);
hold on
plot(uu(:, 6)/pi*180);
hold off
legend("a_1", "a_2", "a_3");
legend boxoff

subplot(2, 2, 2)
plot(du(:, 1));
hold on
plot(du(:, 2));
hold on
plot(du(:, 3));
hold off
legend("df_1", "df_2", "df_3");
legend boxoff

subplot(2, 2, 4)
plot(du(:, 4)/pi*180);
hold on
plot(du(:, 5)/pi*180);
hold on
plot(du(:, 6)/pi*180);
hold off
legend("da_1", "da_2", "da_3");
legend boxoff


% nl=100;
% [ttc,xxc]=compacty(tt,xx,nl);
% [~,uuc]=compacty(tt,uu,nl);
% [~,rrdc]=compacty(tt,rrd,nl);
% [~,tt_execc]=compacty(tt,tt_exec,nl);
% 
% figure(1);clf;
% h(1)=subplot(321);plot(ttc,rrdc,'b-.',ttc,xxc(:,1),'k-','linewidth',2);grid on;
% title('$r$ and $r^d$','fontsize',40,'interpreter','latex');
% 
% h(2)=subplot(322);plot(ttc,ones(size(ttc))*[-param.ocp.theta_max param.ocp.theta_max]*180/pi,'r-.',...
%     ttc,180/pi*xxc(:,3),'k-','linewidth',2);grid on;
% title('$\theta$ and its bounds','fontsize',40,'interpreter','latex');
% 
% h(3)=subplot(323);plot(ttc,ones(size(ttc))*[-param.ocp.thetap_max param.ocp.thetap_max],'r-.',...
%     ttc,180/pi*xxc(:,4)','k','linewidth',2);grid on;
% 
% title('$\dot\theta$ and its bounds','fontsize',40,'interpreter','latex');
% xlabel('time (sec)','fontsize',40,'interpreter','latex');
% 
% h(4)=subplot(324);plot(ttc,ones(size(ttc))*[param.pmin(1) param.pmax(1)],'r-.','linewidth',2); hold on;
% [tts,uucs]=stairs(ttc,uuc);grid on;
% plot(tts,uucs,'k-','linewidth',2);
% title('$u$ and its bounds','fontsize',40,'interpreter','latex');
% xlabel('time (sec)','fontsize',40,'interpreter','latex');
% 
% for i=1:4,
%     subplot(3,2,i);set(gca,'fontsize',34);
% end
% set(gcf,'color','white');
% 
% h(5)=subplot(313);plot(tt(2:end-1),1000*tt_exec(2:end-1),'k','linewidth',2);
% title('CPU-time (ms)','fontsize',40,'interpreter','latex');
% xlabel('time (sec)','fontsize',40,'interpreter','latex');
% grid on;set(gca,'fontsize',34);
% 
% linkaxes(h,'x')