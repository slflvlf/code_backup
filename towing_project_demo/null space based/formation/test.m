%% Local_DP 类的测试

clear all; clc; clf;
% global ei M1 Ma1 MM1 D1 integ_var ;
M1 = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
Ma1 = [7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5, 8.77e6];
MM1 = M1 + Ma1;
D1 = [MM1(1,1)/100, 0, 0; 0, MM1(2,2)/40, 0; 0, 0, MM1(3,3)/20];

p0 = [0; 0; 0];

global delta10 delta20 delta30 delta40;

delta10 = [1, 1]';
delta20 = [-1, 1]';
delta30 = [-1, -1]';
delta40 = [1, -1]';


p1 = [delta10; pi/4] + p0;
p2 = [delta20; pi/4] + p0;
p3 = [delta30; pi/4] + p0;
p4 = [delta40; pi/4] + p0;
[v1, v2, v3, v4] = deal(zeros(3, 1));

% p1 = [1 0.5, pi/4]';
% p2 = [-1, 0.5, pi/4]';
step_time = 1;

ship1 = Local_DP(MM1, D1, p1, v1, step_time);
ship2 = Local_DP(MM1, D1, p2, v2, step_time);
ship3 = Local_DP(MM1, D1, p3, v3, step_time);
ship4 = Local_DP(MM1, D1, p4, v4, step_time);



% p1_his = [p1_his; p1'];
% p2_his = [p2_his; p2'];
% p3_his = [p3_his; p3'];
% p4_his = [p4_his; p4'];

destination = [1, 1, 0]';
T = 50;
pd0 = p0;

pd0_his = pd0;
for i = 1 : 30

    
    [pd0, dot_pd0, ddot_pd] = ref_model(T, i, p0, destination);

    pd0_his = [pd0_his, pd0];
    
    [U, angle_course, U_barycenter, U_formation, U_obstacle] = nsb_controller(p1, p2, p3, p4, pd0, dot_pd0);
    Un1 = sqrt(U(1)^2 + U(2)^2);    Un2 = sqrt(U(3)^2 + U(4)^2);
    Un3 = sqrt(U(5)^2 + U(6)^2);    Un4 = sqrt(U(7)^2 + U(8)^2);
    ship1 = ship1.set_ipunt(Un1, angle_course(1), p1, v1);
    ship2 = ship2.set_ipunt(Un2, angle_course(2), p2, v2);
    ship3 = ship3.set_ipunt(Un3, angle_course(3), p3, v3);
    ship4 = ship4.set_ipunt(Un4, angle_course(4), p4, v4);

%     ship1.dp_controller();
    ship1 = ship1.one_step_simulation();
    ship2 = ship2.one_step_simulation();
    ship3 = ship3.one_step_simulation();
    ship4 = ship4.one_step_simulation();



    [tau1, p1, v1] = ship1.get_output();
    [tau2, p2, v2] = ship2.get_output();
    [tau3, p3, v3] = ship3.get_output();
    [tau4, p4, v4] = ship4.get_output();


    plot(pd0(1), pd0(2), '*');
    hold on
    plot(p1(1), p1(2), '*')
    hold on
    plot(p2(1), p2(2), '*')
    hold on
    plot(p3(1), p3(2), '*')
    hold on
    plot(p4(1), p4(2), '*')
    hold off
    
    pause(0.4);

end

plot(pd0_his(1, :))


% [tau1_his, p1_his, v1_his] = ship1.get_history();
% [tau2_his, p2_his, v2_his] = ship2.get_history();
% [tau3_his, p3_his, v3_his] = ship3.get_history();
% [tau4_his, p4_his, v4_his] = ship4.get_history();







%% module test 控制器测试

clear all; clc; clf;
tic;
% global ei M1 Ma1 MM1 D1 integ_var ;
M1 = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
Ma1 = [7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5, 8.77e6];
MM1 = M1 + Ma1;
D1 = [MM1(1,1)/100, 0, 0; 0, MM1(2,2)/40, 0; 0, 0, MM1(3,3)/20];


p1 = [0, 0, 0]';
v1 = p1;

step_time = 1;

ship1 = Local_DP(MM1, D1, p1, v1, step_time);

destination = [1, 1, 0]';
T = 50;
pd0 = p1;
p0 = p1;

pd0_his = pd0;
p1_his = p1;


k1 = 100;
k2 = eye(3) * 100;

ship1 = ship1.set_gain(k1, k2);
ship1 = ship1.set_filter_gain(0.1, 0.001);

for i = 1 : 30
    [pd0, dot_pd0, ddot_pd] = ref_model(T, i, p0, destination);

    pd0_his = [pd0_his, pd0];
   

    U = norm(dot_pd0(1:2));

    angle = atan2(dot_pd0(2), dot_pd0(1));

    ship1 = ship1.set_ipunt(U, angle, p1, v1);

    ship1 = ship1.one_step_simulation();

    [tau1, p1, v1] = ship1.get_output();

    p1_his = [p1_his, p1];

end
toc

figure(1)
subplot(3, 1, 1); plot(p1_his(1, :)); hold on; plot(pd0_his(1, :)); hold off; legend('p', 'pd');
subplot(3, 1, 2); plot(p1_his(2, :)); hold on; plot(pd0_his(2, :)); hold off; legend('p', 'pd');
subplot(3, 1, 3); plot(p1_his(3, :)); hold on; plot(pd0_his(3, :)); hold off; legend('p', 'pd');


% plot(pd0_his(1,:))


