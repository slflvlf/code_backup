p0 = [0; 0; 0];



delta10 = [1, 1, 0]';
delta20 = [-1, 1, 0]';
delta30 = [-1, -1, 0]';
delta40 = [1, -1, 0]';



p0 = [1, 1, deg2rad(30)]';

[p1, p2, p3, p4] = transform_function(p0, delta10, delta20, delta30, delta40)


p_mean = (p1 + p2 + p3 + p4)/4

p10 = p1 - p0
p20 = p2 - p0
p30 = p3 - p0
p40 = p4 - p0






%% 模块测试（nsb_controller)
clear all; clc;
p0 = [0; 0; 0];

global delta10 delta20 delta30 delta40;

delta10 = [1, 1]';
delta20 = [-1, 1]';
delta30 = [-1, -1]';
delta40 = [1, -1]';


p1 = [delta10; 0] + p0;
p2 = [delta20; 0] + p0;
p3 = [delta30; 0] + p0;
p4 = [delta40; 0] + p0;

p1 = [0.5 0.5, 0]';
p2 = [-1, 0.5, 0]';

p1_his = p1';
p2_his = p2';
p3_his = p3';
p4_his = p4';
% p1_his = [p1_his; p1'];
% p2_his = [p2_his; p2'];
% p3_his = [p3_his; p3'];
% p4_his = [p4_his; p4'];

destination = [1, 1, 0]'
T = 50;
pd0 = p0;
for i = 1 : 100
    
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
    
    pause(0.15);

    [pd0, dot_pd0, ddot_pd] = ref_model(T, i, p0, destination);
    
%     pd0 = [1, 1, 0]';
%     dot_pd0 = [0, 0, 0]';
    [U, angle_course U_barycenter, U_formation, U_obstacle] = nsb_controller(p1, p2, p3, p4, pd0, dot_pd0);
    p1 = p1 + [U(1); U(2); 0];
    p2 = p2 + [U(3); U(4); 0];
    p3 = p3 + [U(5); U(6); 0];
    p4 = p4 + [U(7); U(8); 0];
    p1_his = [p1_his; p1'];
    p2_his = [p2_his; p2'];
    p3_his = [p3_his; p3'];
    p4_his = [p4_his; p4'];

    


end







%%
sigma_formation = [0.8750 0.8750 -1.1250 0.8750 -0.8750 -0.8750 1.1250 -0.8750]';
sigma_formation_desired = [delta10; delta20; delta30; delta40];

n_ship = 4;
%中间矩阵
    A = zeros(n_ship, n_ship);
    for i = 1 : n_ship
        for j = 1 : n_ship
            if(i==j)
                A(i, j) = 1-1/n_ship;
            else
                A(i, j) = -1/n_ship;
            end
        end
    end
    
    %任务函数的雅可比矩阵
    J_formation = [A, zeros(n_ship, n_ship);
                    zeros(n_ship, n_ship), A]
    error = sigma_formation_desired - sigma_formation
U_formation = J_formation * (sigma_formation_desired - sigma_formation)
 

%% 辅助函数

function [p1, p2, p3, p4] = transform_function(p0, delta10, delta20, delta30, delta40)
    R = rotate_matrix(p0(3));

    p1 = p0 + R * delta10;
    p2 = p0 + R * delta20;
    p3 = p0 + R * delta30;
    p4 = p0 + R * delta40;

end


function R = rotate_matrix(fai)
    
    R = [cos(fai), -sin(fai), 0;
        sin(fai), cos(fai), 0;
        0, 0, 1];
end

