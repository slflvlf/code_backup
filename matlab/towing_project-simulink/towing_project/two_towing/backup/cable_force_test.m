% function [tau1, tau2] = cable_force(state)

%% 锚链
    clc
%% 参数数值
   Kc = 10;
%    p0 = [0, 0, 0]';
%    p1 = [100, 100, 45]';
%    p2 = [100, -100, -45]';
   L10 = 300;
   L20 = 300;
   
   p0 = [0, 0, 10]';
   p1 = [397, 237, 31]';
   p2 = [397, -237, -31]';


%%

    x0 = [-75, 75, 75, -75, -75];
    y0 = [50, 50, -50, -50, 50];
    x1 = [-16, 12, 16, 12, -16, -16];
    y1 = [6, 6, 0, -6, -6, 6];
    x2 = [-16, 12, 16, 12, -16, -16];
    y2 = [6, 6, 0, -6, -6, 6];
    
%     p0 = state(1 : 3);
%     p1 = state(7 : 9);
%     p2 = state(13 : 15);
    
    % 角度转化成弧度
    p0(3) = p0(3) * pi / 180;
    p1(3) = p1(3) * pi / 180;
    p2(3) = p2(3) * pi / 180;
    
    % 缆绳起始点坐标（全局坐标系下）
    pc1_start = local_to_global([x0(2); y0(2)], p0);
    pc1_end = local_to_global([(x1(1)+x1(5))/2; (y1(1)+y1(5))/2], p1);
    pc2_start = local_to_global([x0(3); y0(3)], p0);
    pc2_end = local_to_global([(x2(1)+x2(5))/2; (y2(1)+y2(5))/2], p2);
    
    
    % 缆绳长度
    Lc1 = sqrt((pc1_start(1) - pc1_end(1)) ^ 2 + (pc1_start(2) - pc1_end(2)) ^ 2 )
    Lc2 = sqrt((pc2_start(1) - pc2_end(1)) ^ 2 + (pc2_start(2) - pc2_end(2)) ^ 2 )
    
    % 缆绳张力
    fc1 = Kc * (Lc1 - L10)
    fc2 = Kc * (Lc2 - L20)
    
    
    % 缆绳角度
    alpha1 = atan2( -(pc1_start(2) - pc1_end(2)), -(pc1_start(1) - pc1_end(1)) );
    alpha2 = atan2( -(pc2_start(2) - pc2_end(2)), -(pc2_start(1) - pc2_end(1)) );
    alpha1*180/pi
    alpha2*180/pi
    % 缆绳相对母船的角度（母船坐标系下）
    belta1 = alpha1 - p0(3);
    belta2 = alpha2 - p0(3);
    
    % 将角度转化为（-pi, pi)
    belta1 = transfer_deg(belta1);
    belta2 = transfer_deg(belta2);
    belta1/pi*180
    belta2/pi*180
    
    tau1 = fc1 * [ cos(belta1),  sin(belta1), y0(2) * sin(belta1) - x0(2) * cos(belta1) ]'
    tau2 = fc2 * [ cos(belta2),  sin(belta2), y0(3) * sin(belta2) - x0(3) * cos(belta2) ]'



%% 寻找锚链连接点(大地坐标系下）
function pc_g = local_to_global(pc_l, p0)
    fai = p0(3);
    R = [cos(fai), -sin(fai);
        sin(fai), cos(fai)];
    pc_g = R * pc_l + p0(1:2);
    
end

%% 旋转矩阵
function [R2, R3] = rotate_matrix(p)

    fai = p(3);
    R2  = [cos(fai), -sin(fai);
            sin(fai), cos(fai)];
    
    R3 = [cos(fai), -sin(fai), 0;
        sin(fai), cos(fai), 0;
        0, 0, 1];
end

%% 将角度转化为（-pi, pi)
function y = transfer_deg(x)
    y = -sign(x) * pi + rem((x + sign(x) * pi), 2 * pi);
end
