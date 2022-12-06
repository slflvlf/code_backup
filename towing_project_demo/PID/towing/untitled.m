
%% 锚链作用于各个船上的力
clear all; clc;
global hull_x0 hull_y0 hull_x1 hull_y1;
global delta10 delta20 delta30 delta40
%     Kc = 6e4;
    Kc = 1;
%     Kc = 6e4;
    L10 = 350;
    L20 = 350;
    L30 = 350;
    L40 = 350;
    deg2rad = pi/180;
    
    hull_x0 = [-75, 75, 75, -75, -75];
    hull_y0 = [50, 50, -50, -50, 50];
    hull_x1 = [-16, 12, 16, 12, -16, -16];
    hull_y1 = [6, 6, 0, -6, -6, 6];
    
    delta10 = [-394, -237, -149*deg2rad]';
    delta20 = [394, -237, -31*deg2rad]';
    delta30 = [394, 237, 31*deg2rad]';
    delta40 = [-394, 237, 149*deg2rad]';
    
%     p0 = state(1 : 3);
%     p1 = state(4 : 6);
%     p2 = state(10 : 12);
%     p3 = state(16 : 18);
%     p4 = state(22 : 24);
    
    p0 = [0, 0, 0*deg2rad]';
    p1 = p0 + delta10;
    p2 = p0 + delta20;
    p3 = p0 + delta30;
    p4 = p0 + delta40;
    

    
    
    % 缆绳起始点坐标（在母船随体坐标系） 
    R0 = rotate_matrix(p0);
    pc1_start = [hull_x0(4), hull_y0(4), 0]';
    pc1_end = R0' * ( p1 - p0);
    pc2_start = [hull_x0(3), hull_y0(3), 0]';
    pc2_end = R0' * ( p2 - p0);
    pc3_start = [hull_x0(2), hull_y0(2), 0]';
    pc3_end = R0' * ( p3 - p0);
    pc4_start = [hull_x0(1), hull_y0(1), 0]';
    pc4_end = R0' * ( p4 - p0);
    
    
    
    
%     % 缆绳起始点坐标（全局坐标系下）
%     pc1_start = local_to_global([x0(2); y0(2)], p0);
%     pc1_end = local_to_global([(x1(1)+x1(5))/2; (y1(1)+y1(5))/2], p1);
%     pc2_start = local_to_global([x0(3); y0(3)], p0);
%     pc2_end = local_to_global([(x2(1)+x2(5))/2; (y2(1)+y2(5))/2], p2);
%     pc3_start = local_to_global([x0(4); y0(4)], p0);
%     pc3_end = local_to_global([(x1(1)+x1(5))/2; (y1(1)+y1(5))/2], p3);
%     pc4_start = local_to_global([x0(5); y0(5)], p0);
%     pc4_end = local_to_global([(x2(1)+x2(5))/2; (y2(1)+y2(5))/2], p4);
    
    % 缆绳长度
    Lc1 = sqrt((pc1_start(1) - pc1_end(1)) ^ 2 + (pc1_start(2) - pc1_end(2)) ^ 2 );
    Lc2 = sqrt((pc2_start(1) - pc2_end(1)) ^ 2 + (pc2_start(2) - pc2_end(2)) ^ 2 );
    Lc3 = sqrt((pc3_start(1) - pc3_end(1)) ^ 2 + (pc3_start(2) - pc3_end(2)) ^ 2 );
    Lc4 = sqrt((pc4_start(1) - pc4_end(1)) ^ 2 + (pc4_start(2) - pc4_end(2)) ^ 2 );
    
    % 缆绳张力
    if (Lc1 >= L10)
        fc1 = Kc * (Lc1 - L10);
    else
        fc1 = 0;
    end
    
    if (Lc2 >= L20)
        fc2 = Kc * (Lc2 - L20);
    else
        fc2 = 0;
    end
    
    if (Lc3 >= L30)
        fc3 = Kc * (Lc3 - L30);
    else
        fc3 = 0;
    end
    
    if (Lc4 >= L40)
        fc4 = Kc * (Lc4 - L40);
    else
        fc4 = 0;
    end
    
    fc1
    fc2 
    fc3 
    fc4 
    
    %缆绳在母船随体坐标系的角度
    belta1 = atan2( (pc1_end(2) - pc1_start(2)), (pc1_end(1) - pc1_start(1)) );
    belta2 = atan2( (pc2_end(2) - pc2_start(2)), (pc2_end(1) - pc2_start(1)) );
    belta3 = atan2( (pc3_end(2) - pc3_start(2)), (pc3_end(1) - pc3_start(1)) );
    belta4 = atan2( (pc4_end(2) - pc4_start(2)), (pc4_end(1) - pc4_start(1)) );
    
    belta1/deg2rad
    belta2/deg2rad
    belta3/deg2rad
    belta4/deg2rad
    
    % 母船局部坐标系下
    tau10 = fc1 * [ cos(belta1),  sin(belta1), -hull_y0(4) * cos(belta1) + hull_x0(4) * sin(belta1) ]'
    tau20 = fc2 * [ cos(belta2),  sin(belta2), -hull_y0(3) * cos(belta2) + hull_x0(3) * sin(belta2) ]'
    tau30 = fc3 * [ cos(belta3),  sin(belta3), -hull_y0(2) * cos(belta3) + hull_x0(2) * sin(belta3) ]'
    tau40 = fc4 * [ cos(belta4),  sin(belta4), -hull_y0(1) * cos(belta4) + hull_x0(1) * sin(belta4) ]'
    tau0 = tau10 + tau20 + tau30 + tau40
    
    gama1 = p1(3) - p0(3) - belta1;  
    gama2 = p2(3) - p0(3) - belta2;  
    gama3 = p3(3) - p0(3) - belta3; 
    gama4 = p4(3) - p0(3) - belta4; 
    
    tau1_tug = [cos(pi - gama1), sin(pi - gama1), 0]'
    tau2_tug = [cos(pi - gama2), sin(pi - gama2), 0]'
    tau3_tug = [cos(pi - gama3), sin(pi - gama3), 0]'
    tau4_tug = [cos(pi - gama4), sin(pi - gama4), 0]'
    
    
%     % 缆绳角度(全局坐标系）
%     alpha1 = atan2( -(pc1_start(2) - pc1_end(2)), -(pc1_start(1) - pc1_end(1)) );
%     alpha2 = atan2( -(pc2_start(2) - pc2_end(2)), -(pc2_start(1) - pc2_end(1)) );
%     alpha3 = atan2( -(pc3_start(2) - pc3_end(2)), -(pc3_start(1) - pc3_end(1)) );
%     alpha4 = atan2( -(pc4_start(2) - pc4_end(2)), -(pc4_start(1) - pc4_end(1)) );
%     alpha1 * 180 / pi
%     alpha2 * 180 / pi
%     alpha3 * 180 / pi
%     alpha4 * 180 / pi
%     
%     % 缆绳相对母船的角度（母船坐标系下）
%     belta1 = alpha1 - p0(3);
%     belta2 = alpha2 - p0(3);
%     belta3 = alpha3 - p0(3);
%     belta4 = alpha4 - p0(3);
%     
%     % 将角度转化为（-pi, pi)
%     belta1 = transfer_deg(belta1)
%     belta2 = transfer_deg(belta2);
%     belta3 = transfer_deg(belta3);
%     belta4 = transfer_deg(belta4);
%     belta1 * 180 / pi   
%     belta2 * 180 / pi
%     belta3 * 180 / pi
%     belta4 * 180 / pi
%     
%     % 母船局部坐标系下
%     tau10 = fc1 * [ cos(belta1),  sin(belta1), -y0(2) * cos(belta1) + x0(2) * sin(belta1) ]'
%     tau20 = fc2 * [ cos(belta2),  sin(belta2), -y0(3) * cos(belta2) + x0(3) * sin(belta2) ]'
%     tau30 = fc3 * [ cos(belta3),  sin(belta3), -y0(4) * cos(belta3) + x0(4) * sin(belta3) ]'
%     tau40 = fc4 * [ cos(belta4),  sin(belta4), -y0(5) * cos(belta4) + x0(5) * sin(belta4) ]'
%     tau0 = tau10 + tau20 + tau30 + tau40;
%     
%     
%     % 缆绳力相对拖轮的角度（拖轮局部坐标系下）
%     gama1 = pi - (p1(3) - alpha1);  gama1 = transfer_deg(gama1);
%     gama2 = pi - (p2(3) - alpha2);  gama2 = transfer_deg(gama2);
%     gama3 = pi - (p3(3) - alpha3);  gama3 = transfer_deg(gama3);
%     gama4 = pi - (p4(3) - alpha4);  gama4 = transfer_deg(gama4);
%     gama1 / pi * 180
%     gama2 / pi * 180
%     gama3 / pi * 180
%     gama4 / pi * 180
%     
%     % 缆绳作用在拖轮上的力，拖轮局部坐标系下
%     tau1 = fc1 * [cos(gama1), sin(gama1), (x1(1)+x1(5))/2 * sin(gama1) - (y1(1)+y1(5))/2 * cos(gama1)]'
%     tau2 = fc2 * [cos(gama2), sin(gama2), (x1(1)+x1(5))/2 * sin(gama2) - (y1(1)+y1(5))/2 * cos(gama2)]'
%     tau3 = fc3 * [cos(gama3), sin(gama3), (x1(1)+x1(5))/2 * sin(gama3) - (y1(1)+y1(5))/2 * cos(gama3)]'
%     tau4 = fc4 * [cos(gama4), sin(gama4), (x1(1)+x1(5))/2 * sin(gama4) - (y1(1)+y1(5))/2 * cos(gama4)]'
% end


%% 寻找锚链连接点(大地坐标系下）
function pc_g = local_to_global(pc_l, p0)
    fai = p0(3);
    R = [cos(fai), -sin(fai);
        sin(fai), cos(fai)];
    pc_g = R * pc_l + p0(1:2);
    
end
%% 旋转矩阵
function R = rotate_matrix(p)

    fai = p(3);
    
    R = [cos(fai), -sin(fai), 0;
        sin(fai), cos(fai), 0;
        0, 0, 1];
end


%% 将角度转化为（-pi, pi)
function y = transfer_deg(x)
    y = -sign(x) * pi + rem((x + sign(x) * pi), 2 * pi);
end
