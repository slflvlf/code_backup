function [tau0_cable, tau_tug_cable, cable_length, cable_angle] = cable_force(p0, p1, p2, p3, p4, Kcable, cable_length0)
    global hull_x0 hull_y0 hull_x1 hull_y1;
    global delta10 delta20 delta30 delta40;
    
    hull_x0 = [-75, 75, 75, -75, -75];
    hull_y0 = [50, 50, -50, -50, 50];
    hull_x1 = [-16, 12, 16, 12, -16, -16];
    hull_y1 = [6, 6, 0, -6, -6, 6];
    
    deg2rad = pi/180;
    
    R0 = rotate_matrix(p0);
    
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
    
     %缆绳长度
    Lc1 = sqrt((pc1_start(1) - pc1_end(1)) ^ 2 + (pc1_start(2) - pc1_end(2)) ^ 2 );
    Lc2 = sqrt((pc2_start(1) - pc2_end(1)) ^ 2 + (pc2_start(2) - pc2_end(2)) ^ 2 );
    Lc3 = sqrt((pc3_start(1) - pc3_end(1)) ^ 2 + (pc3_start(2) - pc3_end(2)) ^ 2 );
    Lc4 = sqrt((pc4_start(1) - pc4_end(1)) ^ 2 + (pc4_start(2) - pc4_end(2)) ^ 2 );
    cable_length = [Lc1, Lc2, Lc3, Lc4]';
    
    % 缆绳张力
    if (Lc1 >= cable_length0(1))
        fc1 = Kcable * (Lc1 - cable_length0(1));
    else
        fc1 = 0;
    end
    
    if (Lc2 >= cable_length0(2))
        fc2 = Kcable * (Lc2 - cable_length0(2));
    else
        fc2 = 0;
    end
    
    if (Lc3 >= cable_length0(3))
        fc3 = Kcable * (Lc3 - cable_length0(3));
    else
        fc3 = 0;
    end
    
    if (Lc4 >= cable_length0(4))
        fc4 = Kcable * (Lc4 - cable_length0(4));
    else
        fc4 = 0;
    end
    
    %缆绳在母船随体坐标系的角度
    belta1 = atan2( (pc1_end(2) - pc1_start(2)), (pc1_end(1) - pc1_start(1)) );
    belta2 = atan2( (pc2_end(2) - pc2_start(2)), (pc2_end(1) - pc2_start(1)) );
    belta3 = atan2( (pc3_end(2) - pc3_start(2)), (pc3_end(1) - pc3_start(1)) );
    belta4 = atan2( (pc4_end(2) - pc4_start(2)), (pc4_end(1) - pc4_start(1)) );
    
    cable_angle = [belta1, belta2, belta3, belta4]';
    
    
    % 母船局部坐标系下， 缆绳作用于母船上的力和力矩
    tau10 = fc1 * [ cos(belta1),  sin(belta1), -hull_y0(4) * cos(belta1) + hull_x0(4) * sin(belta1) ]';
    tau20 = fc2 * [ cos(belta2),  sin(belta2), -hull_y0(3) * cos(belta2) + hull_x0(3) * sin(belta2) ]';
    tau30 = fc3 * [ cos(belta3),  sin(belta3), -hull_y0(2) * cos(belta3) + hull_x0(2) * sin(belta3) ]';
    tau40 = fc4 * [ cos(belta4),  sin(belta4), -hull_y0(1) * cos(belta4) + hull_x0(1) * sin(belta4) ]';
    tau0 = tau10 + tau20 + tau30 + tau40;
    
    tau0_cable = [tau10, tau20, tau30, tau40, tau0];
    
    % 缆绳在各个拖船随体坐标系的角度
    gama1 = p1(3) - p0(3) - belta1;  
    gama2 = p2(3) - p0(3) - belta2;  
    gama3 = p3(3) - p0(3) - belta3; 
    gama4 = p4(3) - p0(3) - belta4; 
    
    % 缆绳作用于拖轮上的力和力矩
    tau1_tug = [cos(pi - gama1), sin(pi - gama1), 0]';
    tau2_tug = [cos(pi - gama2), sin(pi - gama2), 0]';
    tau3_tug = [cos(pi - gama3), sin(pi - gama3), 0]';
    tau4_tug = [cos(pi - gama4), sin(pi - gama4), 0]';
    
    tau_tug_cable = [tau1_tug, tau2_tug, tau3_tug, tau4_tug];
    
    
    
    
end



%% 旋转矩阵
function R= rotate_matrix(p)
    fai = p(3);
    R = [cos(fai), -sin(fai), 0;
        sin(fai), cos(fai), 0;
        0, 0, 1];
end