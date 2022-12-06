function [tau0_cable, tau_tug_cable, cable_length, cable_angle] = cable_force1(p0, p1, p2, p3, p4, Kcable, cable_length0)
% 缆绳连接点在拖轮的首尾  
% 拖轮编号按照坐标系象限的顺序

global hull_x0 hull_y0 hull_x1 hull_y1;
    global delta10 delta20 delta30 delta40;
    
    hull_x0 = [-75, 75, 75, -75, -75];
    hull_y0 = [50, 50, -50, -50, 50];
    hull_x1 = [-16, 12, 16, 12, -16, -16];
    hull_y1 = [6, 6, 0, -6, -6, 6];

    deg2rad = pi/180;
    
    % 缆绳起始点坐标（全局坐标系下）
    pc1_start = local_to_global([hull_x0(4); hull_y0(4)], p0);
    pc1_end = local_to_global([(hull_x1(1)+hull_x1(5))/2; (hull_y1(1)+hull_y1(5))/2], p1);
    pc2_start = local_to_global([hull_x0(3); hull_y0(3)], p0);
    pc2_end = local_to_global([(hull_x1(1)+hull_x1(5))/2; (hull_y1(1)+hull_y1(5))/2], p2);
    pc3_start = local_to_global([hull_x0(2); hull_y0(2)], p0);
    pc3_end = local_to_global([(hull_x1(1)+hull_x1(5))/2; (hull_y1(1)+hull_y1(5))/2], p3);
    pc4_start = local_to_global([hull_x0(1); hull_y0(1)], p0);
    pc4_end = local_to_global([(hull_x1(1)+hull_x1(5))/2; (hull_y1(1)+hull_y1(5))/2], p4);
    
    % 缆绳长度
    Lc1 = sqrt((pc1_start(1) - pc1_end(1)) ^ 2 + (pc1_start(2) - pc1_end(2)) ^ 2 );
    Lc2 = sqrt((pc2_start(1) - pc2_end(1)) ^ 2 + (pc2_start(2) - pc2_end(2)) ^ 2 );
    Lc3 = sqrt((pc3_start(1) - pc3_end(1)) ^ 2 + (pc3_start(2) - pc3_end(2)) ^ 2 );
    Lc4 = sqrt((pc4_start(1) - pc4_end(1)) ^ 2 + (pc4_start(2) - pc4_end(2)) ^ 2 );
    
    cable_length = [Lc1, Lc2, Lc3, Lc4]';
    L10 = cable_length0(1);
    L20 = cable_length0(2);
    L30 = cable_length0(3);
    L40 = cable_length0(4);
    
    % 缆绳张力
    if (Lc1 >= L10)
        fc1 = Kcable * (Lc1 - L10);
    else
        fc1 = 0;
    end
    
    if (Lc2 >= L20)
        fc2 = Kcable * (Lc2 - L20);
    else
        fc2 = 0;
    end
    
    if (Lc3 >= L30)
        fc3 = Kcable * (Lc3 - L30);
    else
        fc3 = 0;
    end
    
    if (Lc4 >= L40)
        fc4 = Kcable * (Lc4 - L40);
    else
        fc4 = 0;
    end
    
    % 缆绳角度(全局坐标系）
    alpha1 = atan2( -(pc1_start(2) - pc1_end(2)), -(pc1_start(1) - pc1_end(1)) );
    alpha2 = atan2( -(pc2_start(2) - pc2_end(2)), -(pc2_start(1) - pc2_end(1)) );
    alpha3 = atan2( -(pc3_start(2) - pc3_end(2)), -(pc3_start(1) - pc3_end(1)) );
    alpha4 = atan2( -(pc4_start(2) - pc4_end(2)), -(pc4_start(1) - pc4_end(1)) );

    
    % 缆绳相对母船的角度（母船坐标系下）
    belta1 = alpha1 - p0(3);
    belta2 = alpha2 - p0(3);
    belta3 = alpha3 - p0(3);
    belta4 = alpha4 - p0(3);
    
    %相对于母船的缆绳角度，为了更好的和初始角度进行比较
    cable_angle = [belta1, belta2, belta3, belta4]'; 
    
    % 母船局部坐标系下
    tau10 = fc1 * [ cos(belta1),  sin(belta1), -hull_y0(4) * cos(belta1) + hull_x0(4) * sin(belta1) ]';
    tau20 = fc2 * [ cos(belta2),  sin(belta2), -hull_y0(3) * cos(belta2) + hull_x0(3) * sin(belta2) ]';
    tau30 = fc3 * [ cos(belta3),  sin(belta3), -hull_y0(2) * cos(belta3) + hull_x0(2) * sin(belta3) ]';
    tau40 = fc4 * [ cos(belta4),  sin(belta4), -hull_y0(1) * cos(belta4) + hull_x0(1) * sin(belta4) ]';
    tau0 = tau10 + tau20 + tau30 + tau40;
    
    tau0_cable = [tau10, tau20, tau30, tau40, tau0];
    
    
    % 缆绳力相对拖轮的角度（拖轮局部坐标系下）
    gama1 = pi - (p1(3) - alpha1);  
    gama2 = pi - (p2(3) - alpha2);  
    gama3 = pi - (p3(3) - alpha3);  
    gama4 = pi - (p4(3) - alpha4);  
    
    % 缆绳作用在拖轮上的力，拖轮局部坐标系下
    tau1 = fc1 * [cos(gama1), sin(gama1), (hull_x1(1)+hull_x1(5))/2 * sin(gama1) - (hull_y1(1)+hull_y1(5))/2 * cos(gama1)]';
    tau2 = fc2 * [cos(gama2), sin(gama2), (hull_x1(1)+hull_x1(5))/2 * sin(gama2) - (hull_y1(1)+hull_y1(5))/2 * cos(gama2)]';
    tau3 = fc3 * [cos(gama3), sin(gama3), (hull_x1(1)+hull_x1(5))/2 * sin(gama3) - (hull_y1(1)+hull_y1(5))/2 * cos(gama3)]';
    tau4 = fc4 * [cos(gama4), sin(gama4), (hull_x1(1)+hull_x1(5))/2 * sin(gama4) - (hull_y1(1)+hull_y1(5))/2 * cos(gama4)]';
    tau_tug_cable = [tau1, tau2, tau3, tau4];
end


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
