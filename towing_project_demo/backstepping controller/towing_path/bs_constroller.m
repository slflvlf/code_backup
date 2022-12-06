function [tau, s, ei] = bs_constroller(pd, dot_pd, ddot_pd, p, v, M, D, Kd , Ki, ei)
% input:
% pd, dot_pd, ddot_pd: 轨迹
% p, v : 实时位置和速度

%output:
% tau: 控制力 /KN
% s： 中间变量（速度相关）
%     global ei MM1 D1 
%     ei = 0;
%     Kd = diag([20, 20, 5])*10;
%     Ki = diag([1, 1, 1])*1;
    Gama = diag([1, 1, 1])*0.7;
%     Gama = diag([1, 1, 1])*1;

    
    
    
    R = rotate_matrix(p);
    dot_R = dot_rotate_matrix(p, v);
    
    p_error = p - pd;
    dot_pr = dot_pd - Gama * p_error;
    ddot_pr = ddot_pd - Gama * (R * v - dot_pd);
    s = R * v -dot_pr;
    ei;
    ei = ei + p_error * 1; %误差累计，用于积分用
    
%     imax = 100;
%     for i = 1 : 3
%         if ei(i) > imax
%             ei(i) = imax;
%         end
%         if ei(i) < -imax
%             ei(i) = -imax;
%         end
%     end
        
    
    C1 = C_matrix(M, v);
    Mp1 = R * M * R';
    Dp1 = R * D * R';
    Cp1 = R * (C1 - M * R' * dot_R) * R';
    
    tau = R' * (Mp1 * ddot_pr + Cp1 * dot_pr + Dp1 * dot_pr - Kd * s - Ki * ei);
    
    tau = tau/1e3; %输出为KN
    

end






function R = rotate_matrix(p)
    fai = p(3);
    R = [cos(fai),  -sin(fai),  0;
         sin(fai),  cos(fai),   0;
         0,          0,         1];   
end

function dotR = dot_rotate_matrix(p, v)
    fai = p(3);
    r = v(3);
    dotR = [-sin(fai)*r, -cos(fai)*r, 0;
           cos(fai)*r, -sin(fai)*r, 0;
           0, 0, 0];
end
    
function C = C_matrix(M, v)
    C = [0, 0, -M(2, 2) * v(2) - M(2, 3) * v(3);
        0, 0, M(1, 1) * v(1);
        M(2, 2) * v(2) + M(3, 2) * v(3), -M(1, 1) * v(1), 0];
end