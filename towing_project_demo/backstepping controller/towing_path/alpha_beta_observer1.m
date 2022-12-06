function [xk, dot_xk] = alpha_beta_observer1(a, b, xm, dt, xk, dot_xk)
% 不存在坐标转换的ab滤波

xk = xk + dot_xk * dt;
rk = xm - xk;
xk = xk + a * rk;
dot_xk = dot_xk + b / dt * rk;


% R = rotate_matrix(xm(3));
% xm_b = R * xm;
% xk_b = R * xk;
% xk_b = xk_b + vk * dt;
% rk = xm_b - xk_b;
% 
% 
% xk_b = xk_b + a * rk;
% vk = vk + b / dt * rk;
% 
% R = rotate_matrix(xm(3));
% xk = R' * xk_b;


end

function R = rotate_matrix(fai)
    R = [cos(fai), -sin(fai), 0;
        sin(fai), cos(fai), 0;
        0, 0, 1];
end