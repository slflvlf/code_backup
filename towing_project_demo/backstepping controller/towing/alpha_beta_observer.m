function [xk, vk] = alpha_beta_observer(a, b, xm, dt, xk, vk)

R = rotate_matrix(xm(3));
xm_b = R * xm;
xk_b = R * xk;
xk_b = xk_b + vk * dt;
rk = xm_b - xk_b;


xk_b = xk_b + a * rk;
vk = vk + b / dt * rk;

R = rotate_matrix(xm(3));
xk = R' * xk_b;


end

function R = rotate_matrix(fai)
    R = [cos(fai), -sin(fai), 0;
        sin(fai), cos(fai), 0;
        0, 0, 1];
end