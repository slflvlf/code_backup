function [taur, f, a] = Thrust_Allocation_inverse(tau)
% 违逆算法，进行推力分配，
% 优点是分配有解，缺点是没办法考虑推力约束和变化率等约束
L = [-5.02 0; 11.956 2.7; 11.956 -2.7];

B = [1, 0, 1, 0, 1, 0;
    0, 1, 0, 1, 0, 1
    -L(1, 2), L(1, 1), -L(2, 2), L(2, 1), -L(3, 2), L(3, 1)];

B_inv = B' * inv(B * B');

fmax = 800;

x = B_inv * tau;


fx1 = x(1); fy1 = x(2); f1 = sqrt(fx1^2 + fy1^2);   a1 = atan2(fy1, fx1);
fx2 = x(3); fy2 = x(4); f2 = sqrt(fx2^2 + fy2^2);   a2 = atan2(fy2, fx2);
fx3 = x(5); fy3 = x(6); f3 = sqrt(fx3^2 + fy3^2);   a3 = atan2(fy3, fx3);
f = [f1, f2, f3]';  a = [a1, a2, a3]';
taur = tau;

if (f1>fmax || f2>fmax || f3>fmax)
    disp('Thrust exceeds, cannot be allocated!');
end


end

