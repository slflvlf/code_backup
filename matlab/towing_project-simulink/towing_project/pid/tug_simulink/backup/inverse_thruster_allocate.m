function  [f, a, tau_r] = inverse_thruster_allocate(L, tau)
% 拖轮推力分配程序， 伪逆法或广义逆方法(fx, fy)

% input parameters
% L: 推进器位置，维度N*2， tau: 待分配的控制力

 % output parameters
 % f: 推力大小； a: 推力方向； tau_r： 实际推力
 
 
 N = 3; %螺旋桨个数
 
 B = zeros(3, 2*N);
 B = [1, 0, 1, 0, 1, 0;
     0, 1, 0, 1, 0, 1;
     -L(1, 2), L(1, 1), -L(2, 2), L(2, 1), -L(3, 2), L(3, 1)];
 
 B_inv = B' * inv(B * B')
 
 x = B_inv * tau;
 
 fx1 = x(1);    fy1 = x(2);
 fx2 = x(3);    fy2 = x(4);
 fx3 = x(5);    fy3 = x(6);
 
 
 f1 = sqrt(fx1^2 + fy1^2);
 f2 = sqrt(fx2^2 + fy2^2);
 f3 = sqrt(fx3^2 + fy3^2);
 f = [f1, f2, f3]';
 
 a1 = atan2(fy1, fx1);
 a2 = atan2(fy2, fx2);
 a3 = atan2(fy3, fx3);
 a = [a1, a2, a3]';
 a = a / pi *180;
 
 tau_r = B * x;

end
 