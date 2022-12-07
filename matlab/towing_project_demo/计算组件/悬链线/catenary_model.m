% 悬链线理论
clear all; clc;
L=300;
d=0.09;
A=6.362e-3;
w=126;
E=9.16e10;
l_his = 290:0.2:320;
T_init = 1e4;
tic
for i = 1 : length(l_his)
    i
    l = l_his(i);
    eq = @(T)2*T/w*asinh(w*L/2/T)+T*L/E/A-l;
    T = fsolve(eq, T_init);
    T_init = T;
    T_his(i) = T/1e3;
    
    
    % 阻力计算
    s = l;
    v=2;
    Rt = 1.224e0*s*d/10*v^2*(1+1.122e0/T*d/10*(s/1e0)^2);
    Rt_his(i) = Rt;
end
plot(l_his, T_his);
hold on 
plot(l_his, Rt_his+T_his);
hold off


fsolve_time = toc





