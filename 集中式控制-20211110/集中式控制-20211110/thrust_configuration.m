function T_conf=thrust_configuration(u_e)
% This function is used to create thrust configuration matrix
% Input
% u_e       k时刻的seed控制输入

% Output
% T_conf         thrust configuration matrix

global thrConf
global N

% %test
% thrConf=[0.45 0;0 -0.225;-0.45 0;0 0.225];
% N=4;
% u_e=[2;2;2;2;0;0;0;0];
% %end test

alpha=u_e(N+1:end);%角度单位是rad
T_conf=zeros(3,N);%预分配内存

for i=1:N
    T_conf(:,i)=[cos(alpha(i));sin(alpha(i));thrConf(i,1)*sin(alpha(i))-thrConf(i,2)*cos(alpha(i))];
end

end