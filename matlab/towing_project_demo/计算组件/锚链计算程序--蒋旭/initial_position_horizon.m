function [Y_N, p_init0]=initial_position_horizon(p0,p1,H,N)
% function [Y_N]=initial_position(p0,p1,H,N)
X0=p0(1,1);
Y0=p0(2,1);
Z0=p0(3,1);
X1=p1(1,1);
Y1=p1(2,1);
Z1=p1(3,1);
h=(Z0-Z1);
L=sqrt((X1-X0)^2+(Y1-Y0)^2);
% 按照z = ax^2+bx+c 初始化位置
if Z0 > Z1
    a = -3*(Z1 - Z0)/L^2;
    b = -4*a*L/3;
    c = Z0;
elseif Z0<Z1
    a = 3*(Z1 - Z0)/L^2;
    b = -2*a*L/3;
    c = Z0;
else
    min = Z0 - L/400;
    a = -4*(min-Z0)/L^2;
    b = -a*L;
    c = Z0;
end

Y_N=zeros(3*N+3,1);
p_init0 = zeros(N+1, 3);
for i=1:1:N+1
   Y_N(3*(i-1)+1,1)=X0+(X1-X0)/N*(i-1);  % X×ø±ê
   Y_N(3*(i-1)+2,1)=Y0+(Y1-Y0)/N*(i-1);  % Y×ø±ê
   l_m=sqrt((Y_N(3*(i-1)+1,1)-X0)^2+(Y_N(3*(i-1)+2,1)-Y0)^2);
   Y_N(3*(i-1)+3,1)=a*l_m^2+b*l_m+c; % Z×ø±ê
   
   p_init0(i, 1) = Y_N(3*(i-1)+1,1);
   p_init0(i, 2) = Y_N(3*(i-1)+2,1);
   p_init0(i, 3) = Y_N(3*(i-1)+3,1);
end


end

