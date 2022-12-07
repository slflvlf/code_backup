% function [Y_N, p_init0]=initial_position(p0,p1,H,N)
function [Y_N]=initial_position(p0,p1,H,N)
X0=p0(1,1);
Y0=p0(2,1);
Z0=p0(3,1);
X1=p1(1,1);
Y1=p1(2,1);
Z1=p1(3,1);
h=Z0-Z1;
Y_N=zeros(3*N+3,1);
L_0=sqrt((X1-X0)^2+(Y1-Y0)^2);
a=2*h/L_0^2;
b=h/L_0;
p_init0 = zeros(N+1, 3);
for i=1:1:N+1
   Y_N(3*(i-1)+1,1)=X0+(X1-X0)/N*(i-1);  % X×ø±ê
   Y_N(3*(i-1)+2,1)=Y0+(Y1-Y0)/N*(i-1);  % Y×ø±ê
   l_m=sqrt((Y_N(3*(i-1)+1,1)-X0)^2+(Y_N(3*(i-1)+2,1)-Y0)^2);
   Y_N(3*(i-1)+3,1)=(l_m-L_0)*(a*l_m-b)-H; % Z×ø±ê
   
%    p_init0(i, 1) = Y_N(3*(i-1)+1,1);
%    p_init0(i, 2) = Y_N(3*(i-1)+2,1);
%    p_init0(i, 3) = Y_N(3*(i-1)+3,1);
end


end