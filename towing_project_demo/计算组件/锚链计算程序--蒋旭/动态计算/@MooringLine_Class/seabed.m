function [Fb]=seabed(obj, N,Ms,Y_N,H,g)
Fb=zeros(3*N+3,1);
for i=1:1:N+1
    Fb(3*(i-1)+1,1)=0;
    Fb(3*(i-1)+2,1)=0;
    Fb(3*(i-1)+3,1)=Ms(i,1)*g*(1-tanh(Y_N(3*i,1)+H));
end
end