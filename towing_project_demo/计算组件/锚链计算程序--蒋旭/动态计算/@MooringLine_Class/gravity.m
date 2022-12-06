function [G]=gravity(obj, N,Ms,g)
G=zeros(3*N+3,1);
for i=1:1:N+1
   G(3*(i-1)+1,1)=0; 
   G(3*(i-1)+2,1)=0;
   G(3*(i-1)+3,1)=-Ms(i,1)*g;
end
end