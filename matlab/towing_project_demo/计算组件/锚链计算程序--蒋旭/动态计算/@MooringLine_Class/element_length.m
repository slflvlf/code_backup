function [l]=element_length(Y_N,N)
l=zeros(N,1);
for i=1:1:N
   l(i,1)=sqrt((Y_N(3*i+1,1)-Y_N(3*(i-1)+1,1))^2+...
               (Y_N(3*i+2,1)-Y_N(3*(i-1)+2,1))^2+...
               (Y_N(3*i+3,1)-Y_N(3*(i-1)+3,1))^2); 
end
end