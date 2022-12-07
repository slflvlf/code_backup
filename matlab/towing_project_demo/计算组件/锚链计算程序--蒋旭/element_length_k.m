function [lk]=element_length_k(Y_Nk,N)
lk=zeros(N,1);
for i=1:1:N
   lk(i,1)=sqrt((Y_Nk(3*i+1,1)-Y_Nk(3*(i-1)+1,1))^2+...
               (Y_Nk(3*i+2,1)-Y_Nk(3*(i-1)+2,1))^2+...
               (Y_Nk(3*i+3,1)-Y_Nk(3*(i-1)+3,1))^2); 
end
end