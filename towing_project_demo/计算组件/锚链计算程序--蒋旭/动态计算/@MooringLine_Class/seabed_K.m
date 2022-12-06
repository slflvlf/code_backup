function [KB]=seabed_K(obj, N,Ms,H,Y_N,g)
KB=cell(N+1,N+1);
for i=1:N+1
    for j=1:N+1
    KB{i,j}=[0 0 0; 0 0 0 ; 0 0 0];
    end
end
for i=1:1:N+1
    a_0=[0 0 0;0 0 0;0 0 0];
    a_0(3,3)=-Ms(i,1)*g/(cosh(H+Y_N(3*i,1)))^2;%%%!!!!!!!
    KB{i,i}=a_0;
end
KB=cell2mat(KB);
end