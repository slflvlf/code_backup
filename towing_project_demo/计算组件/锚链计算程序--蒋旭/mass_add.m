function [Ms_add]=mass_add(N,L,W)
Ms_add=zeros(3*N+3,3*N+3);
Ms_add(1,1)=L*W(1,1)/2;
Ms_add(2,2)=L*W(1,1)/2;
Ms_add(3,3)=L*W(1,1)/2;
Ms_add(3*N+1,3*N+1)=L*W(N,1)/2;
Ms_add(3*N+2,3*N+2)=L*W(N,1)/2;
Ms_add(3*N+3,3*N+3)=L*W(N,1)/2;
for i=2:1:N
    Ms_add(3*i-2,3*i-2)=W(i-1,1)*L/2+W(i,1)*L/2;
    Ms_add(3*i-1,3*i-1)=W(i-1,1)*L/2+W(i,1)*L/2;
    Ms_add(3*i,3*i)=W(i-1,1)*L/2+W(i,1)*L/2;
end
end