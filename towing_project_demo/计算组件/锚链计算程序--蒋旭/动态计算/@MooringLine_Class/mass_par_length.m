function [Ms]=mass_par_length(obj, N,L,W)
Ms=zeros(N+1,1);
Ms(1,1)=L*W(1,1)/2;
Ms(N+1,1)=L*W(N,1)/2;
for i=2:1:N
    Ms(i,1)=W(i-1,1)*L/2+W(i,1)*L/2;
end
end