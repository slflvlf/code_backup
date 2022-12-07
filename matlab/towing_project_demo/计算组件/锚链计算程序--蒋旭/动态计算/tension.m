function [T]=tension(N,EA,L,l)
T=cell(N+1,N+1);
for i=1:N+1
    for j=1:N+1
    T{i,j}=[0 0 0; 0 0 0 ; 0 0 0];
    end
end
a=cell(N+1,1);
for i=1:1:N
   a_0=EA(i,1)*(1/L-1/l(i,1))*eye(3); 
   a{i,1}=a_0;
end
T{1,1}=-a{1,1};
T{1,2}=a{1,1};
T{N+1,N}=a{N,1};
T{N+1,N+1}=-a{N,1};
for i=2:1:N
    T{i,i-1}=a{i-1,1};
    T{i,i}=-a{i-1,1}-a{i,1};
    T{i,i+1}=a{i,1};
end
T=cell2mat(T);
end