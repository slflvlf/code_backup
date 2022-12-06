function [Msk]=mass_K(N,Y_Nk,pho_water,D,CAn,L)
t=zeros(3*N,1);
for i=1:N
    t(3*i-2:3*i,1)=(Y_Nk(3*i+1:3*i+3,1)-Y_Nk(3*i-2:3*i,1))/norm(Y_Nk(3*i+1:3*i+3,1)-Y_Nk(3*i-2:3*i,1));    
end
Msk=cell(N+1,N+1);
for i=1:N+1
    for j=1:N+1
        Msk{i,j}=[0 0 0;0 0 0;0 0 0];
    end
end
a=cell(N,1);
for i=1:N
    a0=0.125*pho_water*pi*D(i)^2*CAn*L*(eye(3)-t(3*i-2:3*i,1)*t(3*i-2:3*i,1)');
    a{i,1}=a0;
end
Msk{1,1}=a{1,1};
Msk{N+1,N+1}=a{N,1};
for i=2:N
    Msk{i,i}=a{i,1}+a{i-1,1};
end
Msk=cell2mat(Msk);
end