function [MsN]=mass_N(N,Y_N,pho_water,D,CAn,L)
t=zeros(3*N,1);
for i=1:N
    t(3*i-2:3*i,1)=(Y_N(3*i+1:3*i+3,1)-Y_N(3*i-2:3*i,1))/norm(Y_N(3*i+1:3*i+3,1)-Y_N(3*i-2:3*i,1));    
end
MsN=cell(N+1,N+1);
for i=1:N+1
    for j=1:N+1
        MsN{i,j}=[0 0 0;0 0 0;0 0 0];
    end
end
a=cell(N,1);
for i=1:N
    a0=0.125*pho_water*pi*D(i)^2*CAn*L*(eye(3)-t(3*i-2:3*i,1)*t(3*i-2:3*i,1)');
    a{i,1}=a0;
end
MsN{1,1}=a{1,1};
MsN{N+1,N+1}=a{N,1};
for i=2:N
    MsN{i,i}=a{i,1}+a{i-1,1};
end
MsN=cell2mat(MsN);
end