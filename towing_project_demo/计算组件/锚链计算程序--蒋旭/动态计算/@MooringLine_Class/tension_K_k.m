function [KT]=tension_K_k(obj, N,EA,L,lk,Y_Nk)
KT=cell(N+1,N+1);
for i=1:N+1
    for j=1:N+1
    KT{i,j}=[0 0 0; 0 0 0 ; 0 0 0];
    end
end
a=cell(N,1);
a_0=[0 0 0;0 0 0;0 0 0];
for i=1:1:N
    for q=1:1:3
        for r=1:1:3
            if q==r
                a_0(q,r)=EA(i,1)*(1/lk(i,1)-1/L)-EA(i,1)*(Y_Nk(3*i+q,1)-Y_Nk(3*(i-1)+q,1))*(Y_Nk(3*i+r,1)-Y_Nk(3*(i-1)+r,1))/lk(i,1)^3;
            else
                a_0(q,r)=-EA(i,1)*(Y_Nk(3*i+q,1)-Y_Nk(3*(i-1)+q,1))*(Y_Nk(3*i+r,1)-Y_Nk(3*(i-1)+r,1))/lk(i,1)^3;
            end
        end
    end
   a{i,1}=a_0;
end
KT{1,1}=a{1,1};
KT{1,2}=-a{1,1};
KT{N+1,N}=-a{N,1};
KT{N+1,N+1}=a{N,1};
for i=2:1:N
    KT{i,i-1}=-a{i-1,1};
    KT{i,i}=a{i-1,1}+a{i,1};
    KT{i,i+1}=-a{i,1};
end
KT=cell2mat(KT);
end