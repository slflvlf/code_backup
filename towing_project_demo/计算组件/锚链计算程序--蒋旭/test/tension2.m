function [T, Adcell]=tension2(N,EA,L,l)
T=cell(N-1,N-1);                           
for i=1:N-1
    for j=1:N-1
    T{i,j}=zeros(3,3);
    end
end
     Adcell=cell(N,1);   
   for i=1:N
       Adcell{i,1}=zeros(3,3);
   end
   for i=1:N
        for q=1:3
           Adcell{i,1}(q,q)=EA(i)*(1/L-1/l(i));
        end
   end
    for i=1:N-2
        T{i,i+1}=Adcell{i+1,1};
        T{i+1,i}=Adcell{i+1,1};
    end
    for i=1:N-1
        T{i,i}=-Adcell{i,1}-Adcell{i+1,1};
    end
    T=cell2mat(T);
end