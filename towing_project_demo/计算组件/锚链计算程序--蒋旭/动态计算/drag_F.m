function [Fd]=drag_F(N,Y_N,H,current,vv,t,pho_water,D,L,CDn)
t1=zeros(3*N,1);
for i=1:N
    t1(3*i-2:3*i,1)=(Y_N(3*i+1:3*i+3,1)-Y_N(3*i-2:3*i,1))/norm(Y_N(3*i+1:3*i+3,1)-Y_N(3*i-2:3*i,1));    
end
for i=1:N-1
    VR=(1+Y_N(3*i+3,1)/H)*current-vv(3*i+1:3*i+3,t);
    Fd(3*i-2:3*i,1)=0.25*pho_water*D(i)*L*CDn*(VR-VR'*t1(3*i-2:3*i,1)*t1(3*i-2:3*i,1))*norm(VR-VR'*t1(3*i-2:3*i,1)*t1(3*i-2:3*i,1))+...
                    0.25*pho_water*D(i+1)*L*CDn*(VR-VR'*t1(3*i+1:3*i+3,1)*t1(3*i+1:3*i+3,1))*norm(VR-VR'*t1(3*i+1:3*i+3,1)*t1(3*i+1:3*i+3,1));
end
end 