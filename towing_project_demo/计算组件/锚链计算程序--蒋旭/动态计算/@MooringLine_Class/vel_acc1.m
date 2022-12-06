function [aa,vv]=vel_acc1(obj, delta_y,vv,aa,k0,k1,k2,k3,k4,k5,t,N,Y_N,Y_Nk)
aa(1:3,t)=k0*(Y_N(1:3,1)-Y_Nk(1:3,1))-k1*vv(1:3,t-1)-k2*aa(1:3,t-1);
vv(1:3,t)=k3*(Y_N(1:3,1)-Y_Nk(1:3,1))+k4*vv(1:3,t-1)-k5*aa(1:3,t-1);
aa(4:3*N,t)=k0*delta_y-k1*vv(4:3*N,t-1)-k2*aa(4:3*N,t-1);                          
vv(4:3*N,t)=k3*delta_y+k4*vv(4:3*N,t-1)-k5*aa(4:3*N,t-1);
aa(3*N+1:end, t) = k0*(Y_N(3*N+1:end, 1)-Y_Nk(3*N+1:end, 1))-k1*vv(3*N+1:end,1)-k2*aa(3*N+1:end,1);
vv(3*N+1:end, t) = k3*(Y_N(3*N+1:end, 1)-Y_Nk(3*N+1:end, 1))+k4*vv(3*N+1:end,1)-k5*aa(3*N+1:end,1);
end