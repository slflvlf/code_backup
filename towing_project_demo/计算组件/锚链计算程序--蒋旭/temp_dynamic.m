g=9.80665;
L=50;            %分段长m
H=1500;          %水深
Time=400;        %时间 s
p0=[0;0;0];      %坐标m
p1=[800;0;-1500];
%% 定义参数
[dt,k0,k1,k2,k3,k4,k5,CDn,CAn,pho_water,current,nt]=constant(Time);
%% 分段参数
[N,W,EA,D]=element_parameter;
%% 定义质量
[Ms]=mass_par_length(N,L,W);
[Ms_add]=mass_add(N,L,W);
 %% 重力
 [G]=gravity(N,Ms,g);
%% 质点坐标初始化
[Y_N, p_init0]=initial_position_horizon(p0,p1,H,N);
% [Y_N]=initial_position(p0,p1,H,N);
%%%%%%%%%%%%%  初值结束 开始迭代  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while 1
 %% 单元实际长度 
 [l]=element_length(Y_N,N);
 %% 海底支持力
 [Fb]=seabed(N,Ms,Y_N,H,g);
 %% 张力
 [T]=tension(N,EA,L,l);
 %% 张力的雅可比矩阵
 [KT]=tension_K(N,EA,L,l,Y_N);
 %% 海底支持力的雅可比矩阵
 [KB]=seabed_K(N,Ms,H,Y_N,g);
 %% 总的雅克比矩阵
 K=KT+KB;
 %% 边界条件
 BON=zeros(3*N-3,1);
 BON(1:3,1)=-T(1:3,1:3)*Y_N(1:3,1);
 BON(3*N-5:3*N-3,1)=-T(3*N+1:3*N+3,3*N+1:3*N+3)*Y_N(3*N+1:3*N+3,1);
 %% 方程组合
 FY=T(4:3*N,4:3*N)*Y_N(4:3*N,1)+G(4:3*N,1)+Fb(4:3*N,1)+BON;
%% 求解
R=1/2;
Y_N1=Y_N(4:3*N,1)-K(4:3*N,4:3*N)\FY*R;
%% 迭代终止条件
if norm(Y_N1-Y_N(4:3*N,1))<1e-7
    Y_N(4:3*N,1)=Y_N1;
    break
end
%% 下一步迭代
Y_N(4:3*N,1)=Y_N1;
end
% TE1=EA(1,1)*(l(1,1)/L-1)*(Y_N(4:6,1)-Y_N(1:3,1))/norm(Y_N(4:6,1)-Y_N(1:3,1))+G(1:3,1);
% T1=norm(TE1);
%%%%%%%%%%%%%%%%%%静力计算结束%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 动力计算
%% 定义运动后导缆孔的位置 和 参数初始化
aa=zeros(3*N,nt+1);
vv=zeros(3*N,nt+1);
[P_initial]=motion(nt,N,Y_N,dt);
%% 时间迭代
for t=2:100
    t
    P_initial(4:3*N,t)=P_initial(4:3*N,t-1);
    Y_Nk=P_initial(:,t-1);
    Y_N=P_initial(:,t);
   %% 单元实际长度 上一时间点
    [lk]=element_length(Y_Nk,N);
   %% 张力的雅可比矩阵 上一时间点
    [KT]=tension_K_k(N,EA,L,lk,Y_Nk);
   %% 海底支持力的雅可比矩阵 上一时间点
    [KB]=seabed_K_k(N,Ms,H,Y_Nk,g);
   %% 计算雅可比矩阵使用的质量矩阵 上一时间点
   [Msk]=mass_K(N,Y_Nk,pho_water,D,CAn,L);
    K=KT(4:3*N,4:3*N)+KB(4:3*N,4:3*N)-k0*(Msk(4:3*N,4:3*N)+Ms_add(4:3*N,4:3*N))+10^4*eye(3*N-3);
   %% 迭代初始化
   delta_y=zeros(3*N-3,1);
   %% 当前时间迭代
while 1
   %% 单元实际长度  本时间点
    [l]=element_length(Y_N,N);   
   %% 附加质量    本时间点
   [MsN]=mass_N(N,Y_N,pho_water,D,CAn,L);
   %% 海底支持力   本时间点
   [Fb]=seabed(N,Ms,Y_N,H,g);
   %% 张力   本时间点
   [T]=tension(N,EA,L,l);
   %% 质点的数度加速度  本时间点
   [aa,vv]=vel_acc(delta_y,vv,aa,k0,k1,k2,k3,k4,k5,t,N,Y_N,Y_Nk);
   %% 拖曳力  本时间点
   [Fd]=drag_F(N,Y_N,H,current,vv,t,pho_water,D,L,CDn);
   %% 边界条件  本时间点
    BON=zeros(3*N-3,1);
    BON(1:3,1)=EA(1,1)*(1/L-1/l(1,1))*eye(3)*Y_N(1:3,1);
    BON(3*N-5:3*N-3,1)=EA(N,1)*(1/L-1/l(N,1))*eye(3)*Y_N(3*N+1:3*N+3,1);
   %% 方程组合   本时间点
   FY=T(4:3*N,4:3*N)*Y_N(4:3*N,1)+G(4:3*N,1)+Fb(4:3*N,1)+BON+Fd-(MsN(4:3*N,4:3*N)+Ms_add(4:3*N,4:3*N))*aa(4:3*N,t); 
   %% 解方程    本时间点
    y=-K\FY;
    Y_N(4:3*N,1)=Y_N(4:3*N,1)-K\FY;
    delta_y=delta_y-K\FY;   
    if norm(y)<1e-7
       aa(4:3*N,t)=k0*delta_y-k1*vv(4:3*N,t-1)-k2*aa(4:3*N,t-1);                          
       vv(4:3*N,t)=k3*delta_y+k4*vv(4:3*N,t-1)-k5*aa(4:3*N,t-1);
       break    
    end
end
P_initial(4:3*N,t)=Y_N(4:3*N,1); 
end
























