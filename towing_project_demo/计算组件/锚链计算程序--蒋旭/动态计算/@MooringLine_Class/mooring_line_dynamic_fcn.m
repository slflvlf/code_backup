function [TR1, TRN, Y_N, p_final] = mooring_line_dynamic_fcn(obj, p0_next, p1_next, Y_N)
%% 帮助文档

%输入参数
% p0_next, p1_next: 缆绳两端的当前位置（x, y, z)',全局坐标系下。上一步的位置体现在Y_N中
% Y_N: 上一步各节点的位置（3N+3, 1)
% vv， aa: 各个节点的速度和加速度(3*N+3, 2)，包括上一步和当前的，当前的信息不准，需要迭代

% 输出参数
% Y_N:  输出的当前的各节点的位置（3N+3， 1）
% p_final: 输出的当前各节点的位置（N+1, 3), 画图用
% vv, aa: 输出的各个节点的速度和加速度(3*N+3, 2)，包括上一步和当前的，当前的信息是准，需要迭代

% 需要注意的， 缆绳离散过程中，总长 = L * N， 在函数内合理选择L和N， N在element_parameter函数内设定
% constant函数里有些参数也需要确定，current是流速， CD和CA也需要设定, dt是时间步长，和模拟的一样
% element_parameter函数也需要设定
% 上述参数都在mooring_param里设定了

% 此外Y_N,vv, aa都是迭代参数，vv和aa简单设置成零就可以，Y_N是经过静态计算得到锚链形态，计算开始前计算一下就行
mooring_param = obj.mooring_param;

%% 基本参数
g=9.80665;
% L=20;           %% 分段长
H=1500;
Time=2;        %时间 s

%% 分段参数
% N总段数， W:水中单位质量 EA：刚度 D：直径
N = mooring_param.element_number;
L = mooring_param.element_length;
[N,W,EA,D]=element_parameter(mooring_param);
[vv, aa] = deal(zeros(3*N+3, 2)); %这里是因为以前的程序的迭代都是基于零速和加速度
%% 定义参数
[dt,k0,k1,k2,k3,k4,k5,CDn,CAn,pho_water,current,nt]=constant(Time, mooring_param);
%% 定义质量
[Ms]=mass_par_length(N,L,W);
[Ms_add]=mass_add(N,L,W);

 %% 重力
 [G]=gravity(N,Ms,g);

%% 动力计算
%% 定义运动后导缆孔的位置 和 参数初始化
% aa=zeros(3*N,nt+1);
% vv=zeros(3*N,nt+1);
% aa=zeros(3*N+3,nt+1);
% vv=zeros(3*N+3,nt+1);
% [P_initial]=motion1(nt,N,Y_N,dt);
[P_initial]=motion2(nt,N,Y_N,p0_next, p1_next);
p_dynamic0 = getMooringLineShape(P_initial(:, 2),N);
p_dynamic_init = getMooringLineShape(P_initial(:, 1),N);

%% 时间迭代，这里是按照梁博的程序改的，t=2，是前一步t=1和当前步t=2
for t=2:2
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
    K=KT(4:3*N,4:3*N)+KB(4:3*N,4:3*N)-k0*(Msk(4:3*N,4:3*N)+Ms_add(4:3*N,4:3*N))+10^5*eye(3*N-3);
   %% 迭代初始化
   delta_y=zeros(3*N-3,1);
   %% 当前时间迭代
   iter = 0;
while 1
    iter = iter+1;
   %% 单元实际长度  本时间点
    [l]=element_length(Y_N,N);   
   %% 附加质量    本时间点
   [MsN]=mass_N(N,Y_N,pho_water,D,CAn,L);
   %% 海底支持力   本时间点
   [Fb]=seabed(N,Ms,Y_N,H,g);
   %% 张力   本时间点
   [T]=tension(N,EA,L,l);
   %% 质点的数度加速度  本时间点
   [aa,vv]=vel_acc1(delta_y,vv,aa,k0,k1,k2,k3,k4,k5,t,N,Y_N,Y_Nk);
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
    if norm(y)<1e-3
        aa(1:3,t)=k0*(Y_N(1:3,1)-Y_Nk(1:3,1))-k1*vv(1:3,t-1)-k2*aa(1:3,t-1);
        vv(1:3,t)=k3*(Y_N(1:3,1)-Y_Nk(1:3,1))+k4*vv(1:3,t-1)-k5*aa(1:3,t-1);
        aa(4:3*N,t)=k0*delta_y-k1*vv(4:3*N,t-1)-k2*aa(4:3*N,t-1);                          
        vv(4:3*N,t)=k3*delta_y+k4*vv(4:3*N,t-1)-k5*aa(4:3*N,t-1);
        aa(3*N+1:end, t) = k0*(Y_N(3*N+1:end, 1)-Y_Nk(3*N+1:end, 1))-k1*vv(3*N+1:end,1)-k2*aa(3*N+1:end,1);
        vv(3*N+1:end, t) = k3*(Y_N(3*N+1:end, 1)-Y_Nk(3*N+1:end, 1))+k4*vv(3*N+1:end,1)-k5*aa(3*N+1:end,1);
       break    
    end

    if iter > 500
        error('锚链计算失败');
    end
    
end
% sprintf("动态计算完成，迭代了%d次.", iter-1)
P_initial(4:3*N,t)=Y_N(4:3*N,1); 

tvec1 = (Y_N(4:6,1)-Y_N(1:3,1))/norm(Y_N(4:6,1)-Y_N(1:3,1));
tvecN = (Y_N(3*N-2:3*N,1)-Y_N(3*N+1:3*N+3,1))/norm(Y_N(3*N-2:3*N,1)-Y_N(3*N+1:3*N+3,1));
delta_vv1 = current-vv(1:3)';
delta_vvN = current-vv(3*N+1:3*N+3)';
TR1 = -(l(1)-L)/L*EA(1)*tvec1-0.25*pho_water*D(1)*CDn*(delta_vv1-(delta_vv1'*tvec1)*tvec1)...
    *norm(delta_vv1-(delta_vv1'*tvec1)*tvec1)-G(1:3);
TRN = -(l(end)-L)/L*EA(end)*tvecN-0.25*pho_water*D(end)*CDn*(delta_vvN-(delta_vvN'*tvecN)*tvecN)...
    *norm(delta_vvN-(delta_vvN'*tvecN)*tvecN)-G(3*N+1:3*N+3);
%锚链作用于船上的力, KN
TR1 = -TR1/1e3;
TRN = -TRN/1e3;
end

p_final = getMooringLineShape(P_initial(:, 2),N);
end
% p_dynamic_final = getMooringLineShape(Y_N);
% p_dynamic_final = getMooringLineShape(P_initial(:, 2),N);
% 







function p_final = getMooringLineShape(Y_N, N)
p_final = zeros(N+1, 3);
for i = 1:N+1
    p_final(i, 1) = Y_N(3*i-2, 1);
    p_final(i, 2) = Y_N(3*i-1, 1);
    p_final(i, 3) = Y_N(3*i, 1);
end
end
