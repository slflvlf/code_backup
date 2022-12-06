function [TR1, TRN, vv] = mooringline_dynamic_force_fcn(p0, p1, p0_next, p1_next, mooring_param)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 集中质量法的动态计算函数 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 这个计算结果是对的
% p0, p1, 上一时刻的首尾位置
% p0_next, p1_next: 当前时刻的首尾位置
% mooring_param: 计算相关参数设定，其中时间步长很关键, 最好选择0.1，如果控制步长是1s的话，可以把位置离散然后再计算

% 输出： TR1， TRN: 首尾的张力 单位KN，应该是大地坐标系的，作用于船上需要转换
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




% 这一个是在梁博的版本上的初步修改

% mooring_param = struct('length', 300, 'element_length', 10, 'element_number', 30,... 
%                 'node_number', 31, 'E', 9.16e10, 'diameter', 0.09, 'cross_area', 6.362e-3, ...
%                 'density', 126, 'time_step', 0.1, 'coef_drag', 1, 'coef_addmass', 1,...
%                 'current', [0; 0; 0]);

g=9.80665;
L = mooring_param.element_length;            %分段长m
H=1500;          %水深
Time=1;        %时间 s
% p0=[0;0;2];      %坐标m
% p1=[310;0;2];
% p0_next=[0;0;2];      %坐标m
% p1_next=[315;0;2];


%% 定义参数
[dt,k0,k1,k2,k3,k4,k5,CDn,CAn,pho_water,current,nt]=constant(Time, mooring_param);
% dt = 0.1;
% dt反应在k0这些迭代参数上，最终体现在速度和加速度的迭代上
% nt = 1;
%% 分段参数
[N,W,EA,D]=element_parameter(mooring_param);
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
%% 静力计算结果
p_static_final = getMooringLineShape(Y_N,N);

% figure(1)
% plot(p_init0(:, 1), p_init0(:, 3))
% hold on
% plot(p_static_final(:, 1), p_static_final(:, 3))
% hold off
% title("static");
% legend("init", "final") 
% legend boxoff

% TE1=EA(1,1)*(l(1,1)/L-1)*(Y_N(4:6,1)-Y_N(1:3,1))/norm(Y_N(4:6,1)-Y_N(1:3,1))+G(1:3,1);
% T1=norm(TE1);
%%%%%%%%%%%%%%%%%%静力计算结束%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 动力计算

%% 定义运动后导缆孔的位置 和 参数初始化
% aa=zeros(3*N,nt+1);
% vv=zeros(3*N,nt+1);
aa=zeros(3*N+3,nt+1);
vv=zeros(3*N+3,nt+1);
% [P_initial]=motion1(nt,N,Y_N,dt);
[P_initial]=motion2(nt,N,Y_N,p0_next, p1_next);
p_dynamic0 = getMooringLineShape(P_initial(:, 2),N);
p_dynamic_init = getMooringLineShape(P_initial(:, 1),N);

%% 时间迭代
iter = 0;
for t=2:2
    
    t;
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
%    tic
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
    norm(y);
    if norm(y)<1e-3
        
        aa(1:3,t)=k0*(Y_N(1:3,1)-Y_Nk(1:3,1))-k1*vv(1:3,t-1)-k2*aa(1:3,t-1);
        vv(1:3,t)=k3*(Y_N(1:3,1)-Y_Nk(1:3,1))+k4*vv(1:3,t-1)-k5*aa(1:3,t-1);
        aa(4:3*N,t)=k0*delta_y-k1*vv(4:3*N,t-1)-k2*aa(4:3*N,t-1);                          
        vv(4:3*N,t)=k3*delta_y+k4*vv(4:3*N,t-1)-k5*aa(4:3*N,t-1);
        aa(3*N+1:end, t) = k0*(Y_N(3*N+1:end, 1)-Y_Nk(3*N+1:end, 1))-k1*vv(3*N+1:end,1)-k2*aa(3*N+1:end,1);
        vv(3*N+1:end, t) = k3*(Y_N(3*N+1:end, 1)-Y_Nk(3*N+1:end, 1))+k4*vv(3*N+1:end,1)-k5*aa(3*N+1:end,1);
%        aa(4:3*N,t)=k0*delta_y-k1*vv(4:3*N,t-1)-k2*aa(4:3*N,t-1);                          
%        vv(4:3*N,t)=k3*delta_y+k4*vv(4:3*N,t-1)-k5*aa(4:3*N,t-1);
       break    
    end
    
end
% time_sum = toc;
% if time_sum > 30
%     disp('锚链计算失败');toc
% end
% sprintf("动态计算完成，迭代了%d次.", iter-1)
P_initial(4:3*N,t)=Y_N(4:3*N,1); 
% 计算张力
tvec1 = (Y_N(4:6,1)-Y_N(1:3,1))/norm(Y_N(4:6,1)-Y_N(1:3,1));
tvecN = (Y_N(3*N-2:3*N,1)-Y_N(3*N+1:3*N+3,1))/norm(Y_N(3*N-2:3*N,1)-Y_N(3*N+1:3*N+3,1));
delta_vv1 = current-vv(1:3)';
delta_vvN = current-vv(3*N+1:3*N+3)';
TR1 = -(l(1)-L)/L*EA(1)*tvec1-0.25*pho_water*D(1)*CDn*(delta_vv1-(delta_vv1'*tvec1)*tvec1)...
    *norm(delta_vv1-(delta_vv1'*tvec1)*tvec1)-G(1:3);
TRN = -(l(end)-L)/L*EA(end)*tvecN-0.25*pho_water*D(end)*CDn*(delta_vvN-(delta_vvN'*tvecN)*tvecN)...
    *norm(delta_vvN-(delta_vvN'*tvecN)*tvecN)-G(3*N+1:3*N+3);
%锚链作用于船上的力
TR1 = -TR1/1e3;
TRN = -TRN/1e3;
end
% toc


% % p_dynamic_final = getMooringLineShape(Y_N);
% p_dynamic_final = getMooringLineShape(P_initial(:, 2),N);
% 
% figure(2)
% plot(p_dynamic_init(:, 1), p_dynamic_init(:, 3))
% hold on
% plot(p_dynamic_final(:, 1), p_dynamic_final(:, 3))
% hold off
% legend('初始形状', '最终形状'); legend boxoff;
% title('动态计算锚链形态');







end




function p_final = getMooringLineShape(Y_N, N)
p_final = zeros(N+1, 3);
for i = 1:N+1
    p_final(i, 1) = Y_N(3*i-2, 1);
    p_final(i, 2) = Y_N(3*i-1, 1);
    p_final(i, 3) = Y_N(3*i, 1);
end
end


function t = getTangentUnitVector(Y_N, N)
    t = zeros(N+1, 3);
    for i = 1 : N
        x0 = Y_N(3*i-2:3*i, 1);
        x1 = Y_N(3*i+1:3*i+3, 1);
        t(i, :) = (x1-x0)/norm(x1-x0);
    end

end










