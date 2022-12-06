
%% 初始化位置测试（水平）
% clear all;  clc;
% 
% p0 = [0, 0, 2.2]';
% p1 = [400, 0, 2.2]';
% N = 50;
% H = 1500;
% 
%  [Y_N, p_init0]=initial_position_horizon(p0,p1,H,N);
% % plot3(p_init(:, 1), p_init(:, 2), p_init(:, 3))
% figure(1)
% plot(p_init0(:, 1), p_init0(:, 3))

%% 深海锚链初始化测试



%% 
%% 常数
clear all;  clc;
g=9.80665;
% Lz = 2000;

H=1500;
% p0 = [34.3, -41.8, 2]';
% p1 = [3414.63, 1993.52, -1500]';
% p0 = [0, 0, 1]';
% p1 = [300, 0, 1]';
p0 = [0, 0, 1]';
p1 = [300, 10, 1]';


mooring_param = struct('length', 300, 'element_length', 10, 'element_number', 30,... 
                'node_number', 31, 'E', 9.16e10, 'diameter', 0.09, 'cross_area', 6.362e-3, ...
                'density', 126);

% p0=[0;0;0];
% p1=[800;0;-2];
%% 分段参数
% N总段数， W:水中单位质量 EA：刚度 D：直径
N = mooring_param.element_number;
L = mooring_param.element_length;
[N,W,EA,D]=element_parameter(mooring_param);
%% 定义质量
% 各节点的质量N+1
% N = 
% L = Lz / N;
[Ms]=mass_par_length(N,L,W);
%% 质点坐标初始化
% 节点坐标，合成一列了（3*N+3, 1)
[P_initial, p_init0]=initial_position_horizon(p0,p1,H,N);
% [P_initial]=initial_position(p0,p1,H,N);
% Y_N=P_initial;
Y_N=P_initial;

%%%%%%%%%%%%%%  初值结束 开始迭代  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while 1
 %% 单元实际长度 
 % (N, 1)
 [l]=element_length(Y_N,N);
 %% 海底支持力
 [Fb]=seabed(N,Ms,Y_N,H,g);
 Fb_1=Fb(4:3*N,1);  %去除边界条件，（3N-3, 1)
 %% 重力
 [G]=gravity(N,Ms,g);
 G_1=G(4:3*N,1);
 %% 张力
 [T]=tension(N,EA,L,l);
 T_1=T(4:3*N,4:3*N);
 %% 张力的雅可比矩阵
 [KT]=tension_K(N,EA,L,l,Y_N);
 %% 海底支持力的雅可比矩阵
 [KB]=seabed_K(N,Ms,H,Y_N,g);
 %% 总的雅克比矩阵
 K=KT+KB;
 K_1=K(4:3*N,4:3*N);
 %% 边界条件
 BON=zeros(3*N-3,1);
 BON(1:3,1)=-T(1:3,1:3)*Y_N(1:3,1);
 BON(3*N-5:3*N-3,1)=-T(3*N+1:3*N+3,3*N+1:3*N+3)*Y_N(3*N+1:3*N+3,1);
 %% 方程组合
 FY=T_1*Y_N(4:3*N,1)+G_1+Fb_1+BON;
%% 求解
R=1/2;
Y_N1=Y_N(4:3*N,1)-K_1\FY*R;
%% 迭代终止条件
if norm(Y_N1-Y_N(4:3*N,1))<1e-7
    
    
    Y_N(4:3*N,1)=Y_N1;
    break
end
%% 下一步迭代
Y_N(4:3*N,1)=Y_N1;
end

TE=EA(1,1)*(l(1,1)/L-1)*(Y_N(4:6,1)-Y_N(1:3,1))/norm(Y_N(4:6,1)-Y_N(1:3,1))+G(1:3,1);
TE1=EA(N,1)*(l(N,1)/L-1)*(Y_N(3*N+1:3*N+3,1)-Y_N(3*N-2:3*N,1))/norm(Y_N(3*N+1:3*N+3,1,1)-Y_N(3*N-2:3*N,1))+G(3*N+1:3*N+3,1);
TE_begin = [TE(1); TE(2); 0]
TE_end = [-TE1(1); -TE1(2); 0]

p_final = zeros(N+1, 3);
for i = 1:N+1
    p_final(i, 1) = Y_N(3*i-2, 1);
    p_final(i, 2) = Y_N(3*i-1, 1);
    p_final(i, 3) = Y_N(3*i, 1);
end
figure(1)
plot(p_init0(:, 1), p_init0(:, 3))
hold on
plot(p_final(:, 1), p_final(:, 3))
hold off
legend('初始形状', '最终形状');
title('锚链形状变化');







