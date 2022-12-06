function [A_ineq,b_ineq,lb,ub]=phylimofthr(U0,flag)
%This function is used to set the physical limitations of the azimuth
%thrusters. 

% Input
% U0        上一步的控制序列 单船8xNcx1  多船3x8xNc
% flag      flag==1 单船 flag==2 多船
% Output
% A_ineq    Ax<=b中的A矩阵
% b_ineq    Ax<=b中的b矩阵
% lb        自变量U的下限
% ub        自变量U的上限

global Nc
global N
%% 
%thrust saturation and angle limitation
u_lb=[-20;-20;-20;-20;-2*pi;-2*pi;-2*pi;-2*pi];%单位 N
u_ub=[20;20;20;20;2*pi;2*pi;2*pi;2*pi]; 
% u_lb=[-200;-200;-200;-200;-2*pi;-2*pi;-2*pi;-2*pi];%单位 N
% u_ub=[200;200;200;200;2*pi;2*pi;2*pi;2*pi]; 

%repmat实现
U_lb=repmat(u_lb,Nc,1);
U_ub=repmat(u_ub,Nc,1);

%%
%delta thrust and delta angle u(-1)=u(t-1)
du_lb=-[4;4;4;4;deg2rad(15);deg2rad(15);deg2rad(15);deg2rad(15)];
du_ub=[4;4;4;4;deg2rad(15);deg2rad(15);deg2rad(15);deg2rad(15)];
% du_lb=-[20;20;20;20;deg2rad(15);deg2rad(15);deg2rad(15);deg2rad(15)];
% du_ub=[20;20;20;20;deg2rad(15);deg2rad(15);deg2rad(15);deg2rad(15)];

%repmat实现
dU_lb=repmat(du_lb,Nc,1);
dU_ub=repmat(du_ub,Nc,1);

A_dU_cell=cell(Nc);%不等式约束的系数矩阵
for i=1:Nc
    for j=1:Nc
        if i==j 
            A_dU_cell{i,j}=eye(2*N);
        elseif i==j+1
            A_dU_cell{i,j}=-eye(2*N);
        else
            A_dU_cell{i,j}=zeros(2*N);
        end
    end
end
A_dU=cell2mat(A_dU_cell);

%%
%判断单船or多船
if flag==1         %单船
    dU_lb(1:2*N)=U0(1:2*N)+du_lb;%第一项改成u0+du
    dU_ub(1:2*N)=U0(1:2*N)+du_ub;
    
    A_ineq=[A_dU;-A_dU];
    b_ineq=[dU_ub;-dU_lb];
    lb=U_lb;
    ub=U_ub;
    
elseif flag==2     %多船
    dU_lb_self=dU_lb;dU_lb_self(1:2*N)=U0(1:2*N)+du_lb;%self
    dU_ub_self=dU_ub;dU_ub_self(1:2*N)=U0(1:2*N)+du_ub;
    dU_lb_neighbor1=dU_lb;dU_lb_neighbor1(1:2*N)=U0(2*N*Nc+1:2*N*Nc+2*N)+du_lb;%neighbor1
    dU_ub_neighbor1=dU_ub;dU_ub_neighbor1(1:2*N)=U0(2*N*Nc+1:2*N*Nc+2*N)+du_ub;
    dU_lb_neighbor2=dU_lb;dU_lb_neighbor2(1:2*N)=U0(2*N*2*Nc+1:2*N*2*Nc+2*N)+du_lb;%neighbor2
    dU_ub_neighbor2=dU_ub;dU_ub_neighbor2(1:2*N)=U0(2*N*2*Nc+1:2*N*2*Nc+2*N)+du_ub;
    
    A_ineq_temp=[A_dU zeros(size(A_dU)) zeros(size(A_dU));zeros(size(A_dU)) A_dU zeros(size(A_dU));zeros(size(A_dU)) zeros(size(A_dU)) A_dU];
    A_ineq=[A_ineq_temp;-A_ineq_temp];
    
    b_ineq=[dU_ub_self;dU_ub_neighbor1;dU_ub_neighbor2;-dU_lb_self;-dU_lb_neighbor1;-dU_lb_neighbor2];
    
    lb=[U_lb;U_lb;U_lb];
    ub=[U_ub;U_ub;U_ub];
end %if

end %function