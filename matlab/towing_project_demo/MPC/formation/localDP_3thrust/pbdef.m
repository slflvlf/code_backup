clear all; clc;

% addpath("C:\Users\jiang\Desktop\nlmpc\pdfmpc")

%-------------------------------------------------------------------------------
% Definition of p_ode
%-------------------------------------------------------------------------------

M1 = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
Ma1 = [7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5, 8.77e6];
MM1 = M1 + Ma1;
D1 = [MM1(1,1)/100, 0, 0; 0, MM1(2,2)/40, 0; 0, 0, MM1(3,3)/20];
p_ode.tau=1;%步长
p_ode.rk_order=4;%积分阶数
p_ode.x0=[0; 0; 0; 0; 0; 0];%初值
p_ode.u0=[120, 60, 60, 180*pi/180, 0*pi/180, 0*pi/180]';
p_ode.M = MM1;
p_ode.D = D1;
p_ode.thrust_config = [-5.02 0; 11.956 2.7; 11.956 -2.7];


%-------------------------------------------------------------------------------
% Definition of p_uparam
%-------------------------------------------------------------------------------
p_uparam.nu=6;
p_uparam.Np=20;
p_uparam.Ifree=[1;2;3;10;];
p_uparam.R=compute_R(p_uparam.Ifree, p_uparam.Np, p_uparam.nu);
p_uparam.np=size(p_uparam.R,2);
p_uparam.p=repmat(p_ode.u0, size(p_uparam.Ifree, 1), 1);
% p_uparam.p=zeros(p_uparam.np,1);

F_max = 400;%KN
dF_max = 50;
da_max = 10/180*pi;
% p_uparam.pmin = reshape([0, 0, 0, -2*pi -2*pi, -2*pi]'*ones(1,size(p_uparam.Ifree, 1)),p_uparam.np,1);
p_uparam.pmin=repmat([0, 0, 0, -2*pi -2*pi, -2*pi]', size(p_uparam.Ifree, 1), 1);
p_uparam.pmax=repmat([F_max, F_max, F_max, 2*pi 2*pi, 2*pi]', size(p_uparam.Ifree, 1), 1);



%-------------------------------------------------------------------------------
% Definition of p_ocp
%-------------------------------------------------------------------------------
p_ocp.Q=diag([1e8;1e8;1e8;1e1;1e1;1e1]);
p_ocp.R=diag([1e1; 1e1; 1e1; 1; 1; 1])*1;
p_ocp.Rdu=diag([1e4; 1e4; 1e4; 1e4; 1e4; 1e4])*1e-3;
p_ocp.rd=[2, 1, 30/180*pi]';
p_ocp.dF_max = dF_max;
p_ocp.da_max = da_max;
% p_ocp.theta_max=0.0015;
% p_ocp.thetap_max=2*pi/30;
%-------------------------------------------------------------------------------
% Create the param strcurure 
%-------------------------------------------------------------------------------
[param,flag,message,teval]=create_solution(p_ode,p_uparam,p_ocp,0);%1表示重新编译， 0表示不编译直接运行
%-------------------------------------------------------------------------------
% Closed-loop simulation
%-------------------------------------------------------------------------------
tsim=400;param.Nev=1300;
[tt,xx,uu,tt_exec,ntsim]=initialize(tsim,param);
rrd=zeros(ntsim,3);
param=update_trust_region_parameters(param,[2,0.5]);
param.ode.rk_order=4;
subset=[1];
du = zeros(length(uu(:, 1))-1, p_uparam.nu);
for i=1:length(tt)-1
%     disp(i/ntsim);
    i
%     param=user_sim(tt,i,param);
    [param,u,u_sol,tt_exec(i)]=pdf_mpc(xx(i,:)',param);
    uu(i,:)=u';
    rrd(i, :)=param.ocp.rd;
%     u1 = [u(1), u(3), u(5), u(7), u(2), u(4), u(6), u(8)]';
    xx(i+1,:)=one_step(xx(i,:)', u, p_ode);
end
rrd(i+1)=rrd(i);
for i = 1 : length(tt)-1
    du(i, :) = uu(i+1, :) - uu(i, :);
end
uu = uu(1:end-1, :);
du = du(1:end-1, :);

%-------------------------------------------------------------------------------
user_plot;
%-------------------------------------------------------------------------------
