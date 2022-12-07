clear all; clc;
% two ships tracking a same trajectory.


%-------------------------------------------------------------------------------
% Definition of p_ode
%-------------------------------------------------------------------------------
p_ode.Nship = 2;  %number of ships
M1 = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
Ma1 = [7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5, 8.77e6];
MM1 = M1 + Ma1;
D1 = [MM1(1,1)/100, 0, 0; 0, MM1(2,2)/40, 0; 0, 0, MM1(3,3)/20];
p_ode.tau = 1;%步长
p_ode.rk_order = 4;%积分阶数

%initial state (p1, v1, p2, v2) ->12*1, p: position; v: velocity;
p_ode.x0 = repmat([0; 0; 0; 0; 0; 0], p_ode.Nship, 1);

% initial control (f1, a1, f2, a2) ->12*1, 
%f1, a1: three thrusters and angles of ship1
p_ode.u0=repmat([120, 60, 60, 180*pi/180, 0*pi/180, 0*pi/180]', p_ode.Nship, 1);

% two identical ships' parameters
p_ode.M = MM1;
p_ode.D = D1;
p_ode.thrust_config = [-5.02 0; 11.956 2.7; 11.956 -2.7];


%-------------------------------------------------------------------------------
% Definition of p_uparam
%-------------------------------------------------------------------------------
p_uparam.nu = 6 * p_ode.Nship;
p_uparam.Np = 20;
p_uparam.Ifree = [1;2;3;10;];
p_uparam.R = compute_R(p_uparam.Ifree, p_uparam.Np, p_uparam.nu);
p_uparam.np = size(p_uparam.R,2);
p_uparam.p = repmat(p_ode.u0, size(p_uparam.Ifree, 1), 1);


F_max = 400; % upper limit of thrusters (0 ~ 400KN)
dF_max = 100; % change rate of thrusters (-100 ~ 100 KN/s)
da_max = 10/180*pi;% change rate of azimuth angles (-10 ~ 10 degree/s)

% the azimuth angles limit (-360 ~ 360 degree/s)
p_uparam.pmin = repmat(repmat([0, 0, 0, -2*pi -2*pi, -2*pi]', 2, 1), size(p_uparam.Ifree, 1), 1);
p_uparam.pmax = repmat(repmat([F_max, F_max, F_max, 2*pi 2*pi, 2*pi]', 2, 1), size(p_uparam.Ifree, 1), 1);



%-------------------------------------------------------------------------------
% Definition of p_ocp
%-------------------------------------------------------------------------------
p_ocp.Q=diag(repmat([1e8;1e8;2e10;0;0;0], 2, 1));
p_ocp.R=diag(repmat([1e3; 1e3; 1e3; 0; 0; 0], 2, 1))*1;
p_ocp.Rdu=diag(repmat([1e4; 1e4; 1e4; 1e4; 1e4; 1e4], 2, 1))*1e-3*0;

p_ocp.dF_max = dF_max;
p_ocp.da_max = da_max;

% rd: trajectory to be tracking
p_ocp.rd = zeros(3, 1); 

p_ocp.u_last = p_ode.u0;

%-------------------------------------------------------------------------------
% Create the param strcurure 
%-------------------------------------------------------------------------------
[param,flag,message,teval]=create_solution(p_ode,p_uparam,p_ocp,0);%1表示重新编译， 0表示不编译直接运行
%-------------------------------------------------------------------------------
% Closed-loop simulation
%-------------------------------------------------------------------------------
tsim=400;param.Nev=1300;
[tt,xx,uu,tt_exec,ntsim]=initialize(tsim,param);

param=update_trust_region_parameters(param,[1.0000001,0.99999999]);
param.ode.rk_order=4;

du = zeros(length(uu(:, 1))-1, p_uparam.nu);

% the ref_model function to generate a trajectory tracked by the mpc
% controller, and two ships use one same trajectory.
T = 30; %time constant to control the trajectory.
Rr = [200, 40, 30/180*pi]'; % the destination of the trajectory
p0 = zeros(3, 1); % the initial point of the trajectory.
rrd=zeros(ntsim,3);% the record of the trajectory
for i=1:length(tt)-1

    i
    % ref_model function to generate the trajectory point in real time
    [pd, dot_pd, ddot_pd] = ref_model(T, i, p0, Rr);
    param.ocp.rd = pd;

    [param,u,u_sol,tt_exec(i)] = pdf_mpc(xx(i,:)', param);
    uu(i,:) = u';
    rrd(i, :) = pd';
    param.ocp.u_last = u;

    xx(i+1,:) = one_step(xx(i,:)', u, p_ode);
end
rrd(i+1) = rrd(i);

% get the change rate of the control
for i = 1 : length(tt)-1
    du(i, :) = uu(i+1, :) - uu(i, :);
end
uu = uu(1:end-1, :);
du = du(1:end-1, :);
rrd = rrd(1: end-1, :);
%-------------------------------------------------------------------------------
user_plot;
%-------------------------------------------------------------------------------
