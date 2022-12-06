%-------------------------------------------------------------------------------
% pdf_mpc package: Example 1 - Definition of the user_ocp map
%-------------------------------------------------------------------------------
function [J, g]=user_ocp(xx, uu, p_ode, p_uparam, p_ocp)
J=0;
dF_max = p_ocp.dF_max;
da_max = p_ocp.da_max;
du_max = repmat([dF_max, dF_max, dF_max, da_max, da_max, da_max]', p_ode.Nship, 1);
% du_max = repmat([100, 100, 100, 20/180*pi, 20/180*pi, 20/180*pi]', p_ode.Nship, 1);
xd_his = p_ocp.xd_his;
u_last = p_ocp.u_last;
% xd=[p_ocp.rd;0;0;0];
dff_max = ones(6, 1) * dF_max;
daa_max = ones(6, 1) * da_max;

h1_du = zeros(p_uparam.Np, 1);
h2_du = zeros(p_uparam.Np, 1);

hff1_du = zeros(p_uparam.Np, 1);
hff2_du = zeros(p_uparam.Np, 1);
haa1_du = zeros(p_uparam.Np, 1);
haa2_du = zeros(p_uparam.Np, 1);

uu
xx
for i=1:p_uparam.Np
    if i==1
        du = uu(1, :)' - p_ode.u0;%这里没有问题，u0 = u_last, p_ode.u0在这里是实时更新的
%         du = uu(1, :)' - u_last;
    else
        du = uu(i, :)' - uu(i-1, :)';
    end
    duff = [du(1:3); du(7:9)];
    duaa = [du(4:6); du(10:12)];
    
    hff1_du(i, 1) = max(duff - dff_max);
    hff2_du(i, 1) = max(-duff - dff_max);
    haa1_du(i, 1) = max(duaa - daa_max);
    haa2_du(i, 1) = max(-duaa - daa_max);
    
    h1_du(i, 1) = max(du - du_max);
    h2_du(i, 1) = max(-du - du_max);
    

    
%     e = xx(i+1,:)' - xd;
    %xd_his(1, :)代表传进来一个点，xd_his(i, :)代表传进来一条轨迹
    % 编队好像传进去一个点稳定性更高一点
    e = xx(i+1, :)' - xd_his(1, :)';
    
    
%     J = J + (e' * p_ocp.Q * e + uu(i, :) * p_ocp.R * uu(i, :)' + du' * p_ocp.Rdu * du);
    J = J + (e' * p_ocp.Q * e + uu(i, :) * p_ocp.R * uu(i, :)');
%     J = J + e' * p_ocp.Q * e ;
end
% h1 = max(h1_du);
% h2 = max(h2_du);
% g = max([h1; h2]);

h3 = max(hff1_du);
h4 = max(hff2_du);
h5 = max(haa1_du);
h6 = max(haa2_du);
g = max([h3; h4; h5; h6]);
g = max([ h5; h6]);



% h1=max(xx(:,3)-p_ocp.theta_max);
% h2=max(-xx(:,3)-p_ocp.theta_max);
% h3=max(xx(:,4)-p_ocp.thetap_max);
% h4=max(-xx(:,4)-p_ocp.thetap_max);
% g=max([h1;h2;h3;h4]);
% g = -1; 
end
%-------------------------------------------------------------------------------