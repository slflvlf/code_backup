%-------------------------------------------------------------------------------
% pdf_mpc package: Example 1 - Definition of the user_ocp map
%-------------------------------------------------------------------------------
function [J, g]=user_ocp(xx, uu, p_ode, p_uparam, p_ocp)
J=0;
F_max = p_ocp.F_max;
a_max = p_ocp.a_max;
uplus_min = repmat([0, 0, 0, -a_max, -a_max, -a_max]', p_ode.Nship, 1);
uplus_max = repmat([F_max, F_max, F_max, a_max, a_max, a_max]', p_ode.Nship, 1);
% du_max = repmat([100, 100, 100, 20/180*pi, 20/180*pi, 20/180*pi]', p_ode.Nship, 1);
xd_his = p_ocp.xd_his;
u_last = p_ocp.u_last;
% xd=[p_ocp.rd;0;0;0];


h1_du = zeros(p_uparam.Np, 1);
h2_du = zeros(p_uparam.Np, 1);


for i=1:p_uparam.Np
    u_plus = zeros(6*p_ode.Nship, 1);
    for j = 1 : i
        u_plus = u_plus + uu(j, :)';
    end
    u_new = u_last + u_plus;
        

    
    h1_du(i, 1) = max(u_new - uplus_max);
    h2_du(i, 1) = max(-u_new + uplus_min);
    

    
%     e = xx(i+1,:)' - xd;
    %xd_his(1, :)代表传进来一个点，xd_his(i, :)代表传进来一条轨迹
    % 编队好像传进去一个点稳定性更高一点
    e = xx(i+1, :)' - xd_his(1, :)';
    
    
    J = J + (e' * p_ocp.Q * e + uu(i, :) * p_ocp.Rdu * uu(i, :)' + u_new' * p_ocp.R * u_new);
%     J = J + (e' * p_ocp.Q * e + uu(i, :) * p_ocp.R * uu(i, :)');
%     J = J + e' * p_ocp.Q * e ;
end
h1 = max(h1_du);
h2 = max(h2_du);
g = max([h1; h2]);


% g = -1; 
end
%-------------------------------------------------------------------------------