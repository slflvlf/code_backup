%-------------------------------------------------------------------------------
% pdf_mpc package: Example 1 - Definition of the user_ocp map
%-------------------------------------------------------------------------------
function [J,g]=user_ocp(xx,uu,p_ode,p_uparam,p_ocp)
J=0;
dF_max = p_ocp.dF_max;
da_max = p_ocp.da_max;
du_max = repmat([dF_max, dF_max, dF_max, da_max, da_max, da_max]', 2, 1);
xd = repmat([p_ocp.rd; zeros(3, 1)], 2, 1);
u_last = p_ocp.u_last;

h1_du = zeros(p_uparam.Np, 1);
h2_du = zeros(p_uparam.Np, 1);
for i=1:p_uparam.Np
    if i==1
%         du=uu(1, :)'-p_ode.u0;
        du=uu(1, :)'-u_last;
    else
        du=uu(i, :)'-uu(i-1, :)';
    end

    h1_du(i, 1) = max(du - du_max);
    h2_du(i, 1) = max(-du - du_max);
    
    e = xx(i+1,:)' - xd;%traking error


    J = J + (e' * p_ocp.Q * e + uu(i, :) * p_ocp.R * uu(i, :)' + du' * p_ocp.Rdu * du);
end

h1 = max(h1_du);
h2 = max(h2_du);

g = max([h1; h2]);

% h1=max(xx(:,3)-p_ocp.theta_max);
% h2=max(-xx(:,3)-p_ocp.theta_max);
% h3=max(xx(:,4)-p_ocp.thetap_max);
% h4=max(-xx(:,4)-p_ocp.thetap_max);
% g=max([h1;h2;h3;h4]);
% g = [];
end
%-------------------------------------------------------------------------------