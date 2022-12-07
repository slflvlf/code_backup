%-------------------------------------------------------------------------------
%pdf_mpc package: Example 1 - Definition of the user_control_profile map
% u_profile是控制序列， Np行，nu列
% 一般这个子程序不需要改
%-------------------------------------------------------------------------------
function u_profile=user_control_profile(p,p_ode,p_uparam)
    
    u_profile=reshape(p_uparam.R * p, p_uparam.nu, p_uparam.Np);
   
    u_profile = u_profile';

end
%-------------------------------------------------------------------------------