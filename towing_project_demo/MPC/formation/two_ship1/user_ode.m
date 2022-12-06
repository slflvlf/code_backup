%-------------------------------------------------------------------------------
% pdf_mpc package: Example 1 - Definition of the user_ode map
%-------------------------------------------------------------------------------
function xdot = user_ode(x, u, p_ode)


    M = p_ode.M;
    D = p_ode.D;
    thrust_config = p_ode.thrust_config;
    
    % 船的个数
    Nship = p_ode.Nship;
    
    xdot = zeros(6*Nship, 1);
    
%     for i = 1 : Nship 
%         R1 = rotate_matrix(x(6*(i-1)+3));
%         C1 = C_matrix(M, x(6*(i-1)+4 : 6*(i-1)+6));
%         B1 = thrusters_configuration(u(6*(i-1)+4 : 6*(i-1)+6), thrust_config);
%         xdot(6*(i-1)+1 : 6*(i-1)+3, 1) = R1 * x(6*(i-1)+4 : 6*(i-1)+6);
%         xdot(6*(i-1)+4 : 6*(i-1)+6, 1) = inv(M) * (B1 * u(6*(i-1)+1 : 6*(i-1)+3) * 1e3 - C1 * x(6*(i-1)+4 : 6*(i-1)+6) - D * x(6*(i-1)+4 : 6*(i-1)+6));
%     end
        
        
    R1 = rotate_matrix(x(3));
    C1 = C_matrix(M, x(4:6));
    B1 = thrusters_configuration(u(4:6), thrust_config);
    xdot(1:3, 1) = R1 * x(4:6);
    xdot(4:6, 1) = inv(M) * (B1 * u(1:3) * 1e3 - C1 * x(4:6) - D * x(4:6));
    
    % 船2
    R2 = rotate_matrix(x(9));
    C2 = C_matrix(M, x(10:12));
    B2 = thrusters_configuration(u(10:12), thrust_config);
    xdot(7:9, 1) = R2 * x(4:6);
    xdot(10:12, 1) = inv(M) * (B2 * u(7:9) * 1e3 - C2 * x(10:12) - D * x(10:12));

    
%     return

%-------------------------------------------------------------------------------
end





function R = rotate_matrix(fai)
    
    R = [cos(fai),  -sin(fai),  0;
         sin(fai),  cos(fai),   0;
         0,          0,         1];   
end

function C = C_matrix(M, v)
    C = [0, 0, -M(2, 2) * v(2) - M(2, 3) * v(3);
        0, 0, M(1, 1) * v(1);
        M(2, 2) * v(2) + M(3, 2) * v(3), -M(1, 1) * v(1), 0];
end

function T = thrusters_configuration(a,L)
% function thruster_configuration is used to obtain the configuration
% matrix
N_Col = length(a);

if N_Col ~= length(L)
    error('the length of a and L do not match');
end

T = zeros(3,N_Col);   % initialization

for i = 1 : N_Col
    T(1,i) = cos(a(i));
    T(2,i) = sin(a(i));
    T(3,i) = L(i,1)*sin(a(i)) - L(i,2)*cos(a(i));
end

end

