%-------------------------------------------------------------------------------
% pdf_mpc package: Example 1 - Definition of the user_ode map
%-------------------------------------------------------------------------------
function xdot = user_ode(x, u, p_ode)


    M = p_ode.M;
    D = p_ode.D;
    thrust_config = p_ode.thrust_config;
    
    R = rotate_matrix(x(3));
    C = C_matrix(M, x(4:6));
    
    B = thrusters_configuration(u(5:8), thrust_config);
    
    
%     xdot = zeros(6, 1);
%     xdot(1:3, 1) = R * x(4:6);
%     xdot(4:6, 1) = inv(M) * (B*u(1:4)*1e3 - C * x(4:6) - D * x(4:6));
%     return


    xdot = zeros(6, 1);
    xdot(1:3, 1) = rotate_matrix(x(3)) * x(4:6);
    xdot(4:6, 1) = inv(M) * (thrusters_configuration(u(5:8), thrust_config)*u(1:4)*1e3 - C * x(4:6) - D * x(4:6));
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

