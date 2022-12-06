function [p, v] = ship_dynamic_solver(p0, v0, tau, M, D)
    tau = tau * 1e3; %输入的是KN
    
    y0 = [p0; v0];
    [t, y] = ode45(@(t, y) ship_dynamic(t, y, tau, M, D), [0: 0.5: 1], y0);
    
    p = y(end, 1:3)';
    v = y(end, 4:6)';
end



function dydt = ship_dynamic(t, y, tau, M, D)

%     global MM1 D1 
    R = rotate_matrix(y(3));
    C = C_matrix(M, y(4:6));
    dydt = zeros(6, 1);
    dydt(1:3) = R * y(4:6);
    dydt(4:6) = inv(M) * (tau - C * y(4:6) - D * y(4:6));
 
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