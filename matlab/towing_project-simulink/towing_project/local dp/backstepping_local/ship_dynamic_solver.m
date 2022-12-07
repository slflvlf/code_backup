function [p_next, v_next] = ship_dynamic_solver(p0, v0, tau)

    y0 = [p0; v0];
    [t, y] = ode45(@(t, y) ship_dynamic(t, y, tau), [0:0.2: 1], y0);
    
    p_next = y(end, 1:3)';
    v_next = y(end, 4:6)';
end



function dydt = ship_dynamic(t, y, tau)

    global MM1 D1 
    R = rotate_matrix(y(3));
    C = C_matrix(MM1, y(4:6));
    dydt = zeros(6, 1);
    dydt(1:3) = R * y(4:6);
    dydt(4:6) = inv(MM1) * (tau - C * y(4:6) - D1 * y(4:6));
 
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