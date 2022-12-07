function [tau, integral_error] = pid_controller(pd, dot_pd, p, v, M, D, integral_error)
% M包括Ma
% tau  /KN
% integral_error: 误差累积
    R = rotate_matrix(p);
    p_error = R' * ( pd - p );
    integral_error = integral_error + p_error;
    
    error_max = 100;
    
    for i = 1 : 3
        if integral_error(i) < -error_max
            integral_error(i) = -error_max;
        end
        if integral_error(i) > error_max
            integral_error(i) = error_max; 
        end
    end
    
    T = 120; %固有周期设置成120-250s
    wn = 2*pi/T;  eta_d = 0.7;  %固有频率和临界阻尼系数
    
    [Kp, Kd, Ki] = pid_param(M, D, wn, eta_d);

    tau = Kp * p_error + Ki * integral_error + Kd * (R' * dot_pd - v);
    
    tau = tau / 1e3;
    

end



function R = rotate_matrix(p)
    fai = p(3);
    R = [cos(fai),  -sin(fai),  0;
         sin(fai),  cos(fai),   0;
         0,          0,         1];   
end

function [Kp, Kd, Ki] = pid_param(M, D, wn, eta_d)
    Kp = zeros(3, 3);   Kd = Kp;    Ki = Kp;
    for i = 1 : 3
        Kp(i, i) = wn^2 * M(i, i);
        Kd(i, i) = 2 * eta_d * sqrt( M(i, i) * Kp(i, i) ) - D(i, i);
        Ki(i, i) = 0.01 * Kp(i, i);
    end
end
        

