function tau=pid_controller(pd, dot_pd, p, v)
    global integ_var M1 Ma1 MM1 D1;
    p_error = pd-p;
    integ_var = integ_var + p_error;
    [Kp, Kd, Ki] = pid_param();
    R = rotate_matrix(p);
    tau = M1 * Kp * p_error + M1 * Ki * integ_var + M1 * Kd * (0 - v);
    tau = tau/1e3;
    
    
    imax = 100;
    for i = 1 : 3
        if integ_var(i) > imax
            integ_var(i) = imax;
        end
        if integ_var(i) < -imax
            integ_var(i) = -imax;
        end
    end

end



function R = rotate_matrix(p)
    fai = p(3);
    R = [cos(fai),  -sin(fai),  0;
         sin(fai),  cos(fai),   0;
         0,          0,         1];   
end

function [Kp, Kd, Ki] = pid_param()
    global integ_var M1 Ma1 MM1 D1;
    wn = 0.05;
    T = 2*pi/wn;
    eta_D = 0.7; %临界阻尼系数
    kp_x = wn^2 *(M1(1, 1)+Ma1(1, 1));
    kp_y = wn^2 *(M1(2, 2)+Ma1(2, 2));
    kp_psi = wn^2 *(M1(3, 3)+Ma1(3, 3));

    kd_x = 1.4*sqrt((M1(1, 1)+Ma1(1, 1))*kp_x)-D1(1, 1);
    kd_y = 1.4*sqrt((M1(2, 2)+Ma1(2, 2))*kp_y) - D1(2, 2);
    kd_psi = 1.4*sqrt((M1(3, 3)+Ma1(3, 3))*kp_psi) - D1(3, 3);


    ki_x = 0.01 * kp_x;
    ki_y = 0.01 * kp_y;
    ki_psi = 0.01 * kp_psi;

    %无因次化
    Kp = diag([kp_x/M1(1, 1), kp_y/M1(2, 2), kp_psi/M1(3, 3)]);
    Kd = diag([kd_x/M1(1, 1), kd_y/M1(2, 2), kd_psi/M1(3, 3)]);
    Ki = diag([ki_x/M1(1, 1), ki_y/M1(2, 2), ki_psi/M1(3, 3)]);


end