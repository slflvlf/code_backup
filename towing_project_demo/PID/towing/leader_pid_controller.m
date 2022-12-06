function [pd0] = leader_pid_controller(pd0_ref, dot_pd0_ref, p0, v0, Kp, Ki, Kd, time_step)
    
    persistent int_error;
    if isempty(int_error)
        int_error = zeros(3, 1);
    end
    error = pd0_ref - p0;
    int_error = int_error + time_step * error;
    R = rotate_matrix(p0);
    dot_p0 = R * v0;
    
    
    pd0 = pd0_ref + Kp * error + Ki * int_error + Kd * (dot_pd0_ref - dot_p0);
    
end

function R = rotate_matrix(p)
    fai = p(3);
    R = [cos(fai),  -sin(fai),  0;
         sin(fai),  cos(fai),   0;
         0,          0,         1];   
end