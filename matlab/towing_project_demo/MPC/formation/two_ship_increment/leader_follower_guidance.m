function [pd0, vd0, pd1, vd1, pd2, vd2] = leader_follower_guidance(current_time, guidance_param)
    T = guidance_param.T;
    p0_init = guidance_param.p0_init;
    leader_destination = guidance_param.leader_destination;
    delta10 = guidance_param.delta10;
    delta20 = guidance_param.delta20;
    
    [pd0, dot_pd, vd0, ~] = ref_model1(T, current_time, p0_init, leader_destination);
    pd1 = BodytoGlobal(pd0, delta10);
    pd2 = BodytoGlobal(pd0, delta20);
    vd1 = vd0;
    vd2 = vd0;
    
    
end




function [pd, dot_pd, vd,  ddot_pd] = ref_model1(T, t, p0, r)

    pd = r * (1 -  exp(-t / T)) + p0;
    
    R = rotate_matrix(pd(3));

    % 轨迹的一阶微分
    dot_pd = r / T * exp(-t / T);
    
    % 轨迹的速度（体坐标下的），body-fixed coordinate system
    vd = R' * dot_pd;
    
    % 轨迹的二阶微分
    ddot_pd = -r / T / T * exp(-t / T);

end


function R = rotate_matrix(fai)
    
    R = [cos(fai),  -sin(fai),  0;
         sin(fai),  cos(fai),   0;
         0,          0,         1];   
end


% 目标点从体坐标转向大地坐标系

function p = BodytoGlobal(p0, delta)
    R = rotate_matrix(p0(3));
    p = R * delta + p0;
end


