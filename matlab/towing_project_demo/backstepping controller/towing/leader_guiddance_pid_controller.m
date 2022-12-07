function [pd_ref, dot_pd_ref, ddot_pd_ref, ei_leader] = leader_guiddance_pid_controller(p0, v0, pd, dot_pd,pd_ref,...
    kp_ref, kd_ref, ki_ref, time_step, ei_leader)
    %% pid控制器，控制母船和轨迹之间的误差
    
    ei_leader = ei_leader + time_step * (pd - p0);
    
%     %积分变量的上下限
%     ei_leader_max = [50, 50, 50/180*pi];
%     for i = 1 : 3
%         if ei_leader(i) > ei_leader_max(i)
%             ei_leader(i) = ei_leader_max(i);
%         elseif ei_leader(i) < -ei_leader_max(i)
%             ei_leader(i) = -ei_leader_max(i);
%         end
%     end
    
    R = rotate_matrix(p0);
    pd_ref_next = kp_ref * (pd - p0) + kd_ref * (dot_pd - R*v0) + ki_ref * ei_leader + pd;
    
    dot_pd_ref = (pd_ref_next - pd_ref) / time_step;
    pd_ref = pd_ref_next;
    ddot_pd_ref = zeros(3, 1);
end

%% 旋转矩阵
function R= rotate_matrix(p)
    fai = p(3);
    R = [cos(fai), -sin(fai), 0;
        sin(fai), cos(fai), 0;
        0, 0, 1];
end
