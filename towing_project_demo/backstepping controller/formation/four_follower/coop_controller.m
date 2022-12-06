function [tauc1, tauc2, tauc3, tauc4] = coop_controller(s1, s2, s3, s4, Kc1, Kc2, Kc3, Kc4)

% function [tauc1, tauc2] = coop_controller(s1, s2, Kc1, Kc2)
% 李博论文里的一致性
% 四条船
    %拓扑结构2（各船之间相连）
    tauc1 = -Kc1 * 1 * (s1 - s2) - Kc1 * 1 * (s1 - s4) - Kc1 * 1 * (s1 - s3);
    tauc2 = -Kc2 * 1 * (s2 - s1) - Kc2 * 1 * (s1 - s3) - Kc2 * 1 * (s2 - s4);
    tauc3 = -Kc3 * 1 * (s3 - s2) - Kc3 * 1 * (s3 - s4) - Kc3 * 1 * (s3 - s1);
    tauc4 = -Kc4 * 1 * (s4 - s1) - Kc4 * 1 * (s4 - s3) - Kc4 * 1 * (s4 - s2);
    
%     %拓扑结构1（只与相邻的船舶相连）
%     tauc1 = -Kc1 * 1 * (s1 - s2) - Kc1 * 1 * (s1 - s4);
%     tauc2 = -Kc2 * 1 * (s2 - s1) - Kc2 * 1 * (s1 - s3);
%     tauc3 = -Kc3 * 1 * (s3 - s2) - Kc3 * 1 * (s3 - s4);
%     tauc4 = -Kc4 * 1 * (s4 - s1) - Kc4 * 1 * (s4 - s3);
end