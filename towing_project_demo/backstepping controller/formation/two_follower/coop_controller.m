function [tauc1, tauc2] = coop_controller(s1, s2, Kc1, Kc2)
    tauc1 = -Kc1 * 1 * (s1 - s2);
    tauc2 = -Kc2 * 1 * (s2 - s1);
    
end