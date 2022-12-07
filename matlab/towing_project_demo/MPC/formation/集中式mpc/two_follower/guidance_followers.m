function [pd1, pd2] = guidance_followers(pd0)

% function [pd1, pd2, pd3, pd4, dot_pd1, dot_pd2, dot_pd3, dot_pd4,...
%     ddot_pd1, ddot_pd2, ddot_pd3, ddot_pd4] = guidance_followers(pd0, dot_pd0)

% 根据领航者的轨迹，来确定跟随者的轨迹


% global delta10 delta20 delta30 delta40;
global delta10 delta20;
global length_init;

pd1 = BtoG(pd0, delta10);
pd2 = BtoG(pd0, delta20);
% pd3 = BtoG(pd0, delta30);
% pd4 = BtoG(pd0, delta40);

% 体坐标下，拖轮相对母船的初始角度（各自重心位置）
beta1 = atan2(delta10(2), delta10(1));
beta2 = atan2(delta20(2), delta20(1));
% beta3 = atan2(delta30(2), delta30(1));
% beta4 = atan2(delta40(2), delta40(1));

% % 大地坐标系下，拖轮的速度要考虑母船的旋转角速度
% dot_pd1 = dot_pd0 + [dot_pd0(3) * length_init(1) * cos(pi/2 + pd0(3) + beta1);...
%                     dot_pd0(3) * length_init(1) * sin(pi/2 + pd0(3)+ beta1); 0];
%                 
% dot_pd2 = dot_pd0 + [dot_pd0(3) * length_init(2) * cos(pi/2 + pd0(3)+ beta2);...
%                     dot_pd0(3) * length_init(2) * sin(pi/2 + pd0(3)+ beta2); 0];
% 
% dot_pd3 = dot_pd0 + [dot_pd0(3) * length_init(3) * cos(pi/2 + pd0(3) + beta3);...
%                     dot_pd0(3) * length_init(3) * sin(pi/2 + pd0(3)+ beta3); 0];
%                 
% dot_pd4 = dot_pd0 + [dot_pd0(3) * length_init(4) * cos(pi/2 + pd0(3)+ beta4);...
%                     dot_pd0(3) * length_init(4) * sin(pi/2 + pd0(3)+ beta4); 0];
% 
% %加速度不考虑                
% [ddot_pd1, ddot_pd2, ddot_pd3, ddot_pd4] = deal(zeros(3, 1));

end



function p = BtoG(p0, delta)

    R = rotate_matrix(p0);
    p = R * delta + p0;
end


function R= rotate_matrix(p)
    fai = p(3);
    R = [cos(fai), -sin(fai), 0;
        sin(fai), cos(fai), 0;
        0, 0, 1];
end
