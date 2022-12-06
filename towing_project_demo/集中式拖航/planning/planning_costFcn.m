function J = planning_costFcn(stage, x, u)
%     J = sum(u(1:4));
    J = u(1) + u(2) + u(3) + u(4);
%     J  = sum(u(1:4));
%     Wmv = zeros(8, 8);
%     for i = 1 : 4
%         Wmv(i, i) = 1;
%     end
% J =  u'*Wmv*u;
end