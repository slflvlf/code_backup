function [pd, dot_pd, ddot_pd] = ref_model_2ord(r, pd, dot_pd, wn, eta, vmax, time_step)
% velocity saturation

ddot_pd = wn.^2 .* (r - pd) - 2 * eta .* wn .* dot_pd;

dot_pd = dot_pd + ddot_pd * time_step;
for i = 1 : 3
    if abs(dot_pd(i)) > vmax(i)
        dot_pd(i) = sign(dot_pd(i)) * vmax(i);
    end
end

pd = pd + dot_pd * time_step;

end
% x = 0;
% v = 0;
% r3 = 10;
% y3 = zeros(N+1,2);
% for i=1:N+1
%    y3(i,:) = [x v];   
%    v_dot = w^2*(r3-x) - 2*z*w*v;
%    x_dot = v;
%    v = v + h*v_dot;
%    if abs(v)>vmax       % saturation
%       v = sign(v)*vmax;
%    end
%    x = x + h*x_dot;
% end