function [Gx, Gmv] = planning_Cost_Gradient_Fcn(stage, x, u)
%PLANNING_COST_GRADIENT_FCN 此处显示有关此函数的摘要
%   此处显示详细说明
Gx = zeros(6, 1);
Gmv = [ones(4, 1); zeros(4, 1)];
end

