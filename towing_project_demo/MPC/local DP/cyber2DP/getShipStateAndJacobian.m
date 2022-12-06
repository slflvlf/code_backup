clear all;
clc;
syms xt(t) yt(t) fait(t) ut(t) vt(t) rt(t)
syms x y fai u v r

syms f1 f2 f3 f4 alpha1 alpha2 alpha3 alpha4 
syms tau_env1 tau_env2 tau_env3
syms M11 M22 M33   %包含附加质量
syms d11 d22 d33
syms lx1 ly1 lx2 ly2 lx3 ly3 lx4 ly4

%论文中的无人船, 不是cybership 2
% 这里应该存在M23的，因为船舶前后不对称，可能比较小被忽略了
M11Val = 21.67;
M22Val = 39.08;
M33Val = 14.56;

d11Val = 23.52;
d22Val = 22.32;
d33Val = 3.762;

lx1Val = 0.4; ly1Val = 0.2;
lx2Val = -0.4; ly2Val = 0.2;
lx3Val = -0.4; ly3Val = -0.2;
lx4Val = 0.4; ly4Val = -0.2;

paramVal = [M11Val M22Val M33Val d11Val d22Val d33Val...
    lx1Val ly1Val lx2Val ly2Val lx3Val ly3Val lx4Val ly4Val];

statet = {xt(t) yt(t) fait(t) ut(t) vt(t) rt(t)};

state = {x y fai u v r};
state_diff = {diff(xt(t), t), diff(yt(t), t), diff(fait(t), t), ...
    diff(ut(t), t), diff(vt(t), t), diff(rt(t), t)};
state_dot = { xdot ydot faidot udot vdot rdot};

R = [cos(fait), -sin(fait), 0; sin(fait), cos(fait), 0; 0, 0, 1];

B = [cos(alpha1), cos(alpha2), cos(alpha3), cos(alpha4);...
    sin(alpha1), sin(alpha2), sin(alpha3) sin(alpha4);...
    lx1*sin(alpha1)-ly1*cos(alpha1), lx2*sin(alpha2)-ly2*cos(alpha2), ...
    lx3*sin(alpha3)-ly3*cos(alpha3), lx4*sin(alpha4)-ly4*cos(alpha4)];

M = [M11, 0, 0; 0, M22, 0; 0, 0, M33];

C = [0, 0, -M(2, 2)*vt; 0, 0, M(1, 1)*ut;...
    -M(2, 2)*vt, M(1, 1)*ut, 0];

D = [d11, 0, 0; 0, d22, 0; 0, 0, d33];

V = [ut(t); vt(t); rt(t)];

f(1:3) = R * V;

% 加入可测扰动
f(4:6) = inv(M) * (B * [f1; f2; f3; f4] - C * V - D * V...
    + [tau_env1; tau_env2; tau_env3]);

f = subs(f, [M11 M22 M33 d11 d22 d33 lx1 ly1 lx2 ly2 lx3 ly3 lx4 ly4], paramVal);

f = subs(f, statet, state);

f = simplify(f);

A = jacobian(f, [state{:}]);

control = [f1, f2, f3, f4, alpha1, alpha2, alpha3, alpha4, ...
    tau_env1, tau_env2, tau_env3];
% control = [f1, f2, f3, f4, alpha1, alpha2, alpha3, alpha4];
B = jacobian(f, control);

% Create ShipStateFcn.m
matlabFunction(transpose(f),'File','ShipStateFcn',...
    'Vars',{transpose([state{:}]),transpose(control)})

% Create ShipStateJacobianFcn.m 
matlabFunction(A, B,'File','ShipStateJacobianFcn',...
    'Vars',{transpose([state{:}]),transpose(control)})


%Clear symbolic variables
clear