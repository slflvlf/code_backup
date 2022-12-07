syms x1t(t) y1t(t) fai1t(t) u1t(t) v1t(t) r1t(t)
syms x1 y1 fai1 u1 v1 r1
syms x2t(t) y2t(t) fai2t(t) u2t(t) v2t(t) r2t(t)
syms x2 y2 fai2 u2 v2 r2

syms f11 f12 f13 f14 alpha11 alpha12 alpha13 alpha14 
syms f21 f22 f23 f24 alpha21 alpha22 alpha23 alpha24 
syms tau_env11 tau_env12 tau_env13
syms tau_env21 tau_env22 tau_env23

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

statet = {x1t(t) y1t(t) fai1t(t) u1t(t) v1t(t) r1t(t)...
             x2t(t) y2t(t) fai2t(t) u2t(t) v2t(t) r2t(t)};
         
state = {x1 y1 fai1 u1 v1 r1 x2 y2 fai2 u2 v2 r2};

state_diff = {diff(x1t(t), t), diff(y1t(t), t), diff(fai1t(t), t), ...
    diff(u1t(t), t), diff(v1t(t), t), diff(r1t(t), t),...
    diff(x2t(t), t), diff(y2t(t), t), diff(fai2t(t), t), ...
    diff(u2t(t), t), diff(v2t(t), t), diff(r2t(t), t)};

% state_dot = { x1dot y1dot fai1dot u1dot v1dot r1dot ...
%     x2dot y2dot fai2dot u2dot v2dot r2dot};

R1 = [cos(fai1t), -sin(fai1t), 0; sin(fai1t), cos(fai1t), 0; 0, 0, 1];
R2 = [cos(fai2t), -sin(fai2t), 0; sin(fai2t), cos(fai2t), 0; 0, 0, 1];

% 这里两条船一样
B1 = [cos(alpha11), cos(alpha12), cos(alpha13), cos(alpha14);...
    sin(alpha11), sin(alpha12), sin(alpha13) sin(alpha14);...
    lx1*sin(alpha11)-ly1*cos(alpha11), lx2*sin(alpha12)-ly2*cos(alpha12), ...
    lx3*sin(alpha13)-ly3*cos(alpha13), lx4*sin(alpha14)-ly4*cos(alpha14)];

B2 = [cos(alpha21), cos(alpha22), cos(alpha23), cos(alpha24);...
    sin(alpha21), sin(alpha22), sin(alpha23) sin(alpha24);...
    lx1*sin(alpha21)-ly1*cos(alpha21), lx2*sin(alpha22)-ly2*cos(alpha22), ...
    lx3*sin(alpha23)-ly3*cos(alpha23), lx4*sin(alpha24)-ly4*cos(alpha24)];;

M = [M11, 0, 0; 0, M22, 0; 0, 0, M33];

C1 = [0, 0, -M(2, 2)*v1t; 0, 0, M(1, 1)*u1t;...
    -M(2, 2)*v1t, M(1, 1)*u1t, 0];
C2 = [0, 0, -M(2, 2)*v2t; 0, 0, M(1, 1)*u2t;...
    -M(2, 2)*v2t, M(1, 1)*u2t, 0];

D1 = [d11, 0, 0; 0, d22, 0; 0, 0, d33];
D2 = D1;

V1 = [u1t(t); v1t(t); r1t(t)];
V2 = [u2t(t); v2t(t); r2t(t)];

f(1:3) = R1 * V1;

% 加入可测扰动
f(4:6) = inv(M) * (B1 * [f11; f12; f13; f14] - C1 * V1 - D1 * V1...
    + [tau_env11; tau_env12; tau_env13]);

f(7:9) = R2 * V2;

f(10:12) = inv(M) * (B2 * [f21; f22; f23; f24] - C2 * V2 - D2 * V2...
    + [tau_env21; tau_env22; tau_env23]);

f = subs(f, [M11 M22 M33 d11 d22 d33 lx1 ly1 lx2 ly2 lx3 ly3 lx4 ly4], paramVal);

f = subs(f, statet, state);

f = simplify(f);

A = jacobian(f, [state{:}]);

control = [f11, f12, f13, f14, alpha11, alpha12, alpha13, alpha14, ...
           f21, f22, f23, f24, alpha21, alpha22, alpha23, alpha24,...
           tau_env11, tau_env12, tau_env13, tau_env21, tau_env22, tau_env23];
       

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