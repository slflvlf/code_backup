syms xt(t) yt(t) fait(t) ut(t) vt(t) rt(t)
syms x y fai u v r

syms f1 f2 f3 alpha1 alpha2 alpha3
syms M Izz Ma11 Ma22 Ma33 Ma23   %包含附加质量
syms d11 d22 d33
syms lx1 ly1 lx2 ly2 lx3 ly3

MVal = 6.25e5;
IzzVal = 6.25e5*7.5^2;

Ma11Val = 7.58e4;
Ma22Val = 3.69e5;
Ma33Val =  8.77e6;
Ma23Val = -1.4e5;

d11Val = (MVal + Ma11Val)/100;
d22Val = (MVal + Ma22Val)/40;
d33Val = (IzzVal + Ma33Val)/20;

lx1Val = -5.02; ly1Val = 0;
lx2Val = 11.956; ly2Val = 2.7;
lx3Val = 11.956; ly3Val = -2.7;

paramVal = [MVal IzzVal Ma11Val Ma22Val Ma33Val Ma23Val d11Val d22Val d33Val...
    lx1Val ly1Val lx2Val ly2Val lx3Val ly3Val];

statet = {xt(t) yt(t) fait(t) ut(t) vt(t) rt(t)};
state = {x y fai u v r};
state_diff = {diff(xt(t), t), diff(yt(t), t), diff(fait(t), t), ...
    diff(ut(t), t), diff(vt(t), t), diff(rt(t), t)};
% state_dot = { xdot ydot faidot udot vdot rdot};

R = [cos(fait), -sin(fait), 0; sin(fait), cos(fait), 0; 0, 0, 1];

B = [cos(alpha1), cos(alpha2), cos(alpha3);...
    sin(alpha1), sin(alpha2), sin(alpha3);...
    -lx1*sin(alpha1)+ly1*cos(alpha1), -lx2*sin(alpha2)+ly2*cos(alpha2), ...
    -lx3*sin(alpha3)+ly3*cos(alpha3)];

MM = [M, 0, 0; 0, M, 0; 0, 0, Izz] + [Ma11, 0, 0; 0, Ma22, Ma23; 0, Ma23, Ma33];
C = [0, 0, -MM(2, 2)*vt-MM(2, 3)*rt; 0, 0, MM(1, 1)*ut;...
    -MM(2, 2)*vt-MM(2, 3)*rt, MM(1, 1)*ut, 0];
D = [d11, 0, 0; 0, d22, 0; 0, 0, d33];



V = [ut(t); vt(t); rt(t)];


f(1:3) = R * V;

f(4:6) = inv(MM) * (B * [f1; f2; f3] - C * V - D * V);

f = subs(f, [M Izz Ma11 Ma22 Ma33 Ma23 d11 d22 d33 lx1 ly1 lx2 ly2 lx3 ly3], paramVal);

f = subs(f, statet, state);

f = simplify(f);

A = jacobian(f,[state{:}]);

control = [f1, f2, f3, alpha1, alpha2, alpha3];
B = jacobian(f,control);

% Create QuadrotorStateFcn.m
matlabFunction(transpose(f),'File','ShipStateFcn',...
    'Vars',{transpose([state{:}]),transpose(control)})
% Create QuadrotorStateJacobianFcn.m 
matlabFunction(A, B,'File','ShipStateJacobianFcn',...
    'Vars',{transpose([state{:}]),transpose(control)})


%Clear symbolic variables
clear