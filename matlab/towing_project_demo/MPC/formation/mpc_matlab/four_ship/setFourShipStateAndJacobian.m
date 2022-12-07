function [] = setFourShipStateAndJacobian(M, D, L, runFlag)
if runFlag
syms x1t(t) y1t(t) fai1t(t) u1t(t) v1t(t) r1t(t);
syms x1 y1 fai1 u1 v1 r1;
syms x2t(t) y2t(t) fai2t(t) u2t(t) v2t(t) r2t(t);
syms x2 y2 fai2 u2 v2 r2;
syms x3t(t) y3t(t) fai3t(t) u3t(t) v3t(t) r3t(t);
syms x3 y3 fai3 u3 v3 r3;
syms x4t(t) y4t(t) fai4t(t) u4t(t) v4t(t) r4t(t);
syms x4 y4 fai4 u4 v4 r4;

syms f11 f12 f13 alpha11 alpha12 alpha13;
syms f21 f22 f23 alpha21 alpha22 alpha23;
syms f31 f32 f33 alpha31 alpha32 alpha33;
syms f41 f42 f43 alpha41 alpha42 alpha43;



statet = {x1t(t) y1t(t) fai1t(t) u1t(t) v1t(t) r1t(t)...
    x2t(t) y2t(t) fai2t(t) u2t(t) v2t(t) r2t(t)...
    x3t(t) y3t(t) fai3t(t) u3t(t) v3t(t) r3t(t)...
    x4t(t) y4t(t) fai4t(t) u4t(t) v4t(t) r4t(t)};

state = {x1 y1 fai1 u1 v1 r1...
    x2 y2 fai2 u2 v2 r2...
    x3 y3 fai3 u3 v3 r3...
    x4 y4 fai4 u4 v4 r4};

control = [f11 f12 f13 alpha11 alpha12 alpha13...
    f21 f22 f23 alpha21 alpha22 alpha23...
    f31 f32 f33 alpha31 alpha32 alpha33...
    f41 f42 f43 alpha41 alpha42 alpha43];


% state_diff = {diff(xt(t), t), diff(yt(t), t), diff(fait(t), t), ...
%     diff(ut(t), t), diff(vt(t), t), diff(rt(t), t)};

% state_dot = { xdot ydot faidot udot vdot rdot};

R1 = [cos(fai1t), -sin(fai1t), 0; sin(fai1t), cos(fai1t), 0; 0, 0, 1];
R2 = [cos(fai2t), -sin(fai2t), 0; sin(fai2t), cos(fai2t), 0; 0, 0, 1];
R3 = [cos(fai3t), -sin(fai3t), 0; sin(fai3t), cos(fai3t), 0; 0, 0, 1];
R4 = [cos(fai4t), -sin(fai4t), 0; sin(fai4t), cos(fai4t), 0; 0, 0, 1];

B1 = [cos(alpha11), cos(alpha12), cos(alpha13);...
    sin(alpha11), sin(alpha12), sin(alpha13);...
    L(1, 1)*sin(alpha11)-L(1, 2)*cos(alpha11), ...
    L(2, 1)*sin(alpha12)-L(2, 2)*cos(alpha12), ...
    L(3, 1)*sin(alpha13)-L(3, 2)*cos(alpha13)];

B2 = [cos(alpha21), cos(alpha22), cos(alpha23);...
    sin(alpha21), sin(alpha22), sin(alpha23);...
    L(1, 1)*sin(alpha21)-L(1, 2)*cos(alpha21), ...
    L(2, 1)*sin(alpha22)-L(2, 2)*cos(alpha22), ...
    L(3, 1)*sin(alpha23)-L(3, 2)*cos(alpha23)];

B3 = [cos(alpha31), cos(alpha32), cos(alpha33);...
    sin(alpha31), sin(alpha32), sin(alpha33);...
    L(1, 1)*sin(alpha31)-L(1, 2)*cos(alpha31), ...
    L(2, 1)*sin(alpha32)-L(2, 2)*cos(alpha32), ...
    L(3, 1)*sin(alpha33)-L(3, 2)*cos(alpha33)];

B4 = [cos(alpha41), cos(alpha42), cos(alpha43);...
    sin(alpha41), sin(alpha42), sin(alpha43);...
    L(1, 1)*sin(alpha41)-L(1, 2)*cos(alpha41), ...
    L(2, 1)*sin(alpha42)-L(2, 2)*cos(alpha42), ...
    L(3, 1)*sin(alpha43)-L(3, 2)*cos(alpha43)];

% M = [M11, 0, 0; 0, M22, M23; 0, M23, M33];

C1 = [0, 0, -M(2, 2) * v1t - M(2, 3) * r1t; 
     0, 0, M(1, 1) * u1t;
    M(2, 2) * v1t + M(3, 2) * r1t,  -M(1, 1)*u1t, 0];

C2 = [0, 0, -M(2, 2) * v2t - M(2, 3) * r2t; 
     0, 0, M(1, 1) * u2t;
    M(2, 2) * v2t + M(3, 2) * r2t,  -M(1, 1)*u2t, 0];

C3 = [0, 0, -M(2, 2) * v3t - M(2, 3) * r3t; 
     0, 0, M(1, 1) * u3t;
    M(2, 2) * v3t + M(3, 2) * r3t,  -M(1, 1)*u3t, 0];

C4 = [0, 0, -M(2, 2) * v4t - M(2, 3) * r4t; 
     0, 0, M(1, 1) * u4t;
    M(2, 2) * v4t + M(3, 2) * r4t,  -M(1, 1)*u4t, 0];


V1 = [u1t(t); v1t(t); r1t(t)];
V2 = [u2t(t); v2t(t); r2t(t)];
V3 = [u3t(t); v3t(t); r3t(t)];
V4 = [u4t(t); v4t(t); r4t(t)];

f(1:3) = R1 * V1;
f(4:6) = inv(M) * (B1 * [f11; f12; f13] * 1e3 - C1 * V1 - D * V1);%KN
f(7:9) = R2 * V2;
f(10:12) = inv(M) * (B2 * [f21; f22; f23] * 1e3 - C2 * V2 - D * V2);%KN
f(13:15) = R3 * V3;
f(16:18) = inv(M) * (B3 * [f31; f32; f33] * 1e3 - C3 * V3 - D * V3);%KN
f(19:21) = R4 * V4;
f(22:24) = inv(M) * (B4 * [f41; f42; f43] * 1e3 - C4 * V4 - D * V4);%KN

f = subs(f, statet, state);

f = simplify(f);

A = jacobian(f, [state{:}]);

% control = [f1, f2, f3, alpha1, alpha2, alpha3];
% control = [f1, f2, f3, f4, alpha1, alpha2, alpha3, alpha4];
B = jacobian(f, control);


    % Create ShipStateFcn.m
    matlabFunction(transpose(f), 'File', 'FourShipStateFcn',...
    'Vars', {transpose([state{:}]), transpose(control)})

    % Create ShipStateJacobianFcn.m 
    matlabFunction(A, B, 'File', 'FourShipStateJacobianFcn',...
    'Vars', {transpose([state{:}]), transpose(control)})
end


%Clear symbolic variables
clear
end



 
