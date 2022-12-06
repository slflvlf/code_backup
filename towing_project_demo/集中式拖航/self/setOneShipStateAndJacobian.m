function [] = setOneShipStateAndJacobian(M, D, L, runFlag)
% function [] = setOneShipStateAndJacobian( runFlag)
if runFlag
syms xt(t) yt(t) fait(t) ut(t) vt(t) rt(t)
syms x y fai u v r

syms f1 f2 f3 f4 alpha1 alpha2 alpha3 alpha4

% % 船舶参数（拖轮）
% M1 = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
% Ma1 = [7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5, 8.77e6];
% MM1 = M1 + Ma1;
% D1 = [MM1(1,1)/100, 0, 0; 0, MM1(2,2)/40, 0; 0, 0, MM1(3,3)/20];
% L1 = [-5.02 0; 11.956 2.7; 11.956 -2.7];



statet = {xt(t) yt(t) fait(t) ut(t) vt(t) rt(t)};

state = {x y fai u v r};

state_diff = {diff(xt(t), t), diff(yt(t), t), diff(fait(t), t), ...
    diff(ut(t), t), diff(vt(t), t), diff(rt(t), t)};

% state_dot = { xdot ydot faidot udot vdot rdot};

R = [cos(fait), -sin(fait), 0; sin(fait), cos(fait), 0; 0, 0, 1];

B = [cos(alpha1), cos(alpha2), cos(alpha3), cos(alpha4);...
    sin(alpha1), sin(alpha2), sin(alpha3), sin(alpha4);...
    L(1, 1)*sin(alpha1)-L(1, 2)*cos(alpha1), ...
    L(2, 1)*sin(alpha2)-L(2, 2)*cos(alpha2), ...
    L(3, 1)*sin(alpha3)-L(3, 2)*cos(alpha3), ...
    L(4, 1)*sin(alpha4)-L(4, 2)*cos(alpha4)];

% M = [M11, 0, 0; 0, M22, M23; 0, M23, M33];

C = [0, 0, -M(2, 2) * vt - M(2, 3) * rt; 
     0, 0, M(1, 1) * ut;
    M(2, 2) * vt + M(3, 2) * rt,  -M(1, 1)*ut, 0];


V = [ut(t); vt(t); rt(t)];

f(1:3) = R * V;

% 加入可测扰动
f(4:6) = inv(M) * (B * [f1; f2; f3; f4] * 1e3 - C * V - D * V);%KN

f = subs(f, statet, state);

f = simplify(f);

A = jacobian(f, [state{:}]);

control = [f1, f2, f3, f4, alpha1, alpha2, alpha3, alpha4];
% control = [f1, f2, f3, f4, alpha1, alpha2, alpha3, alpha4];
B = jacobian(f, control);


    % Create ShipStateFcn.m
    matlabFunction(transpose(f),'File','OneShipStateFcn',...
    'Vars',{transpose([state{:}]),transpose(control)})

    % Create ShipStateJacobianFcn.m 
    matlabFunction(A, B,'File','OneShipStateJacobianFcn',...
    'Vars',{transpose([state{:}]),transpose(control)})
end


%Clear symbolic variables
clear
end



 
