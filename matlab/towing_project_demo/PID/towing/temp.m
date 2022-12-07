% function [pd1, pd2] = consensus_guidance(p0, v0, p1, v1, p2, v2)
function [pd1, pd2, pd3, pd4] = consensus_guidance(p0, v0, state)
% 默认输入和输出的单位为角度
% % p0, v0的角度单位都是弧度
% p(全局坐标系）， v(局部坐标系）
% dot_p (全局坐标系）
% clear all; clc;
% p1 = [397, 237, 31]';
% p2 = [397, -237, -31]';
% p0 = [1, 0, 0]';
% v0 = zeros(3, 1);
% v1 = zeros(3, 1);
% v2 = zeros(3, 1);

p1 = state(1:3);
v1 = state(4:6);
p2 = state(7:9);
v2 = state(10:12);
p3 = state(13 : 15);
v3 = state(16 : 18);
p4 = state(19 :21);
v4 = state(22 : 24);

% 需要声明lqr函数为外部函数，否则simulink会编译报错
coder.extrinsic('lqr'); 

pd1 = zeros(3, 1);
pd2 = zeros(3, 1);
pd3 = zeros(3, 1);
pd4 = zeros(3, 1);

% 角度单位转化成弧度
deg_to_rad = pi / 180;
p0(3) = p0(3) * deg_to_rad;
p1(3) = p1(3) * deg_to_rad;
p2(3) = p2(3) * deg_to_rad;
p3(3) = p3(3) * deg_to_rad;
p4(3) = p4(3) * deg_to_rad;

dotP0 = v0; % 输入的v0是在大地坐标系下的   
dotP0(3) = dotP0(3) * deg_to_rad;

% % 将角度转化为（-pi, pi)
% p0(3) = transfer_deg(p0(3));
% p1(3) = transfer_deg(p1(3));
% p2(3) = transfer_deg(p2(3));

% dotP0 = V_to_dotP(p0, v0);
dotP1 = V_to_dotP(p1, v1);
dotP2 = V_to_dotP(p2, v2);
dotP3 = V_to_dotP(p3, v3);
dotP4 = V_to_dotP(p4, v4);

% 定义状态变量
x0 = [p0; dotP0];
x1 = [p1; dotP1];
x2 = [p2; dotP2];
x3 = [p3; dotP3];
x4 = [p4; dotP4];

%% 编队的相对位置,这个是在局部坐标系下的，需要转化为大地坐标系下
deta10 = [397, 237, 31 * deg_to_rad, 0, 0, 0]';
deta20 = [397, -237, -31 * deg_to_rad, 0, 0, 0]';
deta30 = [-397, -237, -149 * deg_to_rad, 0, 0, 0]';
deta40 = [-397, 237, 149 * deg_to_rad, 0, 0, 0]';

deta12 = deta10 - deta20;   deta21 = -deta12;
deta14 = deta10 - deta40;   deta41 = -deta14;
deta23 = deta20 - deta30;   deta32 = -deta23;
deta34 = deta30 - deta40;   deta43 = -deta34;


fai0 = p0(3);   fai1 = p1(3);   fai2 = p2(3);
fai3 = p3(3);   fai4 = p4(3);

%拖船到母船的距离
Lc1 = sqrt(deta10(1)^2 + deta10(2)^2);
Lc2 = sqrt(deta20(1)^2 + deta20(2)^2);
Lc3 = sqrt(deta30(1)^2 + deta30(2)^2);
Lc4 = sqrt(deta40(1)^2 + deta40(2)^2);

deta10(1:2) = [cos(fai0), -sin(fai0); sin(fai0), cos(fai0)] * deta10(1:2);
deta20(1:2) = [cos(fai0), -sin(fai0); sin(fai0), cos(fai0)] * deta20(1:2);
deta30(1:2) = [cos(fai0), -sin(fai0); sin(fai0), cos(fai0)] * deta30(1:2);
deta40(1:2) = [cos(fai0), -sin(fai0); sin(fai0), cos(fai0)] * deta40(1:2);

deta12 = deta10 - deta20;   deta21 = -deta12;
deta14 = deta10 - deta40;   deta41 = -deta14;
deta23 = deta20 - deta30;   deta32 = -deta23;
deta34 = deta30 - deta40;   deta43 = -deta34;
% deta12 = deta10 - deta20;
% deta21 = - deta12;



% 先假定aij = 1, gi = 1;， 需要加上相对位置  误差是在全局坐标系下的
e1 = x2 - x1 - deta21 + x4 - x1 - deta41 + x0 - x1 + deta10;
e2 = x1 - x2 - deta12 + x3 - x2 - deta32 + x0 - x2 + deta20;
e3 = x2 - x3 - deta23 + x4 - x3 - deta43 + x0 - x3 + deta30;
e4 = x1 - x4 - deta14 + x3 - x4 - deta34 + x0 - x4 + deta40;

e1(3) = transfer_deg(e1(3));
e2(3) = transfer_deg(e2(3));
e3(3) = transfer_deg(e3(3));
e4(3) = transfer_deg(e4(3));

%% 定义状态反馈方程的系数，求解raccati方程
wn = 0.05; 
cd = 0.7;

A1 = -2 * cd * wn * eye(3);
A0 = -wn^2 * eye(3);
B1 = -A0;

A = [zeros(3, 3), eye(3); A0, A1];
B = [zeros(3, 3); B1];

% Q = diag([0.01, 0.01, 4e-8, 100, 100, 4e-4]);
% R = diag([0.01, 0.01, 4e-8]);
Q = 4 * eye(6);
R = 1 * eye(3);
N = [];
[K, S, e] = lqr(A, B, Q, R, N);

K;
u1 = K * e1;
u2 = K * e2;
u3 = K * e3;
u4 = K * e4;

pd1 = u1; 
pd2 = u2;
pd3 = u3; 
pd4 = u4;

pd1 = pd1 + p1;
pd2 = pd2 + p2;
pd3 = pd3 + p3;
pd4 = pd4 + p4;

pd1(3) = transfer_deg(pd1(3));
pd2(3) = transfer_deg(pd2(3));
pd3(3) = transfer_deg(pd3(3));
pd4(3) = transfer_deg(pd4(3));


%% 函数输入输出都是角度
pd1(3) = pd1(3) / 3.14 * 180;
pd2(3) = pd2(3) / 3.14 * 180;
pd3(3) = pd3(3) / 3.14 * 180;
pd4(3) = pd4(3) / 3.14 * 180;

end







%% 旋转矩阵
function R = RotationMatrix(p)
fai = p(3);
R = zeros(3, 3);
R = [cos(fai), -sin(fai), 0;
    sin(fai), cos(fai), 0;
    0, 0, 1];
end

%% 将速度转换成全局坐标系中，即p的微分
function dot_P = V_to_dotP(p, v)
R = zeros(3, 3);
R = RotationMatrix(p);
dot_P = R * v;

end

%% 将角度转化为（-pi, pi)
function y = transfer_deg(x)
    y = -sign(x) * pi + rem((x + sign(x) * pi), 2 * pi);
end



