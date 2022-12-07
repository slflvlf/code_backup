% function [pd1, pd2] = consensus_guidance_controller(p0, v0, p1, v1, p2, v2)
% 默认输入和输出的单位为角度
% p(全局坐标系）， v(局部坐标系）
% dot_p (全局坐标系）
clear all; clc;
p1 = [397, 237, 31]';
p2 = [397, -237, -31]';
p0 = [1, 0, 0]';
v0 = zeros(3, 1);
v1 = zeros(3, 1);
v2 = zeros(3, 1);

% 角度单位转化成弧度
deg_to_rad = pi / 180;
p0(3) = p0(3) * deg_to_rad;
p1(3) = p1(3) * deg_to_rad;
p2(3) = p2(3) * deg_to_rad;

dotP0 = V_to_dotP(p0, v0);
dotP1 = V_to_dotP(p1, v1);
dotP2 = V_to_dotP(p2, v2);
% 定义状态变量
x0 = [p0; dotP0];
x1 = [p1; dotP1];
x2 = [p2; dotP2];

%% 编队的相对位置
deta10 = [397, 237, 31 * deg_to_rad, 0, 0, 0]';
deta20 = [397, -237, -31 * deg_to_rad, 0, 0, 0]';
deta12 = [0, 474, 62 * deg_to_rad, 0, 0, 0]';
deta21 = -deta12;

% 先假定aij = 1, gi = 1;， 需要加上相对位置
e1 = x2 - x1 - deta21 + x0 - x1 + deta10;
e2 = x1 - x2 - deta12 + x0 - x2 + deta20;

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
K

u1 = K * e1
u2 = K * e2

pd1 = u1; 
pd2 = u2;

pd1 = pd1 + p1;
pd2 = pd2 + p2;

%% 函数输入输出都是角度
pd1(3) = pd1(3) / 3.14 * 180
pd2(3) = pd2(3) / 3.14 * 180







%% 旋转矩阵
function R = RotationMatrix(p)
fai = p(3);
R = [cos(fai), -sin(fai), 0;
    sin(fai), cos(fai), 0;
    0, 0, 1];
end

%% 将速度转换成全局坐标系中，即p的微分
function dot_P = V_to_dotP(p, v)
R = RotationMatrix(p);
dot_P = R * v;

end



% function 