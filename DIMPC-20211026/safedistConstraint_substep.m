function [c,ceq]=safedistConstraint_substep(U_self,U_neighbor1,U_neighbor2,U_e,x0,x0_e,X_e,S,T,I)
% This function is used to create the nonlinear safe distance constraints of
% the followers.

% Input
% U_self          待求的自己的最优控制序列              8xNc
% U_neighbor1     上一步求得的neighbor1的控制序列       8xNc
% U_neighbor2     上一步求得的neighbor2的控制序列       8xNc
% U_e             三条船的seed控制序列                  3x8xNc
% x0              上一步的三条船的状态变量              3x6
% x0_e            上一步的三条船的seed状态变量          3x6
% X_e             三条船seed状态变量 从这一步到未来Np步 3x6xNp
% S               三条船的预测矩阵
% T               三条船的预测矩阵
% I               三条船的预测矩阵

% Output
% c               非线性不等式约束
% ceq             非线性等式约束

global Np

%三条船的控制序列
U=[U_self;U_neighbor1;U_neighbor2];%U_self是待求量
%计算三条船舶的位置和艏向角，没有加上噪声
Y=S*(U-U_e)+T*(x0-x0_e)+I*X_e;
Y_self=Y(1:3*Np);
Y_neighbor1=Y(3*Np+1:6*Np);
%Y_neighbor2=Y(6*Np+1:end);

%安全距离限制
dsafe_leader=0.45;
%dsafe_fo=0.9;

c1=zeros(Np,1);
%c2=zeros(Np,1);
for i=1:Np
    c1(i)=dsafe_leader-sqrt((Y_self(3*i-2)-Y_neighbor1(3*i-2))^2+(Y_self(3*i-1)-Y_neighbor1(3*i-1))^2);
    %c2(i)=dsafe_fo-sqrt((Y_self(3*i-2)-Y_neighbor2(3*i-2))^2+(Y_self(3*i-1)-Y_neighbor2(3*i-1))^2);
end
c=c1;
ceq=[];

end