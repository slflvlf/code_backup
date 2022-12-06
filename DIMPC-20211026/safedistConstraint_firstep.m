function [c,ceq]=safedistConstraint_firstep(U,U_e,x0,x0_e,X_e,S,T,I)
% This function is used to create the nonlinear safe distance constraints of
% the followers.

global Np

%三条船舶的位置和艏向角，没有加上噪声
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