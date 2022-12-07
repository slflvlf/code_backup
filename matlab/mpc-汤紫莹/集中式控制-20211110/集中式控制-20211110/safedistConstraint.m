function [c,ceq]=safedistConstraint(U,U_e,x0,x0_e,X_e,S,T,I)
% This function is used to create the nonlinear constraints of the global
% controller.

global Np

%����������λ�ú�����ǣ�û�м�������
Y=S*(U-U_e)+T*(x0-x0_e)+I*X_e;
Y_leader=Y(1:3*Np);
Y_leftfo=Y(3*Np+1:6*Np);
Y_rightfo=Y(6*Np+1:end);
%��ȫ��������
dsafe_leader=0.45;

c1=zeros(Np,1);
c2=zeros(Np,1);
for i=1:Np
    c1(i)=dsafe_leader-sqrt((Y_leader(3*i-2)-Y_leftfo(3*i-2))^2+(Y_leader(3*i-1)-Y_leftfo(3*i-1))^2);
    c2(i)=dsafe_leader-sqrt((Y_leader(3*i-2)-Y_rightfo(3*i-2))^2+(Y_leader(3*i-1)-Y_rightfo(3*i-1))^2); 
end
c=[c1;c2];
ceq=[];

end