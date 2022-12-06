function [c,ceq]=safedistConstraint_substep(U_self,U_neighbor1,U_neighbor2,U_e,x0,x0_e,X_e,S,T,I)
% This function is used to create the nonlinear safe distance constraints of
% the followers.

% Input
% U_self          ������Լ������ſ�������              8xNc
% U_neighbor1     ��һ����õ�neighbor1�Ŀ�������       8xNc
% U_neighbor2     ��һ����õ�neighbor2�Ŀ�������       8xNc
% U_e             ��������seed��������                  3x8xNc
% x0              ��һ������������״̬����              3x6
% x0_e            ��һ������������seed״̬����          3x6
% X_e             ������seed״̬���� ����һ����δ��Np�� 3x6xNp
% S               ��������Ԥ�����
% T               ��������Ԥ�����
% I               ��������Ԥ�����

% Output
% c               �����Բ���ʽԼ��
% ceq             �����Ե�ʽԼ��

global Np

%�������Ŀ�������
U=[U_self;U_neighbor1;U_neighbor2];%U_self�Ǵ�����
%��������������λ�ú�����ǣ�û�м�������
Y=S*(U-U_e)+T*(x0-x0_e)+I*X_e;
Y_self=Y(1:3*Np);
Y_neighbor1=Y(3*Np+1:6*Np);
%Y_neighbor2=Y(6*Np+1:end);

%��ȫ��������
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