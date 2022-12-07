function [X_e,S,T,I,AJ_cal,BJ_cal]=createPredMat(U_e,x_e,w_estimated)
% This function is used to create the predictive matrice.

% Input
% U_e              ��ǰʱ�̵�seed control input����   u(0)��u(Nc-1)
% x_e              ��ǰʱ�̵�seed state               x0_e            
% w_estimated      ��ǰʱ�̵�Ԥ���Ļ����� 3x1

% Output
% X_e              ��һʱ�̿�ʼ��seed state����       x(1)��x(Np)
% S
% T
% I
% AJ_cal            ��ǰʱ�̵�Jacobian����    AJ(0)  
% BJ_cal            ��ǰʱ�̵�Jacobian����    BJ(0)

global Np;
global Nc;
global N;
global C;

%��ʼ��
X_e_cell=cell(Np,1);  %xe(1)-xe(Np)
A_collect=cell(Np,1); %AJ(0)-AJ(Np-1)
B_collect=cell(Nc,1); %BJ(0)-BJ(Nc-1)
%%   
for i=1:Np
      
        [x_e_next,AJ,BJ]=RungeKutta(x_e,U_e,w_estimated);
        X_e_cell{i}=x_e_next;      %X_e matrix
        A_collect{i}=AJ;       %A matrix [A(0) A(1) ... A(Np-1)]
        if i<=Nc
            B_collect{i}=BJ;   %B matrix [B(0) B(1) ... B(Nc-1)]
        end
        x_e=x_e_next;
%         U_e=[U_e(2*N+1:end);zeros(2*N,1)];%��ʵ������û����ģ���0
        U_e=[U_e(2*N+1:end);U_e(end-2*N+1:end)];%��ʵ������û����ģ���0
end
X_e=cell2mat(X_e_cell);

%��ȡ�����ڼ�����һ��״̬������Ԥ�����
AJ_cal=A_collect{1};
BJ_cal=B_collect{1};

%% 
    %����A B G C���������������Ԥ�����󣬼ǵó���C=[I 0]
    %S
    S_cell=cell(Np,Nc);
    zeros_C_BJ=zeros(size(C*BJ_cal));
    for i=1:Np      %��
        for j=1:Nc  %��
            if (i<j)
                S_cell{i,j}=zeros_C_BJ;
            elseif (i==j)
                S_cell{i,j}=C*B_collect{j};
            else
                S_cell{i,j}=C*contProd(A_collect,i-1,j)*B_collect{j};
            end
        end
    end
    S=cell2mat(S_cell);

    %T
    T_cell=cell(Np,1);
    for i=1:Np
        T_cell{i}=C*contProd(A_collect,i-1,0);
    end
    T=cell2mat(T_cell);

    %I
    I_cell=cell(Np);
    zeros_C=zeros(size(C));
    for i=1:Np
        for j=1:Np
            if i==j
                I_cell{i,j}=C;
            else
                I_cell{i,j}=zeros_C;
            end
        end
    end
    I=cell2mat(I_cell);

%     %W
%     W_cell=cell(Np,1);
%     W_cell{1}=C*G;
%     for i=2:Np
%         W_cell{i}=C*contAdd(A_collect,i-1,1)*G+W_cell{1};
%     end
%     W=cell2mat(W_cell);

end