function [X_e,S,T,I,AJ_cal,BJ_cal]=createPredMat(U_e,x_e,w_estimated)
% This function is used to create the predictive matrice.

% Input
% U_e              当前时刻的seed control input序列   u(0)到u(Nc-1)
% x_e              当前时刻的seed state               x0_e            
% w_estimated      当前时刻的预估的环境力 3x1

% Output
% X_e              下一时刻开始的seed state序列       x(1)到x(Np)
% S
% T
% I
% AJ_cal            当前时刻的Jacobian矩阵    AJ(0)  
% BJ_cal            当前时刻的Jacobian矩阵    BJ(0)

global Np;
global Nc;
global N;
global C;

%初始化
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
%         U_e=[U_e(2*N+1:end);zeros(2*N,1)];%其实这样是没问题的，后补0
        U_e=[U_e(2*N+1:end);U_e(end-2*N+1:end)];%其实这样是没问题的，后补0
end
X_e=cell2mat(X_e_cell);

%提取出用于计算下一步状态变量的预测矩阵
AJ_cal=A_collect{1};
BJ_cal=B_collect{1};

%% 
    %根据A B G C矩阵生成下列输出预测大矩阵，记得乘上C=[I 0]
    %S
    S_cell=cell(Np,Nc);
    zeros_C_BJ=zeros(size(C*BJ_cal));
    for i=1:Np      %行
        for j=1:Nc  %列
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