function [U_e,X_e,S,T,I,AJ_cal,BJ_cal]=vesselPredModel_MatPre(U_last,x_e_last,w_estimated,flag)
% This function is used to create the matrices of the vessel predictive model.
% 预测模型的矩阵只与seed state和seed control相关

% Input     
% U_last        上一步求解的最优的控制输入序列    单船8Ncx1    多船3*8Ncx1
% x_e_last      上一步的seed state               单船6x1      多船3*6x1   
% w_estimated   上一步预估的环境力               单船3x1       多船3*3x1
% flag          判断单船or多船                   单船flag==1  多船flag==2 

% Output
% U_e           这一步的seed control sequence，是根据U_last求得的
% X_e           这一步的seed state sequence X_e=[xe(1) xe(2) ... xe(Np)]
% S
% T
% I
% A_cal         上一步的预测矩阵，用于最后计算新的状态变量
% B_cal         上一步的预测矩阵

global Nc;
global N;           %每条船的推力器数量

%%
if flag==1 %单船
    %U_e
    U_e=[U_last(2*N+1:end);zeros(2*N,1)];%根据上一步求出来的U_last计算这一步seed控制输入序列U_e，向前推进一个，后补0
    %U_e=U_last;
    %x_e
    x_e=x_e_last;
    %生成单船预测矩阵
    [X_e,S,T,I,AJ_cal,BJ_cal]=createPredMat(U_e,x_e,w_estimated);
 
elseif flag==2 %多船
    %拆分U_last
    U_self_last=U_last(1:2*N*Nc);%follower自己的上一步控制序列
    U_neighbor1_last=U_last(2*N*Nc+1:2*2*N*Nc);
    U_neighbor2_last=U_last(2*2*N*Nc+1:3*2*N*Nc);
    
    Ue_self=[U_self_last(2*N+1:end);zeros(2*N,1)];%follower自己的这一步的seed控制序列
    Ue_neighbor1=[U_neighbor1_last(2*N+1:end);zeros(2*N,1)];
    Ue_neighbor2=[U_neighbor2_last(2*N+1:end);zeros(2*N,1)];
    
    %生成三条船的U_e矩阵，作为输出
    U_e=[Ue_self;Ue_neighbor1;Ue_neighbor2];
    
    %拆分x_e_last
    xe_self=x_e_last(1:6);%follower自己的上一步状态变量
    xe_neighbor1=x_e_last(7:12);
    xe_neighbor2=x_e_last(13:18);
    
    %拆分w
    w_self=w_estimated(1:3);
    w_neighbor1=w_estimated(4:6);
    w_neighbor2=w_estimated(7:9);
    
    %三条船的预测矩阵是不一样的 需分别计算
    [Xe_self,S_self,T_self,I_self,AJcal_self,BJcal_self]=createPredMat(Ue_self,xe_self,w_self);   
    [Xe_neighbor1,S_neighbor1,T_neighbor1,I_neighbor1,AJcal_neighbor1,BJcal_neighbor1]=createPredMat(Ue_neighbor1,xe_neighbor1,w_neighbor1);
    [Xe_neighbor2,S_neighbor2,T_neighbor2,I_neighbor2,AJcal_neighbor2,BJcal_neighbor2]=createPredMat(Ue_neighbor2,xe_neighbor2,w_neighbor2);
    
    %生成三条船的预测矩阵(合并)
    X_e=[Xe_self;Xe_neighbor1;Xe_neighbor2];
    
    zeroS=zeros(size(S_self));zeroT=zeros(size(T_self));zeroI=zeros(size(I_self));
    S=[S_self zeroS zeroS;zeroS S_neighbor1 zeroS;zeroS zeroS S_neighbor2];
    T=[T_self zeroT zeroT;zeroT T_neighbor1 zeroT;zeroT zeroT T_neighbor2];
    I=[I_self zeroI zeroI;zeroI I_neighbor1 zeroI;zeroI zeroI I_neighbor2];
    
    AJ_cal=[AJcal_self AJcal_neighbor1 AJcal_neighbor2];%用于下一步的状态变量的计算
    BJ_cal=[BJcal_self BJcal_neighbor1 BJcal_neighbor2];
    
end %if

end %function
