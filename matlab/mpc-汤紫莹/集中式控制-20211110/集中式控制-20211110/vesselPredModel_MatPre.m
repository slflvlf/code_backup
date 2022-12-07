function [U_e,X_e,S,T,I,AJ_cal,BJ_cal]=vesselPredModel_MatPre(U_last,x_e_last,w_estimated,flag)
% This function is used to create the matrices of the vessel predictive model.
% Ԥ��ģ�͵ľ���ֻ��seed state��seed control���

% Input     
% U_last        ��һ���������ŵĿ�����������    ����8Ncx1    �ബ3*8Ncx1
% x_e_last      ��һ����seed state               ����6x1      �ബ3*6x1   
% w_estimated   ��һ��Ԥ���Ļ�����               ����3x1       �ബ3*3x1
% flag          �жϵ���or�ബ                   ����flag==1  �ബflag==2 

% Output
% U_e           ��һ����seed control sequence���Ǹ���U_last��õ�
% X_e           ��һ����seed state sequence X_e=[xe(1) xe(2) ... xe(Np)]
% S
% T
% I
% A_cal         ��һ����Ԥ����������������µ�״̬����
% B_cal         ��һ����Ԥ�����

global Nc;
global N;           %ÿ����������������

%%
if flag==1 %����
    %U_e
    U_e=[U_last(2*N+1:end);zeros(2*N,1)];%������һ���������U_last������һ��seed������������U_e����ǰ�ƽ�һ������0
    %U_e=U_last;
    %x_e
    x_e=x_e_last;
    %���ɵ���Ԥ�����
    [X_e,S,T,I,AJ_cal,BJ_cal]=createPredMat(U_e,x_e,w_estimated);
 
elseif flag==2 %�ബ
    %���U_last
    U_self_last=U_last(1:2*N*Nc);%follower�Լ�����һ����������
    U_neighbor1_last=U_last(2*N*Nc+1:2*2*N*Nc);
    U_neighbor2_last=U_last(2*2*N*Nc+1:3*2*N*Nc);
    
    Ue_self=[U_self_last(2*N+1:end);zeros(2*N,1)];%follower�Լ�����һ����seed��������
    Ue_neighbor1=[U_neighbor1_last(2*N+1:end);zeros(2*N,1)];
    Ue_neighbor2=[U_neighbor2_last(2*N+1:end);zeros(2*N,1)];
    
    %������������U_e������Ϊ���
    U_e=[Ue_self;Ue_neighbor1;Ue_neighbor2];
    
    %���x_e_last
    xe_self=x_e_last(1:6);%follower�Լ�����һ��״̬����
    xe_neighbor1=x_e_last(7:12);
    xe_neighbor2=x_e_last(13:18);
    
    %���w
    w_self=w_estimated(1:3);
    w_neighbor1=w_estimated(4:6);
    w_neighbor2=w_estimated(7:9);
    
    %��������Ԥ������ǲ�һ���� ��ֱ����
    [Xe_self,S_self,T_self,I_self,AJcal_self,BJcal_self]=createPredMat(Ue_self,xe_self,w_self);   
    [Xe_neighbor1,S_neighbor1,T_neighbor1,I_neighbor1,AJcal_neighbor1,BJcal_neighbor1]=createPredMat(Ue_neighbor1,xe_neighbor1,w_neighbor1);
    [Xe_neighbor2,S_neighbor2,T_neighbor2,I_neighbor2,AJcal_neighbor2,BJcal_neighbor2]=createPredMat(Ue_neighbor2,xe_neighbor2,w_neighbor2);
    
    %������������Ԥ�����(�ϲ�)
    X_e=[Xe_self;Xe_neighbor1;Xe_neighbor2];
    
    zeroS=zeros(size(S_self));zeroT=zeros(size(T_self));zeroI=zeros(size(I_self));
    S=[S_self zeroS zeroS;zeroS S_neighbor1 zeroS;zeroS zeroS S_neighbor2];
    T=[T_self zeroT zeroT;zeroT T_neighbor1 zeroT;zeroT zeroT T_neighbor2];
    I=[I_self zeroI zeroI;zeroI I_neighbor1 zeroI;zeroI zeroI I_neighbor2];
    
    AJ_cal=[AJcal_self AJcal_neighbor1 AJcal_neighbor2];%������һ����״̬�����ļ���
    BJ_cal=[BJcal_self BJcal_neighbor1 BJcal_neighbor2];
    
end %if

end %function
