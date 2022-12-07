function [Q_IN,Q_dIN,Q_Err]=createWeightMat()
% This function is used to create the weight matrice which can be globally
% unchanged.
% The locally variable matrice are defined in objective function.

global Q_err
global Q_in
global Q_din
global Q_fm
global Np
global Nc
global terCoe
%% 
%Q_in,Q_din
Q_In_cell=cell(Nc);
Q_dIn_cell=cell(Nc);
for i=1:Nc
    for j=1:Nc
        if i==j
            Q_In_cell{i,j}=Q_in;
            Q_dIn_cell{i,j}=Q_din;
        else
            Q_In_cell{i,j}=zeros(size(Q_in));
            Q_dIn_cell{i,j}=zeros(size(Q_din));
        end
    end
end
Q_In=cell2mat(Q_In_cell);
Q_dIn=cell2mat(Q_dIn_cell);

%Q_err_leader,Q_fm_fo
Q_err_leader_cell=cell(Np);
Q_fm_fo_cell=cell(Np);
for i=1:Np
    for j=1:Np
        if i==j
            if i==Np
                Q_err_leader_cell{i,j}=terCoe*Q_err;
                Q_fm_fo_cell{i,j}=terCoe*Q_fm;
            else
                Q_err_leader_cell{i,j}=Q_err;
                Q_fm_fo_cell{i,j}=Q_fm;
            end
        else
            Q_err_leader_cell{i,j}=zeros(size(Q_err));
            Q_fm_fo_cell{i,j}=zeros(size(Q_fm));
        end
    end
end
Q_err_leader=cell2mat(Q_err_leader_cell);
Q_fm_fo=cell2mat(Q_fm_fo_cell);

%Q_err_leader_addi,Q_fm_fo_addi 在objective function中定义

%% 输出
zerosIN=zeros(size(Q_In));
zerosErr=zeros(size(Q_err_leader));
%1
Q_IN=[Q_In zerosIN zerosIN;zerosIN Q_In zerosIN;zerosIN zerosIN Q_In];

%2
Q_dIN=[Q_dIn zerosIN zerosIN;zerosIN Q_dIn zerosIN;zerosIN zerosIN Q_dIn];

%3
Q_Err=[Q_err_leader zerosErr zerosErr;zerosErr Q_fm_fo zerosErr;zerosErr zerosErr Q_fm_fo];

%4
%Q_Err_addi 在objective function中定义
end