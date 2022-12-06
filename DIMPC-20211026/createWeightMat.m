function [Q_err_leader,Q_in_leader,Q_din_leader,Q_in_fo_firstep,Q_din_fo_firstep,Q_fm_fo_firstep,Q_in_fo_substep,Q_din_fo_substep,Q_fm_fo_substep]=createWeightMat()
% This function is used to create the weight matrice which can be globally
% unchanged.
% The locally variable matrice are defined in each controller function.

global Q_err
global Q_in
global Q_dinleader
global Q_dinfo
global Q_fm
global Np
global Nc
global terCoe
%% leader
%Q_err_leader
Q_err_leader_cell=cell(Np);
for i=1:Np
    for j=1:Np
        if i==j
            if i==Np
                Q_err_leader_cell{i,j}=terCoe*Q_err;
            else
                Q_err_leader_cell{i,j}=Q_err;
            end
        else
            Q_err_leader_cell{i,j}=zeros(size(Q_err));
        end
    end
end
Q_err_leader=cell2mat(Q_err_leader_cell);

%Q_err_leader_addi 在controller_leader中定义

%Q_in_leader,Q_din_leader
Q_in_leader_cell=cell(Nc);
Q_din_leader_cell=cell(Nc);
for i=1:Nc
    for j=1:Nc
        if i==j
            Q_in_leader_cell{i,j}=Q_in;
            Q_din_leader_cell{i,j}=Q_dinleader;
        else
            Q_in_leader_cell{i,j}=zeros(size(Q_in));
            Q_din_leader_cell{i,j}=zeros(size(Q_dinleader));
        end
    end
end
Q_in_leader=cell2mat(Q_in_leader_cell);
Q_din_leader=cell2mat(Q_din_leader_cell);

%% follower_input follower_dinput 
Q_in_fo=Q_in_leader;
%Q_din_fo=Q_din_leader;

Q_din_fo_cell=cell(Nc);
for i=1:Nc
    for j=1:Nc
        if i==j
            
            Q_din_fo_cell{i,j}=Q_dinfo;
        else
            
            Q_din_fo_cell{i,j}=zeros(size(Q_dinfo));
        end
    end
end
Q_din_fo=cell2mat(Q_din_fo_cell);

%Q_in_fo_substep Q_din_fo_substep
Q_in_fo_substep=Q_in_fo;%单船
Q_din_fo_substep=Q_din_fo;

%Q_in_fo_firstep Q_din_fo_firstep
zeroQinfo=zeros(size(Q_in_fo));
Q_in_fo_firstep=[Q_in_fo zeroQinfo zeroQinfo;zeroQinfo Q_in_fo zeroQinfo;zeroQinfo zeroQinfo Q_in_fo];%多船
Q_din_fo_firstep=[Q_din_fo zeroQinfo zeroQinfo;zeroQinfo Q_din_fo zeroQinfo;zeroQinfo zeroQinfo Q_din_fo];

%% follower_formation
%Q_fm_fo_substep
Q_fm_fo_substep_cell=cell(Np);
for i=1:Np
    for j=1:Np
        if i==j
            if i==Np
                Q_fm_fo_substep_cell{i,j}=2*terCoe*Q_fm;%terminal cost
            else
                Q_fm_fo_substep_cell{i,j}=2*Q_fm;
            end
        else
            Q_fm_fo_substep_cell{i,j}=zeros(size(Q_fm));
        end
    end
end
Q_fm_fo_substep=cell2mat(Q_fm_fo_substep_cell);

%Q_fm_fo_substep_addi 在objfunc_fo_substep中定义

%Q_fm_fo_firstep
zeroQfmfosub=zeros(size(Q_fm_fo_substep));
Q_fm_fo_firstep=[Q_fm_fo_substep      -0.5*Q_fm_fo_substep -0.5*Q_fm_fo_substep ;...
                 -0.5*Q_fm_fo_substep 0.5*Q_fm_fo_substep  zeroQfmfosub         ;...
                 -0.5*Q_fm_fo_substep zeroQfmfosub         0.5*Q_fm_fo_substep  ];

%Q_fm_fo_firstep_addi 在objfunc_fo_firstep中定义
end