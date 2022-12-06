function A_contProd=contProd(A_collect,idx_end,idx_begin)
% This function is used to create the continuous product matrix of A.
% Used in predictive matrix S, T, and W.

% Input
% A_collect   [A(0) A(1) ... A(Np-1)]
% idx_end         ���˵�A������ֹ�±�
% idx_begin       ���˵�A������ʼ�±�

% Output
% A_contProd  �������˺�ľ���

A_contProd=eye(6);%��ʼ��
for i=idx_end+1:-1:idx_begin+1 %��Ϊmatlab�Ǵ�1��ʼ
    A_contProd=A_contProd*A_collect{i};
end

end