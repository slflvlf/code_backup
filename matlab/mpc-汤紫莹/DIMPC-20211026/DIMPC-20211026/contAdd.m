function A_contAdd=contAdd(A_collect,idx_end,idx_begin)
% This function is used to continuously add the matrix of A.

% Input
% idx_end    A���������±�
% idx_begin  A�������С�±�


A_contAdd=zeros(6);%��ʼ��
for i=idx_end+1:-1:idx_begin+1
    A_contAdd=A_contAdd+contProd(A_collect,idx_end,i);
end

end