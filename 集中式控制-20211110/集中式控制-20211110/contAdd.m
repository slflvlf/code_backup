function A_contAdd=contAdd(A_collect,idx_end,idx_begin)
% This function is used to continuously add the matrix of A.

% Input
% idx_end    A矩阵的最大下标
% idx_begin  A矩阵的最小下标


A_contAdd=zeros(6);%初始化
for i=idx_end+1:-1:idx_begin+1
    A_contAdd=A_contAdd+contProd(A_collect,idx_end,i);
end

end