function A_contProd=contProd(A_collect,idx_end,idx_begin)
% This function is used to create the continuous product matrix of A.
% Used in predictive matrix S, T, and W.

% Input
% A_collect   [A(0) A(1) ... A(Np-1)]
% idx_end         连乘的A矩阵终止下标
% idx_begin       连乘的A矩阵起始下标

% Output
% A_contProd  返回连乘后的矩阵

A_contProd=eye(6);%初始化
for i=idx_end+1:-1:idx_begin+1 %因为matlab是从1开始
    A_contProd=A_contProd*A_collect{i};
end

end