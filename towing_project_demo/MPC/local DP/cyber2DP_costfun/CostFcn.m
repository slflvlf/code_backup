function J = CostFcn(X, U, e, data, ref)
%COSTFCN ´ú¼Ûº¯Êý
% Yd = param.Yd;
% global Ref;
pd = ref;
% pd = data.References;
global Q_error Q_input;
output_weight = diag(Q_error);
input_weight = diag(Q_input);
% Use states from k+1 to k+p
    p = X(2:end, 1:3);
    input = U(1:end-1, 1:8);
    J = 0;
    for i = 1 : p(:, 1)
        error = p(i, :) - pd(i, :);
        J = J + 0.5 * error * output_weight * error';
    end
    
    for i = 1 : input(:, 1) 
        J = J + 0.5 * input(i, :) * input_weight * input(i, :)';
    end
    

end

