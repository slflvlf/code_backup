function [F] = mooring_line_catenary_fcn(l, F, mooring_param)
    


    E = mooring_param.E;
    A = mooring_param.cross_area;
    EA = E * A;
    w = mooring_param.density;

    L = mooring_param.length;
    
    %选择上一步的张力F作为初始值，求解悬链线方程
    
%     syms T;
%     eq = @(T)2*T/w*asinh(w*L/2/T)+T*L/EA-l;
%     T = solve(eq, T); %符号求解慢 

    eq = @(T)2*T/w*asinh(w*L/2/T)+T*L/EA-l;
    T = fzero(eq, F*1e3); %这个求解结果比较符合静态计算，fsolve结果差别较大


    F = T/1e3;%转化为KN



end


