function str = getVarName(var)

    str = sprintf('%s', inputname(1));

    num = strlength(str);

    str = str(1:num-4);

end

