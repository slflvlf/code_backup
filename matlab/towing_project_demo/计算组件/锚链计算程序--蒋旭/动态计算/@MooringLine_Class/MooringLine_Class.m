classdef MooringLine_Class
    %MOORINGLINE_CLASS 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        mooring_param = struct('length', 300, 'element_length', 10, 'element_number', 30,... 
                'node_number', 31, 'E', 9.16e10, 'diameter', 0.09, 'cross_area', 6.362e-3, ...
                'density', 126, 'time_step', 0.1, 'coef_drag', 1, 'coef_addmass', 1,...
                'current', [0; 0; 0]);
            
        p0_next, p1_next, Y_N, vv, aa;
    end
    
    methods
        function obj = MooringLine_Class(inputArg1,inputArg2)
            %MOORINGLINE_CLASS 构造此类的实例
            %   此处显示详细说明
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            outputArg = obj.Property1 + inputArg;
        end
    end
end

