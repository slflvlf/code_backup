classdef MooringLine_Class
    %MOORINGLINE_CLASS �˴���ʾ�йش����ժҪ
    %   �˴���ʾ��ϸ˵��
    
    properties
        mooring_param = struct('length', 300, 'element_length', 10, 'element_number', 30,... 
                'node_number', 31, 'E', 9.16e10, 'diameter', 0.09, 'cross_area', 6.362e-3, ...
                'density', 126, 'time_step', 0.1, 'coef_drag', 1, 'coef_addmass', 1,...
                'current', [0; 0; 0]);
            
        p0, p1, p0_next, p1_next, Y_N;
        line_shape_static;
        line_shape_dynamic;
        TR1, TRn, TR1_his, TRn_his;

        control_step;
    end
    
    methods
        % ���캯�������м�����һ����̬���㣬��ҪΪ�˵õ�ê����̬
        function obj = MooringLine_Class(mooring_param, p0, p1)
            obj.mooring_param = mooring_param;
            obj.p0 = p0;
            obj.p1 = p1;
            
            %��ҪĿ���ǻ��ê������Y_N, ����Y_Nȥ������̬����
            [TE_begin, TE_end, Y_N, p_final, p_init0] = mooring_line_static_fcn(obj, p0, p1);
            obj.Y_N = Y_N;
            obj.line_shape_static = p_final;
        end
        

        function [obj, TR1, TRn, p_final] = get_mooring_force_dynamic(obj, p0_next, p1_next)
            Y_N = obj.Y_N;
            obj.p0_next;    obj.p1_next;
            
            % ���㶯̬��
            [TR1, TRn, Y_N, p_final] = mooring_line_dynamic_fcn(obj, p0_next, p1_next, Y_N);
            obj.Y_N = Y_N;
            obj.TR1 = TR1;  obj.TRn = TRn;
            obj.TR1_his = [obj.TR1_his, TR1];
            obj.TRn_his = [obj.TRn_his, TRn];
            
        end
    end
end

