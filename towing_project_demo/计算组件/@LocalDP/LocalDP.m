classdef LocalDP
    %UNTITLED 此处提供此类的摘要
    %   此处提供详细说明

    properties
        p;  v;  p_hat;  v_hat
        p_his;  v_his;  p_hat_his;  v_hat_his;

        tau;    tau_his;

        M;  D; % M包括Ma

        step_time;  control_method;

        % 中间变量
        int_error; %误差积分
        counter;

        % 控制器增益
        Kp; Kd; Ki; % pid 控制器增益
        Gama;   Kp_bs;  Kd_bs;  Ki_bs;
        
    end

    methods
        %% 构造函数
        function obj = LocalDP(M, D, step_time, control_method)
            obj.M = M;  obj.D = D;
            obj.step_time = step_time;
            obj.control_method = control_method;
            
            obj.int_error = zeros(3, 1);
            counter = 0;

        end


        function obj = set_input(obj, pd, dot_pd)
            obj.pd = pd;
            obj.dot_pd = dot_pd;
        end


        %% 接口函数
        function obj = set_pid_gain(obj, Kp, Kd, Ki)
            obj.Kp = Kp;
            obj.Kd = Kd;
            obj.Ki = Ki;
        end

        function obj = set_pid_gain(obj)
            

        end

    end











end