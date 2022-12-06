classdef Local_DP
    % DP_CONTROLLER 针对NSB（零空间行为控制）写的localDP
    %   里面因为牵扯到滤波的事情，所以就用类的方式写，
    % 输入变量：Un, Xn, 
    % 输出变量：pose, vel, tau

%% 变量    
    properties
        % 船舶主要参数 M，D
        M; 
        D;


        step_time;

        %输入变量
        Un; Un_hat; dot_Un_hat; 

        Xn; Xn_hat; dot_Xn_hat; ddot_Xn_hat;
        dot_Xn_hat_last;


        %输出变量
        tau;

        % 船舶状态
        pose; 
        vel;
        
        % 船舶初始状态
        pose0;  vel0;

        X;%航向角
        fai; fai_hat; dot_fai_hat;%艏向角
        fai_n; dot_fai_n; ddot_fai_n;
        beta; beta_hat; dot_beta_hat; beta0; beta_n; %漂角
        ddot_beta_hat; dot_beta_hat_last;


        % 控制器增益
        K1 = 1; 
        K2 = diag([1, 1, 1])*1;

        % 滤波增益
        alpha = 0.15; 
        belta = 0.005;


        % 计数
        counter;

        % 记录变量
        tau_his; pose_his; vel_his;

    end


 %% 方法   
    methods
        %% 构造函数，初始化
        function obj = Local_DP(M, D, pose0, vel0, step_time)
            %DP_CONTROLLER 构造此类的实例

            obj.counter = 0;

            obj.M = M;  obj.D = D;
            obj.pose0 = pose0;  obj.vel0 = vel0;
            obj.pose = pose0;   obj.vel = vel0;
            obj.fai =pose0(3);
            obj.fai_hat = obj.fai;

            obj.pose_his = pose0';
            obj.vel_his = vel0';

            obj.tau = zeros(3, 1);
            

            obj.step_time = step_time;
            obj.beta0 = atan2(obj.vel(2), obj.vel(1));
            obj.beta = obj.beta0;

            obj.X = obj.fai + obj.beta;

            obj.K1 = 1; 
            obj.K2 = diag([1, 1, 1])*1;
        end


        %% 输入 函数
        function obj = set_ipunt(obj, Un, Xn, pose, vel)
            % 设定输入, U速度，Xn：航向角（大地坐标系下）
            obj.Un = Un;
            obj.Xn = Xn;
            obj.pose = pose;
            obj.vel = vel;
            obj.fai = pose(3);
           

            obj.counter = obj.counter + 1;

            obj.beta = atan2(vel(2), vel(1));

            obj.X = obj.fai + obj.beta;

            % 计算滤波
            if obj.counter == 1
                obj.Un_hat = Un;
                obj.dot_Un_hat = 0;

                obj.Xn_hat = Xn;
                obj.dot_Xn_hat = 0;
                obj.dot_Xn_hat_last = obj.dot_Xn_hat;
                obj.ddot_Xn_hat = 0;

                obj.beta_hat = obj.beta;
                obj.dot_beta_hat = 0;
                obj.dot_beta_hat_last = obj.dot_beta_hat;
                obj.ddot_beta_hat = 0;


                obj.fai_hat = obj.fai;
                obj.dot_fai_hat = 0;


            else
                [obj.Un_hat, obj.dot_Un_hat] = alpha_beta_filter1(obj, Un, obj.Un_hat, obj.dot_Un_hat, ...
                    0.15, 0.005, obj.step_time);

                [obj.Xn_hat, obj.dot_Xn_hat] = alpha_beta_filter1(obj, Xn, obj.Xn_hat, obj.dot_Xn_hat, ...
                    0.15, 0.005, obj.step_time);

                obj.ddot_Xn_hat = (obj.dot_Xn_hat - obj.dot_Xn_hat_last) / obj.step_time;
                obj.dot_Xn_hat_last = obj.dot_Xn_hat;


                [obj.beta_hat, obj.dot_beta_hat] = alpha_beta_filter1(obj, obj.beta, obj.beta_hat, ...
                    obj.dot_beta_hat, 0.15, 0.005, obj.step_time);
                obj.ddot_beta_hat = (obj.dot_beta_hat - obj.dot_beta_hat_last) / obj.step_time;
                obj.dot_beta_hat_last = obj.dot_beta_hat;


                [obj.fai_hat, obj.dot_fai_hat] = alpha_beta_filter1(obj, obj.fai, obj.fai_hat, ...
                    obj.dot_fai_hat, 0.5, 0.05, obj.step_time);
              
            end
            

            obj.fai_n = obj.Xn - obj.beta;
            obj.dot_fai_n = obj.dot_Xn_hat - obj.dot_beta_hat;
            obj.ddot_fai_n = obj.ddot_Xn_hat - obj.ddot_beta_hat;
             
        end



        %% 输出
        function [tau, pose, vel] = get_output(obj)
            
            tau = obj.tau;
            pose = obj.pose;
            vel = obj.vel;
        end

        function [tau_his, pose_his, vel_his] = get_history(obj)
            tau_his = obj.tau_his;
            pose_his = obj.pose_his;
            vel_his = obj.vel_his;
        end



        %% 主计算函数，求解控制力,零空间行为控制
        function obj = dp_controller(obj)
            obj.beta = atan2(obj.vel(2), obj.vel(1));
            
            obj.X = obj.pose(3) + obj.beta;
            z1 = obj.X - obj.Xn; 
            obj.beta_n = obj.Xn - obj.pose(3); %理想漂角
            
%             beta_n = obj.beta_n;
% 
%             Un = obj.Un

            alpha1 = obj.Un * cos(obj.beta_n);
            alpha2 = obj.Un * sin(obj.beta_n);
            alpha3 = obj.dot_fai_hat - z1;

            alpha = [alpha1; alpha2; alpha3];
            


            z2 = obj.vel - alpha;


            dot_alpha1 = obj.dot_Un_hat * cos(obj.beta_n) - obj.Un * sin(obj.beta_n) * (obj.dot_Xn_hat + obj.dot_fai_hat);
            dot_alpha2 = obj.dot_Un_hat * sin(obj.beta_n) + obj.Un * cos(obj.beta_n) * (obj.dot_Xn_hat - obj.dot_fai_hat);
            dot_alpha3 = obj.ddot_fai_n - (obj.dot_fai_hat - obj.dot_fai_n);
            dot_alpha = [dot_alpha1; dot_alpha2; dot_alpha3];

            
            % 增益待定
%             K1 = 0.11; 
%             K2 = diag([0.2, 0.2, 0.2])*0.1;

            K1 = obj.K1; 
            K2 = obj.K2;
            

            C = C_matrix(obj, obj.M, obj.vel);
            tau = obj.M * dot_alpha + (C + obj.D) * alpha - K1 * z1 * [0, 0, 1]'  - K2 * z2;
            
            obj.tau = tau;
            if obj.counter == 1
                obj.tau_his = obj.tau';
            else
                obj.tau_his = [obj.tau_his; obj.tau'];
            end
            
        end



        function obj = pid_controller(obj)
            
        end

        
        %% 运动方程求解, 单步模拟
        function obj = one_step_simulation(obj)
            %
            obj = obj.dp_controller();

            [obj.pose, obj.vel] = ship_dynamic_solver(obj, obj.pose, obj.vel, obj.tau, obj.M, obj.D);
            obj.pose_his = [obj.pose_his; obj.pose'];
            obj.vel_his = [obj.vel_his; obj.vel'];

            
        end

        
        %% 控制增益设置，接口
        function obj = set_gain(obj, K1, K2)
            obj.K1 = K1; 
            obj.K2 = K2;          
        end

        function obj = set_filter_gain(obj, alpha, belta)
            obj.alpha = alpha;
            obj.belta = belta;
        end




        %% 滤波器
        function [x_hat, dot_x_hat] = alpha_beta_filter1(obj, x, x_hat, dot_x_hat, alpha, beta, step_time)
            % alpha-beta 滤波, x是测量值，x_hat是估计值
            x_hat_temp = x_hat + dot_x_hat * obj.step_time;
            x_error = x - x_hat_temp;
            x_hat = x_hat_temp + alpha * step_time * x_error;
            dot_x_hat  = dot_x_hat + beta / step_time * x_error;
        end


        
        %% 转换函数
        function R = rotate_matrix(obj, fai)
            R = [cos(fai), -sin(fai), 0;
                sin(fai), cos(fai), 0;
                0, 0, 1];
        end

        %% 科氏力
        function C = C_matrix(obj, M, v)
            C = [0, 0, -M(2, 2) * v(2) - M(2, 3) * v(3);
                0, 0, M(1, 1) * v(1);
                M(2, 2) * v(2) + M(3, 2) * v(3), -M(1, 1) * v(1), 0];
        end
        
        


        %% 船舶运动微分方程
        function [p, v] = ship_dynamic_solver(obj, p0, v0, tau, M, D)
            tau = tau; %输入的是N
    
            y0 = [p0; v0];
            [t, y] = ode45(@(t, y) ship_dynamic(obj, t, y, tau, M, D), [0: 0.5: 1], y0);
    
            p = y(end, 1:3)';
            v = y(end, 4:6)';
        end



        function dydt = ship_dynamic(obj, t, y, tau, M, D)

        %     global MM1 D1 
            R = rotate_matrix(obj, y(3));
            C = C_matrix(obj, M, y(4:6));
            dydt = zeros(6, 1);
            dydt(1:3) = R * y(4:6);
            dydt(4:6) = inv(M) * (tau - C * y(4:6) - D * y(4:6));
 
        end
        
    end
end

