function [tau_wind, tau_current, tau_wave1, tau_wave2] = tau_env_semi_interface(pose, current_time, wind_param,... 
            current_param, wave_param)
% 计算semi的环境力       
% 输入参数：
% pose:           位姿（x, y, fai)
% current_time:   当前时间
% wind_param:     风参数
% current_param:  流参数
% wave_param:     浪参数
% 输出：
% 风浪流的载荷 KN

        
wind_speed = wind_param.wind_speed;
wind_angle = wind_param.wind_angle;

current_speed = current_param.current_speed;
current_angle = current_param.current_angle;

tau_wind = wind_load_semi(pose, wind_speed, wind_angle);
tau_current = current_load_semi(pose, current_speed, current_angle);
tau_wave1 = wave_excitation_semi(pose, current_time, wave_param);

% tau_wave2 = wave_drift_newman_semi(pose, current_time, wave_param);

% %自己写的newman近似方法
tau_wave2 = wave_drift_newman_semi(pose, current_time, wave_param);


%李博c++程序里的newman近似方法
% tau_wave2 = wave_drift_newman_tug(pose, current_time, wave_param);
end