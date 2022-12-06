%% 环境力接口测试 semi and tug
clear all;
current_time = 0;
pose = [1, 2, 60/180*pi]';
wind_param = struct('wind_speed', 27, 'wind_angle',  0/180*pi);
current_param = struct('current_speed', 0.65, 'current_angle', 0/180*pi);
wave_param = struct('Hs', 5.27, 'Tp', 10.4, 'gamma', 3.3, 'env_dir', 0/180*pi, 'nfr', 100);
N = 100;
[tau_wave1_tug_his, tau_wave2_tug_his, tau_wave1_semi_his, tau_wave2_semi_his] = deal(zeros(3, N));
tic
for i = 1 : N
    i
    current_time=i;
[tau_wind_tug, tau_current_tug, tau_wave1_tug, tau_wave2_tug] = tau_env_tug_interface(pose, current_time, wind_param,... 
            current_param, wave_param);
tau_wave1_tug_his(:, i) = tau_wave1_tug;
tau_wave2_tug_his(:, i) = tau_wave2_tug;

[tau_wind_semi, tau_current_semi, tau_wave1_semi, tau_wave2_semi] = tau_env_semi_interface(pose, current_time, wind_param,... 
            current_param, wave_param);
tau_wave1_semi_his(:, i) = tau_wave1_semi;
tau_wave2_semi_his(:, i) = tau_wave2_semi;
end
toc
figure(1)
title('tug')
subplot(3, 1, 1)
plot(tau_wave1_tug_his(1, :));
hold on
plot(tau_wave2_tug_his(1, :));
hold off
title('tug')

subplot(3, 1, 2)
plot(tau_wave1_tug_his(2, :));
hold on
plot(tau_wave2_tug_his(2, :));
hold off

subplot(3, 1, 3)
plot(tau_wave1_tug_his(3, :));
hold on
plot(tau_wave2_tug_his(3, :));
hold off

figure(2)
subplot(3, 1, 1)
plot(tau_wave1_semi_his(1, :));
hold on
plot(tau_wave2_semi_his(1, :));
hold off
title('semi')

subplot(3, 1, 2)
plot(tau_wave1_semi_his(2, :));
hold on
plot(tau_wave2_semi_his(2, :));
hold off

subplot(3, 1, 3)
plot(tau_wave1_semi_his(3, :));
hold on
plot(tau_wave2_semi_his(3, :));
hold off

figure(3)
subplot(1, 2, 1)
plot(tau_wave1_semi_his(2, :));
subplot(1, 2, 2)
plot(tau_wave2_semi_his(2, :));
title('semi')

figure(4)
subplot(1, 2, 1)
plot(tau_wave1_tug_his(2, :));
subplot(1, 2, 2)
plot(tau_wave2_tug_his(2, :));
title('tug')


%% tug 二阶力计算 full qtf
pose = [1,2, 60/180*pi]';
wave_parameters = struct('Hs', 5.27, 'Tp', 10.4, 'gamma', 3.3, 'env_dir', 0/180*pi, 'nfr', 100);
current_time = 0;

N= 500;
tau_wave2_his = zeros(3, N);
tic
for i = 1 : N
    i
tau_wave2 = wave_drift_tug(pose, i, wave_parameters)
tau_wave2_his(:, i) = tau_wave2;
end
tau_sum = [sum(tau_wave2_his(1,:)),sum(tau_wave2_his(2,:)),sum(tau_wave2_his(3,:))]
toc
figure(5)
subplot(3, 1, 1) 
plot(tau_wave2_his(1, :));
subplot(3, 1, 2)
plot(tau_wave2_his(2, :));
subplot(3, 1, 3)
plot(tau_wave2_his(3, :));

%% tug 二阶力计算 newman近似
pose = [1,2, 60/180*pi]';
wave_parameters = struct('Hs', 5.27, 'Tp', 10.4, 'gamma', 3.3, 'env_dir', 0/180*pi, 'nfr', 100);
current_time = 10;

N= 100;
tau_wave2_his = zeros(3, N);
tic
for i = 1 : N
    i
tau_wave2 = wave_drift_newman_tug(pose, i, wave_parameters)
tau_wave2_his(:, i) = tau_wave2;
end
tau_sum = [sum(tau_wave2_his(1,:)),sum(tau_wave2_his(2,:)),sum(tau_wave2_his(3,:))]
toc
figure(5)
subplot(3, 1, 1) 
plot(tau_wave2_his(1, :));
subplot(3, 1, 2)
plot(tau_wave2_his(2, :));
subplot(3, 1, 3)
plot(tau_wave2_his(3, :));

%% semi 二阶力计算 full qtf
clear all;
pose = [1, 2, 60/180*pi]';
wave_parameters = struct('Hs', 5.27, 'Tp', 10.4, 'gamma', 3.3, 'env_dir', 0/180*pi, 'nfr', 100);
current_time = 0;
N= 500;
tau_wave2_his = zeros(3, N);
tic
for i = 1 : N
    i
tau_wave2 = wave_drift_fullqtf_semi(pose, i, wave_parameters);
tau_wave2_his(:, i) = tau_wave2;
end
toc
figure(5)
subplot(3, 1, 1) 
plot(tau_wave2_his(1, :));
subplot(3, 1, 2)
plot(tau_wave2_his(2, :));
subplot(3, 1, 3)
plot(tau_wave2_his(3, :));


%% semi 二阶力计算 newman近似
pose = [1,2, 30/180*pi]';
wave_parameters = struct('Hs', 5.27, 'Tp', 10.4, 'gamma', 3.3, 'env_dir', 0/180*pi, 'nfr', 100);
current_time = 10;

N= 500;
tau_wave2_his = zeros(3, N);
tic
for i = 1 : N
    i
tau_wave2 = wave_drift_newman_semi(pose, i, wave_parameters)
tau_wave2_his(:, i) = tau_wave2;
end
tau_sum = [sum(tau_wave2_his(1,:)),sum(tau_wave2_his(2,:)),sum(tau_wave2_his(3,:))]
toc
figure(5)
subplot(3, 1, 1) 
plot(tau_wave2_his(1, :));
subplot(3, 1, 2)
plot(tau_wave2_his(2, :));
subplot(3, 1, 3)
plot(tau_wave2_his(3, :));

%% tug 一阶力计算
clear all;

pose = [1, 2, 0*pi/180]';
current_time = 1;
wave_parameters = struct('env_dir', 0*pi/180, 'Hs', 5.27, 'Tp', 10.4, 'gamma', 3.3, 'nfr', 100);
N = 500;
tau_his = zeros(3, N);
tic
for i = 1:N
    i
tau = wave_excitation_tug(pose, current_time+i, wave_parameters)
tau_his(:, i) = tau;
end
toc

figure(4)
subplot(3, 1, 1) 
plot(tau_his(1, :));
subplot(3, 1, 2)
plot(tau_his(2, :));
subplot(3, 1, 3)
plot(tau_his(3, :));


%% semi 一阶力计算
clear all;

pose = [1, 2, 0*pi/180]';
current_time = 0;
wave_parameters = struct('env_dir', 0*pi/180, 'Hs', 5.27, 'Tp', 10.4, 'gamma', 3.3, 'nfr', 100);
N = 500
tau_his = zeros(3, N);
tic
for i = 1:N
    i
tau = wave_excitation_semi(pose, i, wave_parameters);
tau_his(:, i) = tau;
end
toc
figure(3)
subplot(3, 1, 1) 
plot(tau_his(1, :));
subplot(3, 1, 2)
plot(tau_his(2, :));
subplot(3, 1, 3)
plot(tau_his(3, :));


%% 风力计算
%风力系数
wind_data = [0, pi / 6, pi / 4, pi / 3, pi / 2, pi / 3 * 2, pi / 4 * 3, pi / 6 * 5, pi;
            1e6, 1.078e6, 0.794e6, 0.692e6, -0.079e6, -0.916e6, -1.133e6, -1.196e6, -1.030e6;
            -0.0057e6, 0.688e6, 0.908e6, 1.047e6, 1.06e6, 0.873e6, 0.759e6, 0.474e6, -0.0292e6;
            -0.2e6, 4.524e6, 4.323e6, 3.086e6, 0.201e6, -0.917e6, 1.401e6, 2.853e6, 0.119e6]
wind_data_speed=27;

wind_speed = 27; %真实风速
wind_angle = 90*pi/180;
factor = (wind_speed/wind_data_speed)^2;

pose = [1, 2, 0*pi/180]';
fai = pose(3);
attack_angle = wind_angle - fai;

attack_angle = atan2(sin(attack_angle), cos(attack_angle));% 攻角转换为（-pi, pi)
abs_attack_angle = abs(attack_angle);

taux = interp1(wind_data(1,:), wind_data(2,:), abs_attack_angle, 'spline')*factor;
tauy = interp1(wind_data(1,:), wind_data(3,:), abs_attack_angle, 'spline')*factor;
taufai = interp1(wind_data(1,:), wind_data(4,:), abs_attack_angle, 'spline')*factor;

tau_wind = [taux, tauy, taufai]'/1e3

if attack_angle<0
    tau_wind(2) = -tau_wind(2);
    tau_wind(3) = -tau_wind(3);
end

%% 流力
clear all;clc;
current_data = [0, pi / 6, pi / 4, pi / 3, pi / 2, pi / 3 * 2, pi / 4 * 3, pi / 6 * 5, pi;
            0.274e6, 0.307e6, 0.2966e6, 0.121e6, -0.035e6, -0.1867e6, -0.274e6, -0.331e6, -0.243e6;
            -0.029e6, 0.411e6, 0.531e6, 0.595e6, 0.4726e6, 0.518e6, 0.473e6, 0.3475e6, -0.0113e6;
            -0.54e6, 2.394e6, 4.676e6, 5.461e6, -0.4148e6, -4.368e6, -4.512e6, -3.249e6, -0.113e6];
current_data_speed=0.65;
current_speed = 0.7; %真实风速
current_angle = 0*pi/180;
factor = (current_speed/current_data_speed)^2;

pose = [1, 2, 90*pi/180]';
fai = pose(3);
attack_angle = current_angle - fai;

attack_angle = atan2(sin(attack_angle), cos(attack_angle));% 攻角转换为（-pi, pi)
abs_attack_angle = abs(attack_angle);

taux = interp1(current_data(1,:), current_data(2,:), abs_attack_angle, 'spline')*factor;
tauy = interp1(current_data(1,:), current_data(3,:), abs_attack_angle, 'spline')*factor;
taufai = interp1(current_data(1,:), current_data(4,:), abs_attack_angle, 'spline')*factor;

tau_current = [taux, tauy, taufai]'/1e3;

%这里还要确定一下
if attack_angle<0
    tau_current(2) = -tau_current(2);
    tau_current(3) = -tau_current(3);
end
tau_current


%% semi二阶力计算 Newman近似
clear all
% function tau_wave2 = wave_drift_newman_semi(pose, current_time, wave_parameters)
load('qtf_semi.mat');
pose = [1,2, 60/180*pi]';
wave_parameters = struct('Hs', 5.27, 'Tp', 10.4, 'gamma', 3.3, 'env_dir', 0/180*pi, 'nfr', 100);
current_time = 0;


dir = -5:5:190;
fre = 0.01:0.03:1.48;
dir_num = length(dir);
fre_num = length(fre);
Hs = wave_parameters.Hs;
Tp = wave_parameters.Tp;
gamma = wave_parameters.gamma;
env_dir = wave_parameters.env_dir;
nfr= wave_parameters.nfr;

[wave_fre,wave_amp,wave_num,wave_phase]=jonspec_fixed(Hs,Tp,gamma,nfr);

attack_alpha = env_dir - pose(3);
attack_alpha = atan2(sin(attack_alpha), cos(attack_alpha));% 攻角转换为（-pi, pi)
abs_attack_alpha = abs(attack_alpha);

[qtf_xpii, qtf_ypii, qtf_faipii] = deal(zeros(fre_num, dir_num));
for i = 1:dir_num
    for j = 1 : fre_num
        qtf_xpii(j, i) = qtf_xp(j, j, i);
        qtf_ypii(j, i) = qtf_yp(j, j, i);
        qtf_faipii(j, i) = qtf_faip(j, j, i);
    end
end
[X, Y] = meshgrid(dir, fre);

[Xq,Yq] = meshgrid(abs_attack_alpha*ones(dir_num, 1)/pi*180, wave_fre);

qtf_xpii_interp1 = interp2(X,Y,qtf_xpii,Xq,Yq);
qtf_xpii_interp = qtf_xpii_interp1(:, 1);
qtf_ypii_interp1 = interp2(X,Y,qtf_ypii,Xq,Yq);
qtf_ypii_interp = qtf_ypii_interp1(:, 1);   
qtf_faipii_interp1 = interp2(X,Y,qtf_faipii,Xq,Yq);
qtf_faipii_interp = qtf_faipii_interp1(:, 1);

surf(X, Y, qtf_xpii)
for i = 1:length(wave_fre)
    hold on
plot3(abs_attack_alpha/pi*180,wave_fre(i), qtf_xpii_interp(i), 'o')
end
hold off

drift_loadx = 0;
drift_loady = 0;
drift_loadfai = 0;
U = 0;
x = pose(1);
y = pose(2);
fai = pose(3);
for n = 1 : length(wave_fre)
    xpii = qtf_xpii_interp(n);
    ypii = qtf_ypii_interp(n);
    faipii = qtf_faipii_interp(n);
    for m = n:length(wave_fre)
        xpjj = qtf_xpii_interp(m);  xpij = (xpii+xpjj)/2;
        ypjj = qtf_ypii_interp(m);  ypij = (ypii+ypjj)/2;
        faipjj = qtf_faipii_interp(m);  faipij = (faipii+faipjj)/2;
        
       if  n==m
           drift_loadx = drift_loadx + wave_amp(n)^2 * xpii;
           drift_loady = drift_loady + wave_amp(n)^2 * ypii;
           drift_loadfai = drift_loadfai + wave_amp(n)^2 * faipii;
       else
           amp1x = 2 * wave_amp(n) * wave_amp(m) * xpij;
           amp1y = 2 * wave_amp(n) * wave_amp(m) * ypij;
           amp1fai = 2 * wave_amp(n) * wave_amp(m) * faipij;
           
           we_n = wave_fre(n)-wave_fre(n)^2*U/9.81*cos(attack_alpha);
           we_m = wave_fre(m)-wave_fre(m)^2*U/9.81*cos(attack_alpha);
           phase_temp = (abs(we_n)-abs(we_m))*current_time-(wave_num(n)-wave_num(m))...
                        *(x*cos(env_dir)+y*sin(env_dir))+ (wave_phase(n)-wave_phase(m));
           
           drift_loadx = drift_loadx + amp1x*cos(phase_temp);
           drift_loady = drift_loady + amp1y*cos(phase_temp);
           drift_loadfai = drift_loadfai + amp1fai*cos(phase_temp);
           
       end
            
    end
       
end
% drift_loadx

tau_wave2 = [drift_loadx, drift_loady, drift_loadfai]';

if attack_alpha<0
    tau_wave2(2) = -tau_wave2(2);
    tau_wave2(3) = -tau_wave2(3);
end

tau_wave2 = tau_wave2/1e3; %转为KN


tau_wave2

figure
 plot(fre, qtf_xpii(:, 14))
 hold on
 plot(wave_fre, qtf_xpii_interp, 'o')
 hold off
 title('插值效果')
 

%% semi二阶力计算 Newman近似1
clear all
% function tau_wave2 = wave_drift_newman_semi(pose, current_time, wave_parameters)

pose = [1,2, 60/180*pi]';
wave_parameters = struct('Hs', 5.27, 'Tp', 10.4, 'gamma', 3.3, 'env_dir', 0/180*pi, 'nfr', 100);
current_time = 0;

load('qtf_semi.mat');
dir = -5:5:190;
fre = 0.01:0.03:1.48;
dir_num = length(dir);
fre_num = length(fre);
Hs = wave_parameters.Hs;
Tp = wave_parameters.Tp;
gamma = wave_parameters.gamma;
env_dir = wave_parameters.env_dir;
nfr= wave_parameters.nfr;

[wave_fre,wave_amp,wave_num,wave_phase]=jonspec_fixed(Hs,Tp,gamma,nfr);

attack_alpha = env_dir - pose(3);
attack_alpha = atan2(sin(attack_alpha), cos(attack_alpha));% 攻角转换为（-pi, pi)
abs_attack_alpha = abs(attack_alpha);

[qtf_xpii, qtf_ypii, qtf_faipii] = deal(zeros(fre_num, dir_num));
for i = 1:dir_num
    for j = 1 : fre_num
        qtf_xpii(j, i) = qtf_xp(j, j, i);
        qtf_ypii(j, i) = qtf_yp(j, j, i);
        qtf_faipii(j, i) = qtf_faip(j, j, i);
    end
end
[X, Y] = meshgrid(dir, fre);

[Xq,Yq] = meshgrid(abs_attack_alpha*ones(dir_num, 1)/pi*180, wave_fre);

qtf_xpii_interp1 = interp2(X,Y,qtf_xpii,Xq,Yq);
qtf_xpii_interp = qtf_xpii_interp1(:, 1);
qtf_ypii_interp1 = interp2(X,Y,qtf_ypii,Xq,Yq);
qtf_ypii_interp = qtf_ypii_interp1(:, 1);   
qtf_faipii_interp1 = interp2(X,Y,qtf_faipii,Xq,Yq);
qtf_faipii_interp = qtf_faipii_interp1(:, 1);

surf(X, Y, qtf_xpii)
for i = 1:length(wave_fre)
    hold on
plot3(abs_attack_alpha/pi*180,wave_fre(i), qtf_xpii_interp(i), 'o')
end
hold off

drift_loadx = 0+0i;
drift_loady = 0+0i;
drift_loadfai = 0+0i;
drift_loadx_helper = 0+0i;
drift_loady_helper = 0+0i;
drift_loadfai_helper = 0+0i;

U = 0;
x = pose(1);
y = pose(2);
fai = pose(3);
for n = 1 : length(wave_fre)
    xpii = qtf_xpii_interp(n);
    ypii = qtf_ypii_interp(n);
    faipii = qtf_faipii_interp(n);
    
    direction_e = attack_alpha + 2 * wave_fre(n)/9.81*U*sin(attack_alpha);
    Aex = 1-4*wave_fre(n)/9.81*U*cos(attack_alpha);
    Aey = Aex;
    Aefai = Aex;
    if Aex<0    
        Aex = 0; 
    end
    if Aey<0
        Aey = 0;
    end
    Px = Aex*xpii;
    Py = Aey*ypii;
    Pfai = Aefai*faipii;
    if Px>=0
        Px_sqrt = sqrt(Px)+0i;
    else
        Px_sqrt = complex(0,sqrt(abs(Px)));
    end
    
    if Py>=0
        Py_sqrt = sqrt(Py)+0i;
    else
        Py_sqrt = complex(0,sqrt(abs(Py)));
    end
    
    if Pfai>=0
        Pfai_sqrt = sqrt(Pfai)+0i;
    else
        Pfai_sqrt = complex(0,sqrt(abs(Pfai)));
    end
    
    amp1x = wave_amp(n) * Px_sqrt;
    amp1y = wave_amp(n) * Py_sqrt;
    amp1fai = wave_amp(n) * Pfai_sqrt;
           
    we_n = wave_fre(n)-wave_fre(n)^2*U/9.81*cos(attack_alpha);

    phase_temp = abs(we_n)*current_time-wave_num(n)...
                        *(x*cos(env_dir)+y*sin(env_dir))+ wave_phase(n);
                    
    drift_loadx = drift_loadx+amp1x*cos(phase_temp);
    drift_loadx_helper = drift_loadx_helper+amp1x*sin(phase_temp);
    drift_loady = drift_loady+amp1y*cos(phase_temp);
    drift_loady_helper = drift_loady_helper+amp1y*sin(phase_temp);
    drift_loadfai = drift_loadfai+amp1fai*cos(phase_temp);
    drift_loadfai_helper = drift_loadfai_helper+amp1fai*sin(phase_temp);
    
end

taux = real(drift_loadx^2+drift_loadx_helper^2);
tauy = real(drift_loady^2+drift_loady_helper^2);  
taufai = real(drift_loadfai^2+drift_loadfai_helper^2);
tau_wave2 = [taux, tauy, taufai]'/1e3;

if attack_alpha<0
    tau_wave2(2) = -tau_wave2(2);
    tau_wave2(3) = -tau_wave2(3);
end
tau_wave2
    
        
    
    
    




figure
 plot(fre, qtf_xpii(:, 14))
 hold on
 plot(wave_fre, qtf_xpii_interp, 'o')
 hold off
 title('插值效果')



