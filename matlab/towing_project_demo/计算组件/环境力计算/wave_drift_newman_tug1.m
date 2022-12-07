function tau_wave2 = wave_drift_newman_tug1(pose, current_time, wave_parameters)

% pose = [1,2, 60/180*pi]';
% wave_parameters = struct('Hs', 5.27, 'Tp', 10.4, 'gamma', 3.3, 'env_dir', 0/180*pi, 'nfr', 100);
% current_time = 0;

load('qtf_tug.mat');
% dir = -5:5:190;
% fre = 0.01:0.03:1.48;
% dir_num = length(dir);
% fre_num = length(fre);

fre_num = 49;
dir_num = 40;
fre = 0.03:0.03:1.47;
dir = -5:5:190;
fre_diff = 0:0.03:1.47;
fre_diff_num = length(fre_diff);

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

drift_loadx = complex(0, 0);
drift_loady = complex(0, 0);
drift_loadfai = complex(0, 0);
drift_loadx_helper = complex(0, 0);
drift_loady_helper = complex(0, 0);
drift_loadfai_helper = complex(0, 0);

U = 0;
x = pose(1);
y = pose(2);
fai = pose(3);
for n = 1 : length(wave_fre)
    xpii = qtf_xpii_interp(n);
    ypii = qtf_ypii_interp(n);
    faipii = qtf_faipii_interp(n);
    
    %这个遭遇角没用到，因为速度小可以忽略，不用是担心插值次数过多，影响计算速度
    direction_e = attack_alpha + 2 * wave_fre(n)/9.81*U*sin(attack_alpha);
    Aex = 1-4*wave_fre(n)/9.81*U*cos(attack_alpha);
    Aey = Aex;
    Aefai = 1;
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
        Px_sqrt = complex(sqrt(Px), 0);
    else
        Px_sqrt = complex(0,sqrt(abs(Px)));
    end
    
    if Py>=0
        Py_sqrt = complex(sqrt(Py), 0);
    else
        Py_sqrt = complex(0,sqrt(abs(Py)));
    end
    
    if Pfai>=0
        Pfai_sqrt = complex(sqrt(Pfai), 0);
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
tau_wave2;
end