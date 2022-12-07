function tau_wave2 = wave_drift_fullqtf_tug(pose, current_time, wave_parameters)

load('qtf_tug.mat');

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
nfr = wave_parameters.nfr;

[wave_fre,wave_amp,wave_num,wave_phase]=jonspec_fixed(Hs,Tp,gamma,nfr);
wave_fre_num = length(wave_fre);
diff_matrix = zeros(wave_fre_num, wave_fre_num);
for i = 1:wave_fre_num
    for j = i:wave_fre_num
        diff_matrix(i,j) = -wave_fre(i) + wave_fre(j);
    end
end


attack_alpha = env_dir - pose(3);
attack_alpha = atan2(sin(attack_alpha), cos(attack_alpha));% 攻角转换为（-pi, pi)
abs_attack_alpha = abs(attack_alpha);

[X, Y, Z] = meshgrid(fre_diff, fre, dir);
% [Xq,Yq, Zq] = meshgrid(diff_matrix(1, :), wave_fre(1)*ones(length(wave_fre), 1), abs_attack_alpha*ones(length(dir), 1)/pi*180);
% qtf_xp_interp1 = interp3(X,Y,Z,qtf_xp,Xq,Yq, Zq);
% qtf_xp_interp = qtf_xp_interp1(1, :, 1);

drift_loadx = 0;
drift_loady = 0;
drift_loadfai = 0;
U = 0;
x = pose(1);
y = pose(2);
fai = pose(3);
for n = 1 : length(wave_fre)
    [Xq,Yq, Zq] = meshgrid(diff_matrix(n, :), wave_fre(n)*ones(length(wave_fre), 1), abs_attack_alpha*ones(length(dir), 1)/pi*180);
    qtf_xp_interp1 = interp3(X,Y,Z,qtf_xp,Xq,Yq, Zq);
    qtf_xp_interp = qtf_xp_interp1(1, :, 1);
    qtf_xq_interp1 = interp3(X,Y,Z,qtf_xq,Xq,Yq, Zq);
    qtf_xq_interp = qtf_xq_interp1(1, :, 1);
    
    qtf_yp_interp1 = interp3(X,Y,Z,qtf_yp,Xq,Yq, Zq);
    qtf_yp_interp = qtf_yp_interp1(1, :, 1);
    qtf_yq_interp1 = interp3(X,Y,Z,qtf_yq,Xq,Yq, Zq);
    qtf_yq_interp = qtf_yq_interp1(1, :, 1);
    
    qtf_faip_interp1 = interp3(X,Y,Z,qtf_faip,Xq,Yq, Zq);
    qtf_faip_interp = qtf_faip_interp1(1, :, 1);
    qtf_faiq_interp1 = interp3(X,Y,Z,qtf_faiq,Xq,Yq, Zq);
    qtf_faiq_interp = qtf_faiq_interp1(1, :, 1);
    
    for m = n:length(wave_fre)
       if  n==m
           drift_loadx = drift_loadx + wave_amp(n)^2 * qtf_xp_interp(m);
           drift_loady = drift_loady + wave_amp(n)^2 * qtf_yp_interp(m);
           drift_loadfai = drift_loadfai + wave_amp(n)^2 * qtf_faip_interp(m);
       else
           amp1x = 2 * wave_amp(n) * wave_amp(m) * qtf_xp_interp(m);
           amp2x = 2 * wave_amp(n) * wave_amp(m) * qtf_xq_interp(m);
           amp1y = 2 * wave_amp(n) * wave_amp(m) * qtf_yp_interp(m);
           amp2y = 2 * wave_amp(n) * wave_amp(m) * qtf_yq_interp(m);
           amp1fai = 2 * wave_amp(n) * wave_amp(m) * qtf_faip_interp(m);
           amp2fai = 2 * wave_amp(n) * wave_amp(m) * qtf_faiq_interp(m);
           
          
           we_n = wave_fre(n)-wave_fre(n)^2*U/9.81*cos(attack_alpha);
           we_m = wave_fre(m)-wave_fre(m)^2*U/9.81*cos(attack_alpha);
           phase_temp = (abs(we_n)-abs(we_m))*current_time-(wave_num(n)-wave_num(m))...
                        *(x*cos(env_dir)+y*sin(env_dir))+ (wave_phase(n)-wave_phase(m));
           
           drift_loadx = drift_loadx + amp1x*cos(phase_temp) + amp2x * sin(phase_temp);
           drift_loady = drift_loady + amp1y*cos(phase_temp) + amp2y * sin(phase_temp);
           drift_loadfai = drift_loadfai + amp1fai*cos(phase_temp) + amp2fai * sin(phase_temp);
           
       end
       
       
           
               
    end
       
        
end

tau_wave2 = [drift_loadx, drift_loady, drift_loadfai]';
% if attack_alpha<0
%     tau_wave2(2) = -tau_wave2(2);
%     tau_wave2(3) = -tau_wave2(3);
% end
% 
tau_wave2 = tau_wave2/1e3; %转为KN
end