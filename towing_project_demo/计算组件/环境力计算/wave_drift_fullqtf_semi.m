function tau_wave2 = wave_drift_fullqtf_semi(pose, current_time, wave_parameters)
% function tau_wave2 = wave_drift()
load('qtf_semi.mat');
% pose = [1,2, 60/180*pi]';
% wave_parameters = struct('Hs', 5.27, 'Tp', 10.4, 'gamma', 3.3, 'env_dir', 0/180*pi);
% current_time = 0;

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




[X, Y, Z] = meshgrid(fre, fre, dir);
[Xq,Yq, Zq] = meshgrid(wave_fre, wave_fre, abs_attack_alpha*ones(length(dir), 1)/pi*180);

qtf_xp_interp = interp3(X,Y,Z,qtf_xp,Xq,Yq, Zq);
qtf_xp_interp = qtf_xp_interp(:, :, 1);
qtf_xq_interp = interp3(X,Y,Z,qtf_xq,Xq,Yq, Zq);
qtf_xq_interp = qtf_xq_interp(:, :, 1);

qtf_yp_interp = interp3(X,Y,Z,qtf_yp, Xq,Yq, Zq);
qtf_yp_interp = qtf_yp_interp(:, :, 1);
qtf_yq_interp = interp3(X,Y,Z,qtf_yq, Xq,Yq, Zq);
qtf_yq_interp = qtf_yq_interp(:, :, 1);

qtf_faip_interp = interp3(X,Y,Z,qtf_faip, Xq,Yq, Zq);
qtf_faip_interp = qtf_faip_interp(:, :, 1);
qtf_faiq_interp = interp3(X,Y,Z,qtf_faiq, Xq,Yq, Zq);
qtf_faiq_interp = qtf_faiq_interp(:, :, 1);


drift_loadx = 0;
drift_loady = 0;
drift_loadfai = 0;
U = 0;
x = pose(1);
y = pose(2);
fai = pose(3);
for n = 1 : length(wave_fre)
    for m = n:length(wave_fre)
       if  n==m
           drift_loadx = drift_loadx + wave_amp(n)^2 * qtf_xp_interp(n, m);
           drift_loady = drift_loady + wave_amp(n)^2 * qtf_yp_interp(n, m);
           drift_loadfai = drift_loadfai + wave_amp(n)^2 * qtf_faip_interp(n, m);
       else
%            amp1 = 2 * wave_amp(n) * wave_amp(m) * qtf_xp_interp(n, m);
%            amp2 = 2 * wave_amp(n) * wave_amp(m) * qtf_xq_interp(n, m);
%            we_n = wave_fre(n)-wave_fre(n)^2*U/9.81*cos(attack_alpha);
%            we_m = wave_fre(m)-wave_fre(m)^2*U/9.81*cos(attack_alpha);
%            
%            drift_loadx = drift_loadx ...
%                + amp1*cos((abs(wave_fre(n)-we_n)-abs(wave_fre(m)-we_m))*current_time...
%                -(wave_num(n)-wave_num(m))*(x*cos(env_dir)+y*sin(env_dir))...
%                + (wave_phase(n)-wave_phase(m)) )...
%                + amp2 * sin( ( abs(wave_fre(n)-we_n)-abs(wave_fre(m)-we_m))*current_time...
%                -(wave_num(n)-wave_num(m))*(x*cos(env_dir)+y*sin(env_dir))...
%                + (wave_phase(n)-wave_phase(m)));
           
           taux = drift_temp(wave_fre, wave_num, wave_amp, wave_phase, ...
                    qtf_xp_interp, qtf_xq_interp, current_time, pose, env_dir, m, n);
           drift_loadx = drift_loadx + taux;
           
           tauy = drift_temp(wave_fre, wave_num, wave_amp, wave_phase, ...
                    qtf_yp_interp, qtf_yq_interp, current_time, pose, env_dir, m, n);
           drift_loady = drift_loady + tauy;
           
           taufai = drift_temp(wave_fre, wave_num, wave_amp, wave_phase, ...
                    qtf_faip_interp, qtf_faiq_interp, current_time, pose, env_dir, m, n);
           drift_loadfai = drift_loadfai + taufai;
           
       end
       
       
           
               
    end
       
        
end

% end
    



drift_load = [drift_loadx, drift_loady, drift_loadfai]';

tau_wave2 = drift_load;



if attack_alpha<0
    tau_wave2(2) = -tau_wave2(2);
    tau_wave2(3) = -tau_wave2(3);
end

tau_wave2 = tau_wave2/1e3; %转为KN

end


function tau = drift_temp(wave_fre, wave_num, wave_amp, wave_phase, ...
    qtf_xp_interp, qtf_xq_interp, current_time, pose, env_dir, m, n)
    U = 0;
    x = pose(1);
    y = pose(2);
    fai = pose(3);
    
    attack_alpha = env_dir - fai;
    attack_alpha = atan2(sin(attack_alpha), cos(attack_alpha));% 攻角转换为（-pi, pi)
    abs_attack_alpha = abs(attack_alpha);
    
    amp1 = 2 * wave_amp(n) * wave_amp(m) * qtf_xp_interp(n, m);
    amp2 = 2 * wave_amp(n) * wave_amp(m) * qtf_xq_interp(n, m);
    we_n = wave_fre(n)-wave_fre(n)^2*U/9.81*cos(attack_alpha);
    we_m = wave_fre(m)-wave_fre(m)^2*U/9.81*cos(attack_alpha);
           
           
    tau = amp1*cos((abs(we_n)-abs(we_m))*current_time...
          -(wave_num(n)-wave_num(m))*(x*cos(env_dir)+y*sin(env_dir))...
          + (wave_phase(n)-wave_phase(m)) )...
          + amp2 * sin( ( abs(we_n)-abs(we_m))*current_time...
          -(wave_num(n)-wave_num(m))*(x*cos(env_dir)+y*sin(env_dir))...
          + (wave_phase(n)-wave_phase(m)));
           

end




    