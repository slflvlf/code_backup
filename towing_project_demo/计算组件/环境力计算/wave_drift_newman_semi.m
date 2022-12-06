function tau_wave2 = wave_drift_newman_semi(pose, current_time, wave_parameters)

persistent data;
if isempty(data)
    data = load('qtf_semi.mat');
end

dir = data.dir;
dir_num = data.dir_num;
fre = data.fre;
fre_num = data.fre_num;
qtf_faip = data.qtf_faip;
qtf_faipii = data.qtf_faipii;
qtf_faiq = data.qtf_faiq;
qtf_xp = data.qtf_xp;
qtf_xpii = data.qtf_xpii;
qtf_xq = data.qtf_xq;
qtf_yp = data.qtf_yp;
qtf_ypii = data.qtf_ypii;
qtf_yq = data.qtf_yq;





% pose = [1,2, 60/180*pi]';
% wave_parameters = struct('Hs', 5.27, 'Tp', 10.4, 'gamma', 3.3, 'env_dir', 0/180*pi, 'nfr', 100);
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
        xpjj = qtf_xpii_interp(m); 
        ypjj = qtf_ypii_interp(m);  
        faipjj = qtf_faipii_interp(m);  
        %近似方法1
        xpij = (xpii+xpjj)/2;
        ypij = (ypii+ypjj)/2;
        faipij = (faipii+faipjj)/2;
%         %近似方法2
%         xpij = sqrt(abs(xpii*xpjj));
%         ypij = sqrt(abs(ypii*ypjj));
%         faipij = sqrt(abs(faipii*faipjj));
        
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


tau_wave2;

end



