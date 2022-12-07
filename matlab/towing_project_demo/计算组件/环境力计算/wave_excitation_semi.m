function tau_wave1 = wave_excitation_semi(pose, current_time, wave_parameters)


% load('force_rao');
persistent data;
if isempty(data)
    data = load('force_rao');
end

dir_semi = data.dir_semi;
dir_tug = data.dir_tug;
force_rao_semi = data.force_rao_semi;
force_rao_tug = data.force_rao_tug;
fre_semi = data.fre_semi;
fre_tug = data.fre_tug;



env_dir = wave_parameters.env_dir;
Hs = wave_parameters.Hs;
Tp = wave_parameters.Tp;
gamma = wave_parameters.gamma;
nfr= wave_parameters.nfr;

[wave_fre,wave_amp,wave_num,wave_phase]=jonspec_fixed(Hs,Tp,gamma,nfr);

% fx1 = textread('rao_fx.txt');
% fy1 = textread('rao_fy.txt');
% mz1 = textread('rao_mz.txt');
fx1_amp = force_rao_semi(:, :, 1);
fx1_phase = force_rao_semi(:, :, 2);
fy1_amp = force_rao_semi(:, :, 3);
fy1_phase = force_rao_semi(:, :, 4);
mz1_amp = force_rao_semi(:, :, 5);
mz1_phase = force_rao_semi(:, :, 6);


data_fre = fre_semi;
data_dir = dir_semi;
U = 0;

% tau_x = tau_dof(pose, fx1, data_fre, data_dir, env_dir, current_time, wave_fre,wave_amp,wave_num,wave_phase, U);
% tau_y = tau_dof(pose, fy1, data_fre, data_dir, env_dir, current_time, wave_fre,wave_amp,wave_num,wave_phase, U);
% tau_fai = tau_dof(pose, mz1, data_fre, data_dir, env_dir, current_time, wave_fre,wave_amp,wave_num,wave_phase, U);
tau_x = tau_dof(pose, fx1_amp, fx1_phase, data_fre, data_dir, env_dir, current_time, wave_fre,wave_amp,wave_num,wave_phase, U, 1);
tau_y = tau_dof(pose, fy1_amp, fy1_phase, data_fre, data_dir, env_dir, current_time, wave_fre,wave_amp,wave_num,wave_phase, U, 2);
tau_fai = tau_dof(pose, mz1_amp, mz1_phase, data_fre, data_dir, env_dir, current_time, wave_fre,wave_amp,wave_num,wave_phase, U, 3);
tau_wave1 = [tau_x, tau_y, tau_fai]';
tau_wave1 = tau_wave1/1e3;
end

function tau = tau_dof(pose, fx1_amp, fx1_phase, data_fre, data_dir, env_dir, current_time, wave_fre,wave_amp,wave_num,wave_phase, U, dof)
% function tau = tau_dof(pose, fx1, data_fre, data_dir, env_dir, current_time, wave_fre,wave_amp,wave_num,wave_phase, U)
    x = pose(1);
    y = pose(2);

    attack_alpha = env_dir - pose(3);
    
    
%     fx1_amp = fx1(:, 2:length(data_dir)+1);
%     fx1_phase = fx1(:, length(data_dir)+2:end);

    attack_alpha = atan2(sin(attack_alpha), cos(attack_alpha));% 攻角转换为（-pi, pi)
    abs_attack_alpha = abs(attack_alpha);

    [X, Y] = meshgrid(data_dir,data_fre);



    [Xq,Yq] = meshgrid(abs_attack_alpha*ones(length(data_dir))/pi*180, wave_fre);

    rao_amp = interp2(X,Y,fx1_amp,Xq,Yq);
    rao_amp = rao_amp(:, 1);

    rao_phase = interp2(X,Y,fx1_phase,Xq,Yq);
    rao_phase = rao_phase(:, 1);
    
    %% 这里没看明白
    if attack_alpha<0
        if dof==2||dof==3
            rao_phase  = rao_phase + 180;
        end
    end

    wave_excitation =  rao_amp.* wave_amp .* cos(abs(wave_fre-wave_fre.^2/9.81*U*cos(attack_alpha))*current_time...
    -wave_num.*(x*cos(env_dir)+y*sin(env_dir))+wave_phase+rao_phase/180*pi);

    wave_excitation_x = sum(wave_excitation);
    tau = wave_excitation_x;
    
end




