function tau_current = current_load_semi(pose, current_speed, current_angle)

current_data = [0, pi / 6, pi / 4, pi / 3, pi / 2, pi / 3 * 2, pi / 4 * 3, pi / 6 * 5, pi;
            0.274e6, 0.307e6, 0.2966e6, 0.121e6, -0.035e6, -0.1867e6, -0.274e6, -0.331e6, -0.243e6;
            -0.029e6, 0.411e6, 0.531e6, 0.595e6, 0.4726e6, 0.518e6, 0.473e6, 0.3475e6, -0.0113e6;
            -0.54e6, 2.394e6, 4.676e6, 5.461e6, -0.4148e6, -4.368e6, -4.512e6, -3.249e6, -0.113e6];
current_data_speed=0.65;
% current_speed = 0.7; %真实风速
% current_angle = 0*pi/180;%真实角度
factor = (current_speed/current_data_speed)^2;

% pose = [1, 2, 90*pi/180]';
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
end