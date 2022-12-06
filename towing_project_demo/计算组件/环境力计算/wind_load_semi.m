function tau_wind = wind_load_semi(pose, wind_speed, wind_angle)

wind_data = [0, pi / 6, pi / 4, pi / 3, pi / 2, pi / 3 * 2, pi / 4 * 3, pi / 6 * 5, pi;
            1e6, 1.078e6, 0.794e6, 0.692e6, -0.079e6, -0.916e6, -1.133e6, -1.196e6, -1.030e6;
            -0.0057E6, 0.688E6, 0.908E6, 1.047E6, 1.06E6, 0.873E6, 0.759E6, 0.474E6, -0.0292E6;
            -0.2E6, 4.524E6, 4.323E6, 3.086E6, 0.201E6, -0.917E6, 1.401E6, 2.853E6, 0.119E6];
wind_data_speed=27;

% wind_speed = 27; %真实风速
% wind_angle = 90*pi/180;
factor = (wind_speed/wind_data_speed)^2;

% pose = [1, 2, 0*pi/180]';
fai = pose(3);
attack_angle = wind_angle - fai;

attack_angle = atan2(sin(attack_angle), cos(attack_angle));% 攻角转换为（-pi, pi)
abs_attack_angle = abs(attack_angle);

taux = interp1(wind_data(1,:), wind_data(2,:), abs_attack_angle, 'spline')*factor;
tauy = interp1(wind_data(1,:), wind_data(3,:), abs_attack_angle, 'spline')*factor;
taufai = interp1(wind_data(1,:), wind_data(4,:), abs_attack_angle, 'spline')*factor;

tau_wind = [taux, tauy, taufai]'/1e3;

if attack_angle<0
    tau_wind(2) = -tau_wind(2);
    tau_wind(3) = -tau_wind(3);
end

end