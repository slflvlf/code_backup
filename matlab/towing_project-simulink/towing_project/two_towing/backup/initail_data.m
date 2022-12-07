load("Initialization_data.mat");


%% 拖船参数
M1 = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
Ma1 = [7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5, 8.77e6];
MM = M1 + Ma1;
D1 = [MM(1,1)/100, 0, 0; 0, MM(2,2)/40, 0; 0, 0, MM(3,3)/20];
clear MM;

%% 各船的初始位置
p00 = [0, 0, 0]';
p10 = [397, 237, 31]';
p20 = [397, -237, -31]';

save("Initialization_data.mat");

