function [R,Cv]=create_RotMat_CvMat(x_e)
% This function is used to create the rotation matrix R and the matrix of
% coriolis force Cv.
% 它们是时变的，只与seed state x_e相关！！！

% Input
% x_e       当前时刻的seed state [x y psi u v r]--------psi的单位是rad

% Output
% R        
% Cv

global M

% % test
% M=[21.6700000000000,0,0;0,39.0800000000000,0;0,0,14.5600000000000];
% x_e=[0 0 deg2rad(45) 1 2 0];

%Rotation matrix
R=[cos(x_e(3)) -sin(x_e(3)) 0;...
   sin(x_e(3)) cos(x_e(3))  0;...
   0           0            1     ];

%Coriolis and centripetal forces matrix
Cv=[0              0              -M(2,2)*x_e(5);...
    0              0              M(1,1)*x_e(4);...
    M(2,2)*x_e(5)  -M(1,1)*x_e(4) 0                  ];

end


