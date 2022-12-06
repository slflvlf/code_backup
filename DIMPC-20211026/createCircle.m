function reference=createCircle(center,radius)

% %test
% center=[0 0];
% radius=5;

dtheta=pi/90;
theta=-0.5*pi:dtheta:1.5*pi;
X=center(1)+radius*cos(theta);
Y=center(2)+radius*sin(theta);
PSI=0:dtheta:2*pi;
reference=[X;Y;PSI];

% 
% plot(X,Y)
% plot(rad2deg(PSI))
end