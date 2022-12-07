clear all; close all;
z = 0.9;     % relative damping ratio
w = 0.005;     % natural frequency
delta = 1; % nonlinear damping coeff.
vmax = 1;  % max velocity
h = 0.5;   % sampling time
N = 1000;   % number of samples

t  = 0:h:h*N;

% linear mass-damper-spring system
r1 = 100*ones(max(size(t)),1);
[A,B,C,D] = ord2(w,z); 
[x1,y1]   = lsim(A,B,C,D,r1,t); 

plot(t, y1(:, 1));




