clear all; clc;
x0 = [0, 0, 0]';
r = [4, 1, 0]';
obstacle = [1, 4, 0]';
KG = 3;
KL = 10;
DL = 5;
L = 0.1;
x = [1, 1, 0]';
% [fai, fai_att, fai_rep, delta_fai, delta_fai_att, delta_fai_rep] = potential(x, r, obstacle)
[xgrid, ygrid] = meshgrid(0:0.1:10, 0:0.1:12);
[fai_his, fai_att_his, fai_rep_his] = deal(zeros(size(xgrid, 1), size(xgrid, 2)));
[delta_fai_att_x_his, delta_fai_x_his, delta_fai_rep_x_his] = deal(zeros(size(xgrid, 1), size(xgrid, 2)));
[delta_fai_att_y_his, delta_fai_y_his, delta_fai_rep_y_his] = deal(zeros(size(xgrid, 1), size(xgrid, 2)));
for i = 1 : length(xgrid(1,:))
    for j = 1 : length(ygrid(:, 1))
        x = [xgrid(1, i), ygrid(j, 1)];
        [fai, fai_att, fai_rep, delta_fai, delta_fai_att, delta_fai_rep] = potential(x, r, obstacle, KG, KL, DL);
        fai_his(j, i) = fai;
        fai_att_his(j, i) = fai_att;
        fai_rep_his(j, i) = fai_rep;
        delta_fai_x_his(j, i) = delta_fai(1);
        delta_fai_y_his(j, i) = delta_fai(2);
        delta_fai_att_x_his(j, i) = delta_fai_att(1);
        delta_fai_att_y_his(j, i) = delta_fai_att(2);
        delta_fai_rep_x_his(j, i) = delta_fai_rep(1);
        delta_fai_rep_y_his(j, i) = delta_fai_rep(2);
    end
end
figure(1)
subplot(1, 3, 1)
surf(xgrid, ygrid, fai_att_his)
% hold on 
% plot(r(1), r(2), 
subplot(1, 3, 2)
surf(xgrid, ygrid, fai_rep_his)
subplot(1, 3, 3)
surf(xgrid, ygrid, fai_his)

figure(2)
quiver(xgrid,ygrid,delta_fai_x_his, delta_fai_y_his);

startx = 0:1:8;
starty = ones(size(startx));
streamline(xgrid, ygrid, delta_fai_x_his, delta_fai_y_his, startx, starty)
        
%%
clear all; clc;
x = [0; 0];
r = [4; 5];
obstacle = [2; 2];
L = 0.01;
N = 100;

KG = 30;
KL = 10;
DL = 10;

x_his = zeros(2, N);
for i = 1 : N
    x_his(:, i) = x;
    [fai, fai_att, fai_rep, delta_fai, delta_fai_att, delta_fai_rep] = potential(x, r, obstacle, KG, KL, DL);
    x = [delta_fai(1)*L; delta_fai(2)*L] + x;
end
plot(x_his(1, :), x_his(2, :));        

%%
function [fai, fai_att, fai_rep, delta_fai, delta_fai_att, delta_fai_rep] = potential(x, r, obstacle, KG, KL, DL)


dr = sqrt((x(1)- r(1))^2 + ((x(2)- r(2))^2 ));
do = sqrt((x(1)- obstacle(1))^2 + ((x(2)- obstacle(2))^2 ));

fai_att = 1/2 * KG * dr^2;


if do < DL
    fai_rep = 1/2 * KL * dr * (1/do - 1/DL)^2;
else
    fai_rep = 0;
end

fai = fai_att + fai_rep;


delta_fai_att = KG * [x(1) - r(1); x(2) - r(2)];

if do < DL
delta_fai_rep = 1/2 * KL * (1/do - 1/DL)^2 * 1/dr * [x(1) - r(1); x(2) - r(2)]...
    -KL * dr * (1/do - 1/DL) * 1/do^3 * [x(1) - obstacle(1); x(2) - obstacle(2)];
else
    delta_fai_rep = [0; 0];
end

delta_fai = delta_fai_att + delta_fai_rep;
end




