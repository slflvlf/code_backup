function [dist_ship_obs]=dist_obstacle(shipPos,obsPos)
%this funciton is used to calculate the distance between ship and obstacle

step=size(shipPos,2);
dist_ship_obs=zeros(1,step);
for i=1:step
    dist_ship_obs(i)=sqrt((shipPos(1,i)-obsPos(1))^2+(shipPos(2,i)-obsPos(2))^2);%Œª÷√æ‡¿Î 
end
end