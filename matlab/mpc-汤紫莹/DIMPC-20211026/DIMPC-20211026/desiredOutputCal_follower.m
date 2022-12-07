function [Yd_follower]=desiredOutputCal_follower(Yd_leader,range,bearing,flag)

%Input
%Yd_leader    leader的期望位置和艏向角 也就是参考轨迹
%range        leader和follower之间的期望距离
%bearing      leader和follower之间形成编队的角度
%flag         flag==1 leftfo;flag==2 rightfo

% Output
% Yd_follower  follower的期望位置和艏向角

global Np;
Yd_follower=zeros(3,Np);
if flag==1 %计算左边follower
    for i=1:Np
    psi=Yd_leader(3,i);%leader的期望艏向角 rad
        if psi>=0 && psi<=pi 
            phi=psi+bearing-pi*0.5;      
            Yd_follower(:,i)=Yd_leader(:,i)+[-range*cos(phi);-range*sin(phi);0];
        else
            phi=psi+bearing-pi*1.5;
            Yd_follower(:,i)=Yd_leader(:,i)+[range*cos(phi);range*sin(phi);0];
        end
    end
elseif flag==2 %计算右边follower
    for i=1:Np
    psi=Yd_leader(3,i);%leader的期望艏向角 rad
        if psi>=0 && psi<=pi 
            phi=pi*0.5-psi+bearing;      
            Yd_follower(:,i)=Yd_leader(:,i)+[range*cos(phi);-range*sin(phi);0];
        else
            phi=pi*1.5-psi+bearing;
            Yd_follower(:,i)=Yd_leader(:,i)+[-range*cos(phi);range*sin(phi);0];
        end
    end
end


end