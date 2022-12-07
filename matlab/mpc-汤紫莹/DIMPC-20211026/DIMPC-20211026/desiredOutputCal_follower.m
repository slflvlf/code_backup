function [Yd_follower]=desiredOutputCal_follower(Yd_leader,range,bearing,flag)

%Input
%Yd_leader    leader������λ�ú������ Ҳ���ǲο��켣
%range        leader��follower֮�����������
%bearing      leader��follower֮���γɱ�ӵĽǶ�
%flag         flag==1 leftfo;flag==2 rightfo

% Output
% Yd_follower  follower������λ�ú������

global Np;
Yd_follower=zeros(3,Np);
if flag==1 %�������follower
    for i=1:Np
    psi=Yd_leader(3,i);%leader����������� rad
        if psi>=0 && psi<=pi 
            phi=psi+bearing-pi*0.5;      
            Yd_follower(:,i)=Yd_leader(:,i)+[-range*cos(phi);-range*sin(phi);0];
        else
            phi=psi+bearing-pi*1.5;
            Yd_follower(:,i)=Yd_leader(:,i)+[range*cos(phi);range*sin(phi);0];
        end
    end
elseif flag==2 %�����ұ�follower
    for i=1:Np
    psi=Yd_leader(3,i);%leader����������� rad
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