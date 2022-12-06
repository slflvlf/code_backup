function [P_initial]=motion(nt,N,Y_N,dt)
Amp=10;             %%横荡纵荡正弦运动幅值
Amp_V=0;            %%垂荡运动幅值
Amp_RPY=0;
vesseltranslation=zeros(3,nt+1);
vesselrotation=zeros(3,nt+1);
for i=1:nt+1
    vesseltranslation(1,i)=Amp*(-cos(0.02*pi*(i-1)*dt)+1);    %%%%%  模拟低频运动，周期100s，幅值10m
    vesseltranslation(2,i)=0*Amp*(-cos(0.02*pi*(i-1)*dt)+1);
    vesseltranslation(3,i)=Amp_V*(-cos(0.1*pi*(i-1)*dt)+1);   %%%%%  垂荡运动，周期20s，幅值2m
    vesselrotation(1,i)=Amp_RPY*(-cos(0.06*pi*(i-1)*dt)+1);   %%%%%  横摇运动，周期33.3s，幅值5°
    vesselrotation(2,i)=Amp_RPY*(-cos(0.1*pi*(i-1)*dt)+1);    %%%%%  纵摇运动，周期20s，幅值5°
    vesselrotation(3,i)=Amp_RPY*(-cos(0.02*pi*(i-1)*dt)+1);   %%%%%  艏摇运动，周期100s，幅值5°
end
P_initial=zeros(3*N+3,nt+1);
for i=2:nt+1
    alfa1=vesselrotation(1,i);
    alfa2=vesselrotation(2,i);
    alfa3=vesselrotation(3,i);
    T_Matrix=[cosd(alfa2)*cosd(alfa3),-cosd(alfa2)*sind(alfa3),sind(alfa2);
    sind(alfa1)*sind(alfa2)*cosd(alfa3)+cosd(alfa1)*sind(alfa3),cosd(alfa1)*cosd(alfa3)-sind(alfa1)*sind(alfa2)*sind(alfa3),-sind(alfa1)*cosd(alfa2);
    sind(alfa1)*sind(alfa3)-cosd(alfa1)*sind(alfa2)*cosd(alfa3),sind(alfa1)*cosd(alfa3)+cosd(alfa1)*sind(alfa2)*sind(alfa3),cosd(alfa1)*cosd(alfa2)];
    P_initial(1:3,i)=vesseltranslation(:,i)+T_Matrix*Y_N(1:3,1);
    P_initial(3*N+1,i)=Y_N(3*N+1,1);
    P_initial(3*N+2,i)=Y_N(3*N+2,1);
    P_initial(3*N+3,i)=Y_N(3*N+3,1);
end
P_initial(:,1)=Y_N;
end