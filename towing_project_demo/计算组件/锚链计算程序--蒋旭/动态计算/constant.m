function [dt,k0,k1,k2,k3,k4,k5,CDn,CAn,pho_water,current,nt]=constant(Time, mooring_param)
% CDn=1;          %拖曳系数
% CAn=1;          %附加质量系数
% dt=1;  %秒
% current=[0;0;0];% 流速
CDn = mooring_param.coef_addmass;  %拖曳系数
CAn=mooring_param.coef_addmass;          %附加质量系数
dt=mooring_param.time_step;  %秒
current=mooring_param.current;

pho_water=1025;  %水密度kg/m3  

alfa=0.505;
beta=0.256;
k0=1/beta/dt/dt;
k1=1/beta/dt;
k2=1/(2*beta)-1;
k3=alfa/beta/dt;
k4=1-alfa/beta;
k5=dt*(alfa/beta-2)/2;

nt=Time/dt;
nt = 1;
end