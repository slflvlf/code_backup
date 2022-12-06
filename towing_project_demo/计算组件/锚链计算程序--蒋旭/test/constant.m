function [dt,k0,k1,k2,k3,k4,k5,CDn,CAn,pho_water,current,nt]=constant(Time, mooring_param)


CDn=mooring_param.coef_drag;          %拖曳系数
CAn=mooring_param.coef_addmass;          %附加质量系数
pho_water=1025;  %水密度kg/m3  
dt=mooring_param.time_step;  %秒
alfa=0.505;
beta=0.256;
k0=1/beta/dt/dt;
k1=1/beta/dt;
k2=1/(2*beta)-1;
k3=alfa/beta/dt;
k4=1-alfa/beta;
k5=dt*(alfa/beta-2)/2;
current=mooring_param.current;
% nt=Time/dt;

nt = 1;
end