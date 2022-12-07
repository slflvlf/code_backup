function out1 = ShipStateFcn(in1,in2)
%SHIPSTATEFCN
%    OUT1 = SHIPSTATEFCN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    12-Nov-2021 18:21:06

alpha1 = in2(5,:);
alpha2 = in2(6,:);
alpha3 = in2(7,:);
alpha4 = in2(8,:);
f1 = in2(1,:);
f2 = in2(2,:);
f3 = in2(3,:);
f4 = in2(4,:);
fai = in1(3,:);
r = in1(6,:);
tau_env1 = in2(9,:);
tau_env2 = in2(10,:);
tau_env3 = in2(11,:);
u = in1(4,:);
v = in1(5,:);
t2 = cos(alpha1);
t3 = cos(alpha2);
t4 = cos(alpha3);
t5 = cos(alpha4);
t6 = cos(fai);
t7 = sin(alpha1);
t8 = sin(alpha2);
t9 = sin(alpha3);
t10 = sin(alpha4);
t11 = sin(fai);
out1 = [t6.*u-t11.*v;t11.*u+t6.*v;r;tau_env1.*4.614674665436087e-2-u.*1.085371481310568+f1.*t2.*4.614674665436087e-2+f2.*t3.*4.614674665436087e-2+f3.*t4.*4.614674665436087e-2+f4.*t5.*4.614674665436087e-2+r.*v.*1.803414859252423;tau_env2.*(2.5e+1./9.77e+2)-v.*(5.58e+2./9.77e+2)+f1.*t7.*(2.5e+1./9.77e+2)+f2.*t8.*(2.5e+1./9.77e+2)+f3.*t9.*(2.5e+1./9.77e+2)+f4.*t10.*(2.5e+1./9.77e+2)-r.*u.*5.54503582395087e-1;r.*(-2.583791208791209e-1)+tau_env3.*(2.5e+1./3.64e+2)+u.*v.*1.195741758241758-f1.*(t2./5.0-t7.*(2.0./5.0)).*(2.5e+1./3.64e+2)-f2.*(t3./5.0+t8.*(2.0./5.0)).*(2.5e+1./3.64e+2)+f3.*(t4./5.0-t9.*(2.0./5.0)).*(2.5e+1./3.64e+2)+f4.*(t5./5.0+t10.*(2.0./5.0)).*(2.5e+1./3.64e+2)];
