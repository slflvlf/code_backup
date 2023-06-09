function out1 = OneShipStateFcn(in1,in2)
%OneShipStateFcn
%    OUT1 = OneShipStateFcn(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    22-Nov-2022 20:29:28

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
et1 = u.*(-4.0e-2)+f1.*t2.*1.639344262295082e-5+f2.*t3.*1.639344262295082e-5+f3.*t4.*1.639344262295082e-5;
et2 = f4.*t5.*1.639344262295082e-5+r.*v.*1.40983606557377;
et3 = v.*(-5.674418604651163e-2)+f1.*t7.*1.162790697674419e-5+f2.*t8.*1.162790697674419e-5+f3.*t9.*1.162790697674419e-5;
et4 = f4.*t10.*1.162790697674419e-5-r.*u.*7.093023255813954e-1;
et5 = r.*(-7.030416945292873e-2)-u.*v.*2.259131409155808e-4-f1.*(t2.*5.0e+1-t7.*7.5e+1).*9.036525636623231e-9;
et6 = f2.*(t3.*5.0e+1+t8.*7.5e+1).*(-9.036525636623231e-9)+f3.*(t4.*5.0e+1-t9.*7.5e+1).*9.036525636623231e-9+f4.*(t5.*5.0e+1+t10.*7.5e+1).*9.036525636623231e-9;
out1 = [t6.*u-t11.*v;t11.*u+t6.*v;r;et1+et2;et3+et4;et5+et6];
