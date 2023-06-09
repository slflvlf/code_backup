function out1 = OneShipStateFcn(in1,in2)
%OneShipStateFcn
%    OUT1 = OneShipStateFcn(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    28-May-2022 15:38:52

alpha1 = in2(4,:);
alpha2 = in2(5,:);
alpha3 = in2(6,:);
f1 = in2(1,:);
f2 = in2(2,:);
f3 = in2(3,:);
fai = in1(3,:);
r = in1(6,:);
u = in1(4,:);
v = in1(5,:);
t2 = cos(alpha2);
t3 = cos(alpha3);
t4 = cos(fai);
t5 = sin(alpha1);
t6 = sin(alpha2);
t7 = sin(alpha3);
t8 = sin(fai);
et1 = u.*(-1.0e-2)+f2.*t2.*1.426940639269406e-3+f3.*t3.*1.426940639269406e-3+f1.*cos(alpha1).*1.426940639269406e-3;
et2 = r.*(r.*1.4e+5-v.*9.94e+5).*(-1.426940639269406e-6);
et3 = r.*(-7.045416178974943e-3)-v.*2.501122743536104e-2-f2.*t2.*8.661164421380084e-6+f1.*t5.*9.903846754214311e-4;
et4 = f3.*t3.*8.661164421380084e-6+f2.*t6.*1.04484094476119e-3+f3.*t7.*1.04484094476119e-3;
et5 = r.*u.*(-7.048977109035066e-1)-u.*v.*9.405382993883855e-4;
et6 = r.*(-5.00224548707221e-2)-v.*7.971479106344263e-5-f2.*t2.*6.14942673917986e-5-f1.*t5.*1.111259473649811e-4;
et7 = f3.*t3.*6.14942673917986e-5+f2.*t6.*2.755135649473053e-4+f3.*t7.*2.755135649473053e-4+r.*u.*9.405382993883858e-4;
et8 = u.*v.*(-6.677821925657537e-3);
out1 = [t4.*u-t8.*v;t8.*u+t4.*v;r;et1+et2;et3+et4+et5;et6+et7+et8];
