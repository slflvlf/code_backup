function [A,B] = OneShipStateJacobianFcn(in1,in2)
%OneShipStateJacobianFcn
%    [A,B] = OneShipStateJacobianFcn(IN1,IN2)

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
mt1 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-t11.*u-t6.*v,t6.*u-t11.*v,0.0,0.0,0.0,0.0,t6,t11,0.0,-4.0e-2,r.*(-7.093023255813954e-1),v.*(-2.259131409155808e-4),-t11,t6,0.0];
mt2 = [r.*1.40983606557377,-5.674418604651163e-2,u.*(-2.259131409155808e-4),0.0,0.0,1.0,v.*1.40983606557377];
mt3 = [u.*(-7.093023255813954e-1),-7.030416945292873e-2];
A = reshape([mt1,mt2,mt3],6,6);
if nargout > 1
    mt4 = [0.0,0.0,0.0,t2.*1.639344262295082e-5,t7.*1.162790697674419e-5,t2.*(-4.518262818311615e-7)+t7.*6.777394227467423e-7,0.0,0.0,0.0];
    mt5 = [t3.*1.639344262295082e-5,t8.*1.162790697674419e-5,t3.*(-4.518262818311615e-7)-t8.*6.777394227467423e-7,0.0,0.0,0.0];
    mt6 = [t4.*1.639344262295082e-5,t9.*1.162790697674419e-5,t4.*4.518262818311615e-7-t9.*6.777394227467423e-7,0.0,0.0,0.0];
    mt7 = [t5.*1.639344262295082e-5,t10.*1.162790697674419e-5,t5.*4.518262818311615e-7+t10.*6.777394227467423e-7,0.0,0.0,0.0];
    mt8 = [f1.*t7.*(-1.639344262295082e-5),f1.*t2.*1.162790697674419e-5,f1.*(t2.*7.5e+1+t7.*5.0e+1).*9.036525636623231e-9,0.0,0.0,0.0,f2.*t8.*(-1.639344262295082e-5)];
    mt9 = [f2.*t3.*1.162790697674419e-5,f2.*(t3.*7.5e+1-t8.*5.0e+1).*(-9.036525636623231e-9),0.0,0.0,0.0,f3.*t9.*(-1.639344262295082e-5)];
    mt10 = [f3.*t4.*1.162790697674419e-5,f3.*(t4.*7.5e+1+t9.*5.0e+1).*(-9.036525636623231e-9),0.0,0.0,0.0,f4.*t10.*(-1.639344262295082e-5)];
    mt11 = [f4.*t5.*1.162790697674419e-5,f4.*(t5.*7.5e+1-t10.*5.0e+1).*9.036525636623231e-9];
    B = reshape([mt4,mt5,mt6,mt7,mt8,mt9,mt10,mt11],6,8);
end
