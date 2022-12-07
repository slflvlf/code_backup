function [A,B] = QuadrotorStateJacobianFcn(in1,in2)
%QUADROTORSTATEJACOBIANFCN
%    [A,B] = QUADROTORSTATEJACOBIANFCN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    11-Nov-2021 20:28:27

phi = in1(4,:);
phidot = in1(10,:);
psidot = in1(12,:);
psi = in1(6,:);
thetadot = in1(11,:);
theta = in1(5,:);
u1 = in2(1,:);
u2 = in2(2,:);
u3 = in2(3,:);
u4 = in2(4,:);
t2 = cos(phi);
t3 = cos(psi);
t4 = cos(theta);
t5 = sin(phi);
t6 = sin(psi);
t7 = sin(theta);
t8 = phi.*2.0;
t9 = theta.*2.0;
t10 = thetadot.^2;
t22 = u1+u2+u3+u4;
t11 = cos(t8);
t12 = t2.^2;
t13 = t2.^3;
t15 = cos(t9);
t16 = t4.^2;
t17 = t4.^3;
t18 = sin(t8);
t19 = t5.^2;
t20 = sin(t9);
t21 = t7.^2;
t23 = t4.*6.0e+1;
t24 = 1.0./t4;
t27 = t7.*9.2e+1;
t28 = t7.*1.15e+2;
t33 = (t2.*t4)./2.0;
t34 = (t3.*t5)./2.0;
t35 = (t5.*t6)./2.0;
t42 = t2.*t4.*t5.*5.5e+1;
t43 = t2.*t5.*t7.*5.5e+1;
t48 = (t2.*t3.*t7)./2.0;
t51 = (t2.*t6.*t7)./2.0;
t14 = t12.^2;
t25 = 1.0./t16;
t26 = 1.0./t17;
t29 = -t27;
t30 = t12.*4.4e+1;
t31 = t12.*5.5e+1;
t32 = t18.*2.2e+1;
t38 = -t34;
t44 = t7.*t12.*-4.4e+1;
t45 = t7.*t12.*-5.5e+1;
t53 = t7.*t42;
t54 = t4.*t12.*thetadot.*2.53e+2;
t55 = t4.*t12.*u3.*-5.5e+1;
t62 = phidot.*psidot.*t12.*t16.*3.85e+2;
t65 = t35+t48;
t36 = -t30;
t37 = -t31;
t39 = t4.*t31;
t40 = t7.*t30;
t41 = t7.*t31;
t52 = t16.*t31;
t56 = t45.*u4;
t57 = phidot.*t54;
t58 = t7.*t10.*t14.*1.21e+2;
t59 = t7.*t54;
t60 = psidot.*t4.*t7.*t14.*thetadot.*1.21e+2;
t64 = -t62;
t66 = t38+t51;
t46 = t39.*u1;
t47 = t39.*u3;
t49 = t41.*u2;
t50 = t41.*u4;
t61 = -t58;
t63 = -t60;
A = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,(t22.*(t2.*t6-t3.*t5.*t7))./2.0,t22.*(t2.*t3+t5.*t6.*t7).*(-1.0./2.0),t4.*t5.*t22.*(-1.0./2.0),t25.*(t10.*t11.*2.53e+2-t10.*t14.*1.21e+2+t7.*t46-t10.*t12.*t16.*2.53e+2+t10.*t14.*t16.*1.21e+2+t10.*t12.*t19.*3.63e+2+t10.*t16.*t19.*2.53e+2+t2.*t5.*u2.*1.1e+2-t2.*t5.*u4.*1.1e+2+t4.*t45.*u3-t10.*t12.*t16.*t19.*3.63e+2+t2.*t5.*t7.*u1.*8.8e+1-t2.*t5.*t7.*u2.*8.8e+1+t2.*t5.*t7.*u3.*8.8e+1-t2.*t5.*t7.*u4.*8.8e+1-t2.*t5.*t16.*u2.*1.1e+2+t2.*t5.*t16.*u4.*1.1e+2-t4.*t7.*t19.*u1.*5.5e+1+t4.*t7.*t19.*u3.*5.5e+1-phidot.*psidot.*t7.*t12.*t16.*3.85e+2+phidot.*psidot.*t7.*t16.*t19.*3.85e+2+psidot.*t2.*t4.*t5.*thetadot.*5.06e+2-psidot.*t4.*t5.*t13.*thetadot.*4.84e+2-psidot.*t2.*t5.*t17.*thetadot.*5.06e+2+psidot.*t5.*t13.*t17.*thetadot.*4.84e+2+phidot.*t2.*t4.*t5.*t7.*thetadot.*5.06e+2).*(-1.0./5.52e+2),t24.*(t49+t56+t57+t63+t11.*u1.*4.4e+1-t11.*u2.*4.4e+1+t11.*u3.*4.4e+1-t11.*u4.*4.4e+1-t7.*t19.*u2.*5.5e+1+t7.*t19.*u4.*5.5e+1-phidot.*t4.*t19.*thetadot.*2.53e+2-t2.*t5.*t7.*t10.*2.42e+2+t5.*t7.*t10.*t13.*4.84e+2-t2.*t4.*t5.*u1.*1.1e+2+t2.*t4.*t5.*u3.*1.1e+2+phidot.*psidot.*t2.*t5.*t16.*7.7e+2+psidot.*t4.*t7.*t12.*t19.*thetadot.*3.63e+2).*(-1.0./5.52e+2),t25.*(t46+t55+t61+t64+t7.*t10.*t12.*2.53e+2-t7.*t10.*t19.*2.53e+2+t2.*t5.*u1.*8.8e+1-t2.*t5.*u2.*8.8e+1+t2.*t5.*u3.*8.8e+1-t2.*t5.*u4.*8.8e+1-t4.*t19.*u1.*5.5e+1+t4.*t19.*u3.*5.5e+1+phidot.*psidot.*t16.*t19.*3.85e+2+t7.*t10.*t12.*t19.*3.63e+2+t2.*t5.*t7.*u2.*1.1e+2-t2.*t5.*t7.*u4.*1.1e+2+phidot.*t2.*t4.*t5.*thetadot.*5.06e+2+psidot.*t2.*t4.*t5.*t7.*thetadot.*5.06e+2-psidot.*t4.*t5.*t7.*t13.*thetadot.*4.84e+2).*(-1.0./5.52e+2),0.0,0.0,0.0,0.0,0.0,0.0,t3.*t22.*t33,t6.*t22.*t33,t2.*t7.*t22.*(-1.0./2.0),t25.*(t4.*u1.*9.2e+1-t4.*u2.*9.2e+1+t4.*u3.*9.2e+1-t4.*u4.*9.2e+1-phidot.*t15.*thetadot.*2.3e+1+psidot.*t7.*thetadot.*2.76e+2-t4.*t12.*u1.*4.4e+1-t4.*t12.*u3.*4.4e+1+t4.*t30.*u2+t4.*t30.*u4-phidot.*t12.*t16.*thetadot.*2.53e+2+phidot.*t12.*t21.*thetadot.*2.53e+2+psidot.*t7.*t12.*thetadot.*2.53e+2-psidot.*t7.*t14.*thetadot.*1.21e+2+t2.*t5.*t16.*u1.*5.5e+1-t4.*t7.*t12.*u2.*1.1e+2-t2.*t5.*t16.*u3.*5.5e+1+t4.*t7.*t12.*u4.*1.1e+2-t2.*t5.*t21.*u1.*5.5e+1+t2.*t5.*t21.*u3.*5.5e+1-phidot.*psidot.*t2.*t5.*t17.*3.85e+2-psidot.*t7.*t12.*t16.*thetadot.*7.59e+2+psidot.*t7.*t14.*t16.*thetadot.*3.63e+2+t2.*t4.*t5.*t7.*t10.*5.06e+2-t4.*t5.*t7.*t10.*t13.*2.42e+2+phidot.*psidot.*t2.*t4.*t5.*t21.*7.7e+2).*(-1.0./5.52e+2)+(t7.*t26.*(u2.*-1.15e+2+u4.*1.15e+2+psidot.*t54-t10.*t18.*(2.53e+2./2.0)+t7.*t57-t7.*u1.*9.2e+1-t7.*u3.*9.2e+1-t12.*u4.*5.5e+1+t27.*u2+t27.*u4+t31.*u2+t40.*u1+t40.*u3+t44.*u2+t44.*u4+t52.*u4+t53.*u3+phidot.*t20.*thetadot.*(2.3e+1./2.0)+psidot.*t4.*thetadot.*2.76e+2+t5.*t10.*t13.*1.21e+2-t12.*t16.*u2.*5.5e+1-psidot.*t4.*t14.*thetadot.*1.21e+2-psidot.*t12.*t17.*thetadot.*2.53e+2+psidot.*t14.*t17.*thetadot.*1.21e+2+t2.*t5.*t10.*t16.*2.53e+2-t5.*t10.*t13.*t16.*1.21e+2-t2.*t4.*t5.*t7.*u1.*5.5e+1+phidot.*psidot.*t2.*t5.*t7.*t16.*3.85e+2))./2.76e+2,(t24.*(t7.*u1.*6.0e+1-t7.*u3.*6.0e+1+t41.*u1+t42.*u4+t45.*u3-t4.*t10.*t12.*1.21e+2+t4.*t10.*t14.*1.21e+2+phidot.*psidot.*t4.*t7.*2.64e+2-t2.*t4.*t5.*u2.*5.5e+1-phidot.*psidot.*t4.*t7.*t12.*7.7e+2+phidot.*t2.*t5.*t7.*thetadot.*2.53e+2+psidot.*t5.*t13.*t16.*thetadot.*1.21e+2-psidot.*t5.*t13.*t21.*thetadot.*1.21e+2))./5.52e+2+(t7.*t25.*(t47+t58+t62+t4.*u3.*6.0e+1-t18.*u1.*2.2e+1-t18.*u3.*2.2e+1-t23.*u1+t32.*u2+t32.*u4+t43.*u4-phidot.*psidot.*t16.*1.32e+2-t7.*t10.*t12.*1.21e+2-t4.*t12.*u1.*5.5e+1-t2.*t5.*t7.*u2.*5.5e+1-phidot.*t2.*t4.*t5.*thetadot.*2.53e+2+psidot.*t4.*t5.*t7.*t13.*thetadot.*1.21e+2))./5.52e+2,t25.*(t4.*u2.*1.15e+2-t4.*u4.*1.15e+2+t39.*u4+t43.*u3+phidot.*t7.*thetadot.*2.3e+1-psidot.*t15.*thetadot.*2.76e+2-t4.*t12.*u2.*5.5e+1+phidot.*t7.*t12.*thetadot.*2.53e+2-psidot.*t12.*t16.*thetadot.*2.53e+2+psidot.*t14.*t16.*thetadot.*1.21e+2+psidot.*t12.*t21.*thetadot.*2.53e+2-psidot.*t14.*t21.*thetadot.*1.21e+2+t2.*t4.*t5.*t10.*2.53e+2-t4.*t5.*t10.*t13.*1.21e+2-t2.*t5.*t7.*u1.*5.5e+1+phidot.*psidot.*t2.*t4.*t5.*t7.*7.7e+2).*(-1.0./5.52e+2)+(t7.*t26.*(t49+t56+t57+t63-u1.*9.2e+1+u2.*9.2e+1-u3.*9.2e+1+u4.*9.2e+1+psidot.*t59-t7.*u2.*1.15e+2-t12.*u2.*4.4e+1-t12.*u4.*4.4e+1+t30.*u1+t28.*u4+t30.*u3+t42.*u3+phidot.*t4.*thetadot.*2.3e+1+psidot.*t20.*thetadot.*1.38e+2-t2.*t5.*t7.*t10.*2.53e+2+t5.*t7.*t10.*t13.*1.21e+2-t2.*t4.*t5.*u1.*5.5e+1+phidot.*psidot.*t2.*t5.*t16.*3.85e+2))./2.76e+2,0.0,0.0,0.0,0.0,0.0,0.0,(t22.*(t3.*t5-t2.*t6.*t7))./2.0,(t22.*(t5.*t6+t2.*t3.*t7))./2.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,(t25.*(t59+t20.*thetadot.*(2.3e+1./2.0)+psidot.*t2.*t5.*t7.*t16.*3.85e+2))./5.52e+2,t24.*(psidot.*t16.*1.32e+2-psidot.*t12.*t16.*3.85e+2+t2.*t4.*t5.*thetadot.*2.53e+2).*(-1.0./5.52e+2),(t25.*(t54+t4.*thetadot.*2.3e+1+psidot.*t2.*t5.*t16.*3.85e+2))./5.52e+2,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,(t25.*(phidot.*t20.*(2.3e+1./2.0)+psidot.*t4.*2.76e+2-t18.*thetadot.*2.53e+2+psidot.*t4.*t12.*2.53e+2-psidot.*t4.*t14.*1.21e+2-psidot.*t12.*t17.*2.53e+2+psidot.*t14.*t17.*1.21e+2+t5.*t13.*thetadot.*2.42e+2+phidot.*t4.*t7.*t12.*2.53e+2+t2.*t5.*t16.*thetadot.*5.06e+2-t5.*t13.*t16.*thetadot.*2.42e+2))./5.52e+2,t24.*(t7.*t12.*thetadot.*2.42e+2-t7.*t14.*thetadot.*2.42e+2+phidot.*t2.*t4.*t5.*2.53e+2-psidot.*t4.*t5.*t7.*t13.*1.21e+2).*(-1.0./5.52e+2),(t25.*(phidot.*t4.*2.3e+1+psidot.*t20.*1.38e+2+phidot.*t4.*t12.*2.53e+2+psidot.*t4.*t7.*t12.*2.53e+2-psidot.*t4.*t7.*t14.*1.21e+2-t2.*t5.*t7.*thetadot.*5.06e+2+t5.*t7.*t13.*thetadot.*2.42e+2))./5.52e+2,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,(t25.*(t54+t4.*thetadot.*2.76e+2-t4.*t14.*thetadot.*1.21e+2-t12.*t17.*thetadot.*2.53e+2+t14.*t17.*thetadot.*1.21e+2+phidot.*t2.*t5.*t7.*t16.*3.85e+2))./5.52e+2,(t24.*(phidot.*t16.*-1.32e+2+phidot.*t12.*t16.*3.85e+2+t4.*t5.*t7.*t13.*thetadot.*1.21e+2))./5.52e+2,(t25.*(t59+t20.*thetadot.*1.38e+2+phidot.*t2.*t5.*t16.*3.85e+2-t4.*t7.*t14.*thetadot.*1.21e+2))./5.52e+2],[12,12]);
if nargout > 1
    B = reshape([0.0,0.0,0.0,0.0,0.0,0.0,t65,t66,t33,t25.*(t27+t44+t53).*(-1.0./5.52e+2),t24.*(t23+t32+t39).*(-1.0./5.52e+2),t25.*(t36+t42+9.2e+1).*(-1.0./5.52e+2),0.0,0.0,0.0,0.0,0.0,0.0,t65,t66,t33,t25.*(t29+t37+t40+t52+1.15e+2).*(-1.0./5.52e+2),(t24.*(t32-t43))./5.52e+2,t25.*(t28+t30+t45-9.2e+1).*(-1.0./5.52e+2),0.0,0.0,0.0,0.0,0.0,0.0,t65,t66,t33,(t25.*(t29+t40+t53))./5.52e+2,(t24.*(t23-t32+t39))./5.52e+2,(t25.*(t30+t42-9.2e+1))./5.52e+2,0.0,0.0,0.0,0.0,0.0,0.0,t65,t66,t33,(t25.*(t27+t37+t44+t52+1.15e+2))./5.52e+2,(t24.*(t32+t43))./5.52e+2,(t25.*(t28+t36+t45+9.2e+1))./5.52e+2],[12,4]);
end
