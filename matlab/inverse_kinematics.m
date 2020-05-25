% This code generates the inverse kinematics of the phantom given an input
% of x_ee, y_ee, z_ee

syms l1 l2 x_ee y_ee z_ee

l_star = sqrt(x_ee^2 + y_ee^2);
ee_theta = atan2(z_ee,l_star);
lh = sqrt(z_ee^2 + l_star^2);

%Sol 1
theta1(1) = atan2(y_ee,x_ee);
theta2(1) = acos((l2^2-l1^2-lh^2)/(-2*l1*lh))+ee_theta;
theta3(1) = acos((lh^2-l1^2-l2^2)/(-2*l1*l2))+theta2(1)-pi/2;

%Sol 2
theta1(2) = wrapToPi(atan2(y_ee,x_ee)-pi);
theta2(2) = pi-ee_theta+acos((l2^2-l1^2-lh^2)/(-2*l1*lh));
theta3(2) = acos((lh^2-l1^2-l2^2)/(-2*l1*l2))+theta2(2)-pi/2;
