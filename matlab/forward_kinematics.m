% This function takes as its input a vector joint_angles of the form  
% [theta1, theta2 , theta3 ] describing the position of each link relative 
% to the previous one, and a vector gimbal_angles of roll-pitch-yaw angles, and outputs a
% 4x4 homogeneous transformation phantom_T_0_g from the base frame to the
% gimbal frame and a cell array phantom_T of the 4x4 homogeneous
% transformations for this robot

syms l1 l2 x_ee y_ee z_ee

theta1 = atan2(y_ee,x_ee);
l_star = sqrt(x_ee^2 + y_ee^2);
ee_theta = atan2(z_ee,l_star);
lh = sqrt(z_ee^2 + l_star^2);
theta2 = acos((l2^2-l1^2-lh^2)/(-2*l1*lh))+ee_theta;
theta3 = acos((lh^2-l1^2-l2^2)/(-2*l1*l2))+theta2-pi/2;
