% This function takes as its input a vector joint_angles of the form  
% [theta1, theta2 , theta3 ] describing the position of each link relative 
% to the previous one, and a vector gimbal_angles of roll-pitch-yaw angles, and outputs a
% 4x4 homogeneous transformation phantom_T_0_g from the base frame to the
% gimbal frame and a cell array phantom_T of the 4x4 homogeneous
% transformations for this robot

syms l1 l2 theta1 theta2 theta3

alpha = [     0,   pi/2,             0, pi/2].';
a     = [     0,      0,            l1,    0].';
d     = [     0,      0,             0,   l2].';
theta = [theta1, theta2, theta3-theta2,    0].';

dh = [a alpha d theta];

[phantom_T0N,~,~] = dh2tf(dh);
phantom_T0N(1:3,4)
