% This function takes as its input a vector joint_angles of the form  
% [theta1, theta2 , theta3 ] describing the position of each link relative 
% to the previous one, and outputs a 4x4 homogeneous transformation from 
% the base frame to the EE frame 

syms l1 l2 q1 q2 q3

DH_table = [0 0 0 q1;
            0 pi/2 0 q2;
            l1 0 0 q3-q2;
            0 pi/2 l2 0];
[phantom_FK,T_array1] = dh2tf(DH_table);

fk = phantom_FK(1:3,4)

generate_code(fk,'P')