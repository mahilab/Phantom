% This code computes the Jacobian of the phantom

syms l1 l2 q1 q2 q3

DH_table = [0 0 0 q1;
            0 pi/2 0 q2;
            l1 0 0 q3-q2;
            0 pi/2 l2 0];
[phantom_FK,T_array1] = dh2tf(DH_table);

fk = phantom_FK(1:3,4);
                
J = jacobian(fk,[q1,q2,q3]);
generate_code(J,'J')
generate_code(J.','J_transpose')