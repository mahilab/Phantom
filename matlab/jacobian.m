% This code computes the Jacobian of the phantom

syms l1 l2 theta1 theta2 theta3

fk = [cos(theta1)*(l1*cos(theta2) + l2*sin(theta3));
      sin(theta1)*(l1*cos(theta2) + l2*sin(theta3));
                    l1*sin(theta2) - l2*cos(theta3)];
                
phantom_jac = jacobian(fk,[theta1,theta2,theta3])