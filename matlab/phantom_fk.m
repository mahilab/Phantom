function [ee] = phantom_fk(q1,q2,q3)
l1 = 0.209550;
l2 = 0.169545;
l3 = 0.031750;
ee = [cos(q1)*(l1*cos(q2) + l2*sin(q3));
     sin(q1)*(l1*cos(q2) + l2*sin(q3));
     l1*sin(q2) - l2*cos(q3)];
end