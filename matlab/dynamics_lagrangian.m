% Evan Pezent | evanpezent.com | epezent@rice.edu
% 04/10/2018

function [Tau,L,K,U] = dynamics_lagrangian(k,u,Q,Qd,Qdd)
% =========================================================================
% Computes the dynamic equations of motion for a robotic manipulator using 
% the Lagrangian formulation.
%
% k   = array of kinetic energy terms
% u   = array of potential energy terms
% Q   = [n x 1] vector of joint angles
% Qd  = [n x 1] vector of joint angular velocities
% Qdd = [n x 1] vector of joint angular accelerations
% 
% =========================================================================
% Source: Introduction to Robotics: Mechanics and Control (3e) - Craig, J.
% Eqns: 5.64 (pg. 150), 6.69 - 6.76 (pg. 182 - 182)
% =========================================================================

num = length(Q);

% Total Kinetic Engery
K = simplify(sum(k)); % 6.70
% Total Potential Energy
U = simplify(sum(u)); % 6.74
% Lagrangian
L = simplify(K-U);

% Compute Derivatives
for i = 1:num
    dLdQ(i,1)  = simplify( diff(L, Q(i)) );  % dL/dQ
    dLdQd(i,1) = simplify( diff(L, Qd(i)) ); % dL/dQd
    ddtdLdQd(i,1) = sym(0);                  
    for j = 1:num
        ddtdLdQd(i,1) = ddtdLdQd(i) + diff(dLdQd(i),Qd(j))*Qdd(j) + diff(dLdQd(i),Q(j))*Qd(j);
    end
    ddtdLdQd(i,1) = simplify(ddtdLdQd(i));   % d/dt(dL/dQd)
end

% compute torques / equations of motion
Tau = simplify(ddtdLdQd - dLdQ); % 6.76

end
