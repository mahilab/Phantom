% Evan Pezent | evanpezent.com | epezent@rIc_e.edu
% 05/22/2020


%% Define Symbolic Symbols
syms tau1 eta1 Jm1 q1 q1d q1dd b1 fk1 ...
     tau2 eta2 Jm2 q2 q2d q2dd b2 fk2 ...
     tau3 eta3 Jm3 q3 q3d q3dd b3 fk3 ...
     l2 l3 ...
     m_a  Pc_ax  Pc_ay  Pc_az    Ic_axx Ic_axy   Ic_axz  Ic_ayy  Ic_ayz  Ic_azz ...
     m_c  Pc_cx  Pc_cy  Pc_cz    Ic_cxx Ic_cxy   Ic_cxz  Ic_cyy  Ic_cyz  Ic_czz ...
     m_be Pc_bex Pc_bey Pc_bez   Ic_bexx Ic_bexy Ic_bexz Ic_beyy Ic_beyz Ic_bezz ...
     m_df Pc_dfx Pc_dfy Pc_dfz   Ic_dfxx Ic_dfxy Ic_dfxz Ic_dfyy Ic_dfyz Ic_dfzz ...
     m_g  Pc_gx  Pc_gy  Pc_gz    Ic_gxx Ic_gxy   Ic_gxz  Ic_gyy  Ic_gyz  Ic_gzz ...
     g
 
Tau = [tau1; tau2; tau3];
Eta = [eta1;eta2;eta3];
Jm = [Jm1;Jm2;Jm3];

Q = [q1;q2;q3];
Qd = [q1d;q2d;q3d];
Qdd = [q1dd;q2dd;q3dd];

B = [b1;b2;b3];
Fk = [fk1;fk2;fk3];

Pc_a  =  [Pc_ax Pc_ay Pc_az].';
Pc_c  =  [Pc_cx Pc_cy Pc_cz].';
Pc_be = [Pc_bex Pc_bey Pc_bez].';
Pc_df = [Pc_dfx Pc_dfy Pc_dfz].';
Pc_g = [Pc_gx Pc_gy Pc_gz].';

% TODO: zero non-diagonal
Ic_a = [Ic_axx -Ic_axy -Ic_axz;
       -Ic_axy  Ic_ayy -Ic_ayz;
       -Ic_axz -Ic_ayz  Ic_axx];
   
% TODO: zero non-diagonal   
Ic_c = [Ic_cxx -Ic_cxy -Ic_cxz;
       -Ic_cxy  Ic_cyy -Ic_cyz;
       -Ic_cxz -Ic_cyz  Ic_cxx];
   
Ic_be = [Ic_bexx -Ic_bexy -Ic_bexz;
        -Ic_bexy  Ic_beyy -Ic_beyz;
        -Ic_bexz -Ic_beyz  Ic_bexx];
    
Ic_df = [Ic_dfxx -Ic_dfxy -Ic_dfxz;
        -Ic_dfxy  Ic_dfyy -Ic_dfyz;
        -Ic_dfxz -Ic_dfyz  Ic_dfxx];   

Ic_g = [Ic_gxx -Ic_gxy -Ic_gxz;
       -Ic_gxy  Ic_gyy -Ic_gyz;
       -Ic_gxz -Ic_gyz  Ic_gxx];
   
DH_table1 = [0 0 0 q1;
             0 pi/2 0 q2;
             l2 0 0 q3-q2];
         
DH_table2 = [0 0 0 q1;
             0 pi/2 0 q3-pi/2
             l3 0 0 q2-q3];
             
[~,T_array2] = dh2tf(DH_table2);

g0 = [0; 0; g];

%% Lagrangian Dynamics

% setup Lagrangian first chain
[~,T_array1] = dh2tf(DH_table1);
m = [m_g,m_be,m_a];
Pc = {Pc_g,Pc_be,Pc_a};
Ic = {Ic_g,Ic_be,Ic_a};
[~, ~, ~, ~, k1, u1] = dynamics_lagrangian(m,Pc,Ic,T_array1,Q,Qd,Qdd,g0,0);

% setup Lagrangian second chain
[~,T_array1] = dh2tf(DH_table2);
m = [m_g,m_df,m_c];
Pc = {Pc_g,Pc_df,Pc_c};
Ic = {Ic_g,Ic_df,Ic_c};
[~, ~, ~, ~, k2, u2] = dynamics_lagrangian(m,Pc,Ic,T_array1,Q,Qd,Qdd,g0,0);

% sum kinetic/potential energies (k2(1) = k1(1) & u2(1) = u1(1), so skip)
K = simplify( k1(1) + k1(2) + k1(3) + k2(2) + k2(3) ); 
U = simplify( u1(1) + u1(2) + u1(3) + u2(2) + u2(3) );

% complete Lagrangian
L = simplify(K-U);

% compute derivatives
for i = 1:3
    dLdQ(i,1)  = simplify( diff(L, Q(i)) );  % dL/dQ
    dLdQd(i,1) = simplify( diff(L, Qd(i)) ); % dL/dQd
    ddtdLdQd(i,1) = sym(0);                  
    for j = 1:3
        ddtdLdQd(i,1) = ddtdLdQd(i) + diff(dLdQd(i),Qd(j))*Qdd(j) + diff(dLdQd(i),Q(j))*Qd(j);
    end
    ddtdLdQd(i,1) = simplify(ddtdLdQd(i));   % d/dt(dL/dQd)
end

% compute torques / equations of motion
MVG = simplify(ddtdLdQd - dLdQ); % 6.76

[M,V,G] = separate_mvg(MVG,Qdd,g);

%% Get Equation of Motion
EOM = Tau == M*Qdd + Jm.*Eta.^2.*Qdd + V + G + B.*Qd + Fk.*tanh(10 * Qd);
