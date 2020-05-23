% Evan Pezent | evanpezent.com | epezent@rIc_e.edu
% 05/22/2020

%% Define Symbolic Symbols
syms tau1 eta1 Jm1 q1 qd1 qdd1 b1 fk1 ...
     tau2 eta2 Jm2 q2 qd2 qdd2 b2 fk2 ...
     tau3 eta3 Jm3 q3 qd3 qdd3 b3 fk3 ...
     l2 l3 ...
     m_a  Pc_ax  Pc_ay  Pc_az    Ic_axx Ic_axy   Ic_axz  Ic_ayy  Ic_ayz  Ic_azz ...
     m_c  Pc_cx  Pc_cy  Pc_cz    Ic_cxx Ic_cxy   Ic_cxz  Ic_cyy  Ic_cyz  Ic_czz ...
     m_be Pc_bex Pc_bey Pc_bez   Ic_bexx Ic_bexy Ic_bexz Ic_beyy Ic_beyz Ic_bezz ...
     m_df Pc_dfx Pc_dfy Pc_dfz   Ic_dfxx Ic_dfxy Ic_dfxz Ic_dfyy Ic_dfyz Ic_dfzz ...
     m_g  Pc_gx  Pc_gy  Pc_gz    Ic_gxx Ic_gxy   Ic_gxz  Ic_gyy  Ic_gyz  Ic_gzz ...
     g

% joint torques 
Tau = [tau1; tau2; tau3];
% transmission ratios
Eta = [eta1;eta2;eta3];
% motor inertias
Jm  = [Jm1;Jm2;Jm3];

% joint angles
Q   = [q1;q2;q3];
% joint velocities
Qd  = [qd1;qd2;qd3];
% joint accelerations
Qdd = [qdd1;qdd2;qdd3];

% joint damping
B  = [b1;b2;b3];
% joint kinetic friction
Fk = [fk1;fk2;fk3];

% rigidbody centers of mass
Pc_a  = [Pc_ax Pc_ay Pc_az].';
Pc_c  = [Pc_cx Pc_cy Pc_cz].';
Pc_be = [Pc_bex Pc_bey Pc_bez].';
Pc_df = [Pc_dfx Pc_dfy Pc_dfz].';
Pc_g  = [Pc_gx Pc_gy Pc_gz].';

% rigidbody inertia tensors
Ic_a  = [Ic_axx -Ic_axy -Ic_axz;
        -Ic_axy  Ic_ayy -Ic_ayz;
        -Ic_axz -Ic_ayz  Ic_azz];
   
Ic_c  = [Ic_cxx -Ic_cxy -Ic_cxz;
        -Ic_cxy  Ic_cyy -Ic_cyz;
        -Ic_cxz -Ic_cyz  Ic_czz];
   
Ic_be = [Ic_bexx -Ic_bexy -Ic_bexz;
        -Ic_bexy  Ic_beyy -Ic_beyz;
        -Ic_bexz -Ic_beyz  Ic_bezz];
    
Ic_df = [Ic_dfxx -Ic_dfxy -Ic_dfxz;
        -Ic_dfxy  Ic_dfyy -Ic_dfyz;
        -Ic_dfxz -Ic_dfyz  Ic_dfzz];   

Ic_g  = [Ic_gxx -Ic_gxy -Ic_gxz;
        -Ic_gxy  Ic_gyy -Ic_gyz;
        -Ic_gxz -Ic_gyz  Ic_gzz];
    
%% Forward Kinematics   

% chain 1
DH_table1 = [0 0 0 q1;
             0 pi/2 0 q2;
             l2 0 0 q3-q2];
[~,T_array1] = dh2tf(DH_table1);

% chain 2       
DH_table2 = [0 0 0 q1;
             0 pi/2 0 q3-pi/2
             l3 0 0 q2-q3];
[~,T_array2] = dh2tf(DH_table2);
            
g0 = [0; 0; -g];

%% Lagrangian Dynamics

% compute energies of first chain
m1  = [m_g,m_be,m_a];
Pc1 = {Pc_g,Pc_be,Pc_a};
Ic1 = {Ic_g,Ic_be,Ic_a};
[k1, u1, vc1, w1] = compute_energies(m1,Pc1,Ic1,T_array1,Q,Qd,g0,0);
% compute energie of second chain
m2  = [m_g,m_df,m_c];
Pc2 = {Pc_g,Pc_df,Pc_c};
Ic2 = {Ic_g,Ic_df,Ic_c};
[k2, u2, vc2, w2] = compute_energies(m2,Pc2,Ic2,T_array2,Q,Qd,g0,0);
% compute MVG
[MVG,L,K,U] = dynamics_lagrangian([k1.' k2(2) k2(3)],[u1.' u2(2) u2(3)],Q,Qd,Qdd);

%% Separate MVG into M, V, and G
[M,V,G] = separate_mvg(MVG,Qdd,g);

%% Get Equation of Motion
EOM = Tau == M*Qdd + Jm.*Eta.^2.*Qdd + V + G + B.*Qd + Fk.*tanh(10 * Qd);

rb_a = parse_sw_mp('sw_mp_a.txt');
rb_c = parse_sw_mp('sw_mp_c.txt');
rb_be = parse_sw_mp('sw_mp_be.txt');
rb_df = parse_sw_mp('sw_mp_df.txt');
rb_g = parse_sw_mp('sw_mp_g.txt');

M_code = generate_code(M)
V_code = generate_code(V)
G_code = generate_code(G)