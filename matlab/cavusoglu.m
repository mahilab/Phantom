clear all
close all
clc

% syms q1 q2 q3 qd1 qd2 qd3 qdd1 qdd2 qdd3
% syms l1 l2 l3 l5 l6 M11 M22 M23 M32 M33
% syms Ibaseyy Ibasezz
% syms ma Iaxx Iayy Iazz
% syms mc Icxx Icyy Iczz
% syms mb me mbe Ibexx Ibeyy Ibezz
% syms md mf mdf Idfxx Idfyy Idfzz
% syms g

q1 = 0;
q2 = 0;
q3 = 0;

qd1=0;
qd2=0;
qd3=0;

g = 9.81;

ma = 0.0202;
Iaxx = 0.4864e-4;
Iayy = 0.001843e-4;
Iazz = 0.4864e-4;

mc = 0.0249;
Icxx = 0.959e-4;
Icyy = 0.959e-4;
Iczz = 0.0051e-4;

mbe = 0.2359;
Ibexx = 11.09e-4;
Ibeyy = 10.06e-4;
Ibezz = 0.591e-4;

mdf = 0.1906;
Idfxx = 7.11e-4;
Idfyy = 0.629e-4;
Idfzz = 6.246e-4;  

Ibaseyy = 11.87e-4;

l1 = 0.215;
l2 = 0.170;
l3 = 0.0325;
l5 = -0.0368;
l6 = 0.0527;

M11 = (1/8*(4*Iayy + 4*Iazz + 8*Ibaseyy + 4*Ibeyy + 4*Ibezz + 4*Icyy ... 
      + 4*Iczz + 4*Idfyy + 4*Idfzz + 4*l1*l1*ma + l2*l2*ma + l1*l1*mc + 4*l3*l3*mc) ...
      + 1/8*(4*Ibeyy - 4*Ibezz + 4*Icyy - 4*Iczz ...
      + l1 *l1 *(4*ma + mc))*cos(2*q2) ...
      + 1/8*(4*Iayy - 4*Iazz + 4*Idfyy - 4*Idfzz ...
      - l2*l2*ma - 4*l3*l3*mc)*cos(2*q3) + l1*(l2*ma + l3*mc)*cos(q2)*sin(q3));
  
 M22 = 1/4*(4*(Ibexx + Icxx + l1*l1*ma) + l1*l1*mc);
 
 M23 = -1/2*l1*(l2*ma + l3*mc)*sin(q2-q3);
 
 M32 = -1/2*l1*(l2*ma + l3*mc)*sin(q2-q3);
 
 M33 = 1/4*(4*Iaxx + 4*Idfxx + l2*l2*ma + 4*l3*l3*mc);
 
 M = [M11, 0, 0; 0, M22, M23; 0, M32, M33]
 
 C11 = 1/8*(-2*sin(q2)*((4*Ibeyy - 4*Ibezz + 4*Icyy - 4*Iczz ...
       + 4*l1*l1*ma + l1*l1*mc)*cos(q2) + 2*l1*(l2*ma + l3*mc)*sin(q3))*qd2 ...
       + 2*cos(q3)*(2*l1*(l2*ma+l3*mc)*cos(q2) ...
       + (-4*Iayy + 4*Iazz - 4*Idfyy + 4*Idfzz + l2*l2*ma ...
       + 4*l3*l3*mc)*sin(q3))*qd3);
 
C12 = -1/8*((4*Ibeyy - 4*Ibezz + 4*Icyy - 4*Iczz + l1*l1*(4*ma ...
       + mc))*sin(2*q2) + 4*l1*(l2*ma + l3*mc)*sin(q2)*sin(q3))*qd1;
   
C13 = -1/8*(-4*l1*(l2*ma + l3*mc)*cos(q2)*cos(q3) -(-4*Iayy ...
      + 4*Iazz - 4*Idfyy + 4*Idfzz + l2*l2*ma + 4*l3*l3*mc)*sin(2*q3))*qd1;
  
C21 = -C12;

C23 = 1/2*l1*(l2*ma+l3*mc)*cos(q2-q3)*qd3;

C31 = -C13;

C32 = 1/2*l1*(l2*ma+l3*mc)*cos(q2-q3)*qd2;

N2 = 1/2*g*(2*l1*ma + 2*l5*mbe + l1*mc)*cos(q2);

N3 = 1/2*g*(l2*ma + 2*l3*mc - 2*l6*mdf)*sin(q3);

 