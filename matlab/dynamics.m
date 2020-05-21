clear all
close all
clc

syms q1 q2 q3 l1 l2 l3 M11 M22 M23 M32 M33
syms Ibaseyy Ibasezz
syms ma Iaxx Iayy Iazz
syms mc Icxx Icyy Iczz
syms mb me mbe Ibexx Ibeyy Ibezz
syms md mf mdf Idfxx Idfyy Idfzz

M11 = (1/8*(4*Iayy + 4*Iazz + 8*Ibaseyy + 4*Ibeyy + 4*Ibezz + 4*Icyy ... 
      + 4*Iczz + 4*Idfyy + 4*Idfzz ++ 4*l1*l1*ma + l2*l2*ma + l1*l1*mc + 4*l3*l3*mc) ...
      + 1/8*(4*Ibeyy - 4*Ibezz + 4 *Icyy - 4*Iczz ...
      + l1 *l1 *(4*ma + mc))*cos(2*q2) ...
      + 1/8*(4*Iayy - 4*Iazz + 4*Idfyy - 4*Idfzz ...
      - l2*l2*ma - 4*l3*l3*mc)*cos(2*q3) + l1*(l2*ma + l3*mc)*cos(q2)*sin(q3))
  
 M22 = 1/4*(4*(Ibexx + Icxx + l1*l1*ma) + l1*l1*mc)
 
 M23 = -1/2*l1*(l2*ma + l3*mc)*sin(q2-q3)
 
 M32 = -1/2*l1*(l2*ma + l3*mc)*sin(q2-q3)
 
 M33 = 1/4*(4*Iaxx + 4*Idfxx + l2*l2*ma + 4*l3*l3*mc)