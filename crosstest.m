%test
clc
clear all
syms x1 y1 z1 phi1 theta1 psi1 u1 v1 w1 x2 y2 z2 phi2 theta2 psi2 u2 v2 w2 real;
X1=[x1 y1 z1 u1 v1 w1 phi1 theta1 psi1];
X2=[x2 y2 z2 u2 v2 w2 phi2 theta2 psi2];
X=[X1 X2];
n1=atan((y2-y1)/(x2-x1))-psi1;
Rx=((x1-x2)^2+(y1-y2)^2)^0.5;
R=((x1-x2)^2+(y1-y2)^2+(z1-z2)^2)^0.5;
n2=atan((z2-z1)/Rx)-theta1;
Z=[R];
H=jacobian(Z,X)
size(H)