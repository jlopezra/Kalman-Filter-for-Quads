function [n1,n2]=BearingMeasurement3d(Xk,P,Xj)
x=Xk(1);
y=Xk(2);
z=Xk(3);
u=Xk(4);
v=Xk(5);
w=Xk(6);
phi=Xk(7);
theta=Xk(8);
psi=Xk(9);
xl=Xj(1);
yl=Xj(2);
zl=Xj(3);
Rxy=((x-xl)^2+(y-yl)^2)^0.5;
n1=pi_to_pi(atan2(yl-y,xl-x)-psi);
n2=pi_to_pi(atan2(zl-z,Rxy)-theta);