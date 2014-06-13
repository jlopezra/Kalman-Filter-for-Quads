function Rij=RangeMeasurement(Xk,P,Xj)
% Xk is the vehicle state
% P is the parameter file
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
Rij=((x-xl)^2+(y-yl)^2+(z-zl)^2)^0.5;