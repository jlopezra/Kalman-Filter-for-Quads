function [Hb1,Hb2]=SingleBearingJacobian(Xk,P,j)
x=Xk(1);
y=Xk(2);
z=Xk(3);
u=Xk(4);
v=Xk(5);
w=Xk(6);
phi=Xk(7);
theta=Xk(8);
psi=Xk(9);
xl=P.Xl(j);
yl=P.Yl(j);
zl=P.Zl(j);
Hb1 =[ -(y - yl)/((x - xl)^2*((y - yl)^2/(x - xl)^2 + 1)), 1/((x - xl)*((y - yl)^2/(x - xl)^2 + 1)), 0, 0, 0, 0, 0, 0, -1];
 Hb2 =[ ((2*x - 2*xl)*(z - zl))/(2*((x - xl)^2 + (y - yl)^2)^(3/2)*((z - zl)^2/((x - xl)^2 + (y - yl)^2) + 1)), ((2*y - 2*yl)*(z - zl))/(2*((x - xl)^2 + (y - yl)^2)^(3/2)*((z - zl)^2/((x - xl)^2 + (y - yl)^2) + 1)), -1/(((x - xl)^2 + (y - yl)^2)^(1/2)*((z - zl)^2/((x - xl)^2 + (y - yl)^2) + 1)), 0, 0, 0, 0, -1, 0];
 