function [H]=jacobianMeasurement(Xk,j,P)
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
% n1=atan2((yl-y),(xl-x))-psi;
% Rx=((x-xl)^2+(y-yl)^2)^0.5;
% n2=atan2((zl-z),Rx)-theta;
% 
% Zest=[pi_to_pi(n1);(n2)];
H =[                                          (2*x - 2*xl)/(2*((x - xl)^2 + (y - yl)^2 + (z - zl)^2)^(1/2)),                                          (2*y - 2*yl)/(2*((x - xl)^2 + (y - yl)^2 + (z - zl)^2)^(1/2)),                   (2*z - 2*zl)/(2*((x - xl)^2 + (y - yl)^2 + (z - zl)^2)^(1/2)), 0, 0, 0, 0,  0,  0
                                                     -(y - yl)/((x - xl)^2*((y - yl)^2/(x - xl)^2 + 1)),                                                               1/((x - xl)*((y - yl)^2/(x - xl)^2 + 1)),                                                                               0, 0, 0, 0, 0,  0, -1
 ((2*x - 2*xl)*(z - zl))/(2*((x - xl)^2 + (y - yl)^2)^(3/2)*((z - zl)^2/((x - xl)^2 + (y - yl)^2) + 1)), ((2*y - 2*yl)*(z - zl))/(2*((x - xl)^2 + (y - yl)^2)^(3/2)*((z - zl)^2/((x - xl)^2 + (y - yl)^2) + 1)), -1/(((x - xl)^2 + (y - yl)^2)^(1/2)*((z - zl)^2/((x - xl)^2 + (y - yl)^2) + 1)), 0, 0, 0, 0, -1,  0];
 
